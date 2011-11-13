#include "networkmesh.h"
#include "referencemesh.h"
#include "solvers.h"
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include "eiquadprog.hpp"
#include "networkmeshrenderer.h"
#include "controller.h"

using namespace Eigen;
using namespace std;

NetworkMesh::NetworkMesh(Controller &cont) : Mesh(cont)
{
    renderer_ = new NetworkMeshRenderer(*this);
}

NetworkMesh::~NetworkMesh()
{
    delete renderer_;
}

MeshRenderer &NetworkMesh::getRenderer()
{
    return *renderer_;
}

void NetworkMesh::copyFromReferenceMesh(ReferenceMesh &rm)
{
    auto_ptr<MeshLock> rml = rm.acquireMesh();
    auto_ptr<MeshLock> ml = acquireMesh();
    copyMesh(rm.getMesh());
    subdreference_ = rm.getMesh();
}

void NetworkMesh::saveSubdivisionReference()
{
    auto_ptr<MeshLock> ml = acquireMesh();
    subdreference_ = mesh_;
}

double NetworkMesh::calculateEquilibriumViolation()
{
    auto_ptr<MeshLock> ml = acquireMesh();
    double result = 0;
    int n = mesh_.n_vertices();
    for(int i=0; i<n; i++)
    {
        MyMesh::VertexHandle vh = mesh_.vertex_handle(i);
        if(mesh_.data(vh).pinned())
            continue;

        MyMesh::Point center = mesh_.point(vh);
        MyMesh::Point sum;
        sum[0] = sum[1] = sum[2] = 0;
        for(MyMesh::VertexOHalfedgeIter voh = mesh_.voh_iter(vh); voh; ++voh)
        {
            MyMesh::HalfedgeHandle heh = voh;
            MyMesh::EdgeHandle eh = mesh_.edge_handle(heh);
            double weight = mesh_.data(eh).weight();
            MyMesh::Point adj = mesh_.point(mesh_.to_vertex_handle(heh));
            sum += (center-adj)*weight;
        }
        sum[1] += mesh_.data(vh).load();
        result += sum[0]*sum[0] + sum[1]*sum[1] + sum[2]*sum[2];
    }
    return sqrt(result);
}

double NetworkMesh::computeBestWeights(double maxweight)
{
    auto_ptr<MeshLock> ml = acquireMesh();

    int n = mesh_.n_vertices();
    int e = mesh_.n_edges();

    if(n == 0 || e == 0)
        return 0;

    int interiorn=0;
    for(MyMesh::VertexIter it = mesh_.vertices_begin(); it != mesh_.vertices_end(); ++it)
    {
        if(!mesh_.data(it.handle()).pinned())
            interiorn++;
    }

    int boundarye=0;
    for(MyMesh::EdgeIter it = mesh_.edges_begin(); it != mesh_.edges_end(); ++it)
    {
        if(edgePinned(it.handle()) || mesh_.data(it).is_crease())
            boundarye++;
    }

    DynamicSparseMatrix<double> Md(3*interiorn+boundarye, e);
    VectorXd rhs(3*interiorn+boundarye);
    rhs.setZero();

    // min || \sum_{j~i} w_ij (q_i-q_j) + F_i ||^2 + || w_ij ||^2 (boundary) s.t. w_ij >= 0

    int row=0;
    for(int i=0; i<n; i++)
    {
        MyMesh::VertexHandle vh = mesh_.vertex_handle(i);
        if(mesh_.data(vh).pinned())
            continue;

        MyMesh::Point center = mesh_.point(vh);

        for(MyMesh::VertexOHalfedgeIter voh = mesh_.voh_iter(vh); voh; ++voh)
        {

            MyMesh::HalfedgeHandle heh = voh;
            MyMesh::VertexHandle tov = mesh_.to_vertex_handle(heh);
            MyMesh::EdgeHandle eh = mesh_.edge_handle(heh);
            int eidx = eh.idx();
            MyMesh::Point adj = mesh_.point(tov);
            Md.coeffRef(row, eidx) += center[0]-adj[0];
            Md.coeffRef(row+1, eidx) += (center[1]-adj[1]);
            Md.coeffRef(row+2, eidx) += center[2]-adj[2];
        }
        rhs[row] = 0;
        rhs[row+1] = -mesh_.data(vh).load();
        rhs[row+2] = 0;
        row += 3;
    }
    for(int i=0; i<e; i++)
    {
        MyMesh::EdgeHandle eh = mesh_.edge_handle(i);
        if(edgePinned(eh))
        {
            Md.coeffRef(row, i) = 1.0;
            row++;
        }
        else if(mesh_.data(eh).is_crease())
        {
            Md.coeffRef(row, i) = 1.0;
            rhs[row] = mesh_.data(eh).crease_value();
            row++;
        }
    }
    assert(row == 3*interiorn + boundarye);

    VectorXd result(e);
    VectorXd lb(e);
    VectorXd ub(e);
    lb.setZero();
    for(int i=0; i<e; i++)
    {
        ub[i] = maxweight;
        MyMesh::EdgeHandle eh = mesh_.edge_handle(i);
        result[i] = mesh_.data(eh).weight();
    }

    SparseMatrix<double> M(Md);
    int oldid = getMeshID();
    ml.reset();

    cont_.getSolvers().solveBCLS(M, rhs, lb, ub, result);
    double residual = std::numeric_limits<double>::infinity();

    ml = acquireMesh();
    if(getMeshID() == oldid)
    {
        for(int i=0; i<e; i++)
        {
            if(result[i] < 0 || isnan(result[i]))
                result[i] = 0;
            MyMesh::EdgeHandle eh = mesh_.edge_handle(i);
            if(edgePinned(eh))
                result[i] = 0;
            else if(mesh_.data(eh).is_crease())
                result[i] = mesh_.data(eh).crease_value();
            mesh_.data(eh).set_weight(result[i]);
        }

        fixBadVertices();
        residual = calculateEquilibriumViolation();
        invalidateMesh();
    }

    return residual;
}

double NetworkMesh::computeBestPositionsTangentLS(double alpha, double beta)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    int n = mesh_.n_vertices();

    VectorXd q0(3*n);

    for(int i=0; i<n; i++)
    {
        const MyMesh::Point &pt = mesh_.point(mesh_.vertex_handle(i));
        for(int j=0; j<3; j++)
            q0[3*i+j] = pt[j];
    }

    DynamicSparseMatrix<double> Md(3*n,3*n);
    for(int i=0; i<n; i++)
    {
        MyMesh::Point normal;
        mesh_.calc_vertex_normal_correct(mesh_.vertex_handle(i),normal);
        for(int j=0; j<3; j++)
        {
            for(int k=0; k<3; k++)
            {
                Md.coeffRef(3*i+j,3*i+k) = normal[j]*normal[k];
            }
        }
    }

    VectorXd rhs = Md*q0;

    for(int i=0; i<n; i++)
    {
        MyMesh::Point pt = mesh_.point(mesh_.vertex_handle(i));
        Vector3d ptv(pt[0],pt[1],pt[2]);
        Vector3d projpt = approximateClosestPoint(subdreference_, ptv);
        for(int j=0; j<3; j++)
        {
            rhs[3*i+j] += alpha*(projpt[j]);
            Md.coeffRef(3*i+j,3*i+j) += alpha;
        }
    }


    DynamicSparseMatrix<double> CEd(3*n, 3*n);
    VectorXd ce0(3*n);
    ce0.setZero();
    for(int i=0; i<n; i++)
    {
        MyMesh::VertexHandle vh = mesh_.vertex_handle(i);
        if(mesh_.data(vh).pinned())
        {
            for(int j=0; j<3; j++)
            {
                MyMesh::Point pt = mesh_.point(vh);
                CEd.coeffRef(3*i+j,3*i+j) = 1.0;
                ce0[3*i+j] = -pt[j];
            }
        }
        else
        {
            for(MyMesh::VertexOHalfedgeIter voh = mesh_.voh_iter(vh); voh; ++voh)
            {
                MyMesh::HalfedgeHandle heh = voh;
                MyMesh::EdgeHandle eh = mesh_.edge_handle(heh);
                double weight = mesh_.data(eh).weight();
                int vidx = mesh_.to_vertex_handle(heh).idx();
                for(int j=0; j<3; j++)
                {
                    CEd.coeffRef(3*i+j, 3*i+j) += weight;
                    CEd.coeffRef(3*vidx+j, 3*i+j) -= weight;
                }
            }
            ce0[3*i+0] = 0;
            ce0[3*i+1] = mesh_.data(vh).load();
            ce0[3*i+2] = 0;
        }
    }
    VectorXd result = q0;
    rhs -= beta*CEd*ce0;
    Md += beta*CEd*CEd.transpose();
    SparseMatrix<double> M(Md);

    int oldid = getMeshID();
    ml.reset();

    cont_.getSolvers().linearSolveCG(M, rhs, result);
    double residual = std::numeric_limits<double>::infinity();

    ml = acquireMesh();
    if(oldid == getMeshID())
    {
        for(int i=0; i<n; i++)
        {
            MyMesh::VertexHandle vh = mesh_.vertex_handle(i);
            MyMesh::Point &pt = mesh_.point(vh);
            for(int j=0; j<3; j++)
                pt[j] = result[3*i+j];
        }

        residual = calculateEquilibriumViolation();
        invalidateMesh();
    }
    return residual;
}


double NetworkMesh::computeWeightsOnPlane(ReferenceMesh &rm, double sum)
{
    auto_ptr<MeshLock> rml = rm.acquireMesh();
    auto_ptr<MeshLock> ml = acquireMesh();

    copyFromReferenceMesh(rm);
    int n = mesh_.n_vertices();
    if(n == 0)
        return 0;

    int interiorn=0;
    for(int i=0; i<n; i++)
    {
        MyMesh::VertexHandle vh = mesh_.vertex_handle(i);
        if(!mesh_.data(vh).pinned())
            interiorn++;
    }

    int e = mesh_.n_edges();

    int boundarye=0;
    for(int i=0; i<e; i++)
    {
        MyMesh::EdgeHandle eh = mesh_.edge_handle(i);
        if(edgePinned(eh))
            boundarye++;
    }

    MatrixXd M(2*interiorn + boundarye + 1,e);
    VectorXd rhs(2*interiorn + boundarye + 1);
    M.setZero();
    rhs.setZero();

    // min (w_ij - w^0_ij)^2
    // st
    // \sum_{i~j} w_ij (q_i-q_j) = 0 (i on interior)
    // w_ij = 0 (ij boundary)
    // \sum w_ij = sum
    // w_ij >= 0

    int row = 0;
    for(int i=0; i<n; i++)
    {
        MyMesh::VertexHandle vh = mesh_.vertex_handle(i);
        if(mesh_.data(vh).pinned())
            continue;

        MyMesh::Point center = mesh_.point(vh);
        for(MyMesh::VertexOHalfedgeIter voh = mesh_.voh_iter(vh); voh; ++voh)
        {
            MyMesh::VertexHandle tov = mesh_.to_vertex_handle(voh);
            MyMesh::HalfedgeHandle heh = voh;
            MyMesh::EdgeHandle eh = mesh_.edge_handle(heh);
            int col = eh.idx();

            MyMesh::Point adj = mesh_.point(tov);
            M(row, col)   += center[0] - adj[0];
            M(row+1, col) += center[2] - adj[2];
        }
        row+=2;
    }

    for(int i=0; i<e; i++)
    {
        MyMesh::EdgeHandle eh = mesh_.edge_handle(i);
        if(edgePinned(eh))
        {
            M(row,i) = 1.0;
            row++;
        }
    }

    for(int i=0; i<e; i++)
    {
        {
            M(2*interiorn + boundarye,i) = 1.0;
        }
    }

    rhs[2*interiorn + boundarye] = sum;

    MatrixXd CI(e,e);
    CI.setZero();
    for(int i=0; i<e; i++)
    {
        CI(i,i)=1.0;
    }
    VectorXd ci0(e);
    ci0.setZero();

    VectorXd result(e);

    MatrixXd G(e,e);
    G.setZero();
    for(int i=0; i<e; i++)
        G(i,i) = 1.0;
    VectorXd g0(e);
    g0.setZero();
    // initial guess: uniform weights
    double initialweight = sum/(e-boundarye);
    for(int i=0; i<e; i++)
    {
        MyMesh::EdgeHandle eh = mesh_.edge_handle(i);
        if(!edgePinned(eh))
            g0[i] = initialweight;
    }

    MatrixXd CE = M.transpose();
    VectorXd ce0 = -rhs;

    solve_quadprog(G, g0, CE, ce0, CI, ci0, result);

    for(int i=0; i<e; i++)
    {
        if(result[i] < 0 || isnan(result[i]))
            result[i] = 0;
    }

    double residual = (M*result - rhs).norm();

    for(int i=0; i<e; i++)
    {
        double weight = result[i];
        MyMesh::EdgeHandle eh = mesh_.edge_handle(i);
        mesh_.data(eh).set_weight(weight);
    }

    return residual;
}

double NetworkMesh::updateHeights()
{
    auto_ptr<MeshLock> ml = acquireMesh();
    int n = mesh_.n_vertices();
    if(n == 0)
    {
        return 0;
    }

    MatrixXd M(n,n);
    M.setZero();
    VectorXd rhs(n);
    rhs.setZero();

    // \sum_{i~j} w_ij (z_i - z_j) = -1 (i interior)
    // z_i = 0 (i boundary)

    for(int i=0; i<n; i++)
    {
        MyMesh::VertexHandle vh = mesh_.vertex_handle(i);
        if(mesh_.data(vh).pinned() || isBadVertex(vh))
        {
            M(i, i) = 1.0;
        }
        else
        {
            for(MyMesh::VertexOHalfedgeIter voh = mesh_.voh_iter(vh); voh; ++voh)
            {
                MyMesh::VertexHandle tov = mesh_.to_vertex_handle(voh);
                MyMesh::HalfedgeHandle heh = voh;
                MyMesh::EdgeHandle eh = mesh_.edge_handle(heh);

                int tovidx = tov.idx();
                double weight = mesh_.data(eh).weight();
                M(i, i) += weight;
                M(i, tovidx) -= weight;
            }
            rhs[i] = mesh_.data(vh).load();
        }
    }

    VectorXd heights = M.fullPivLu().solve(rhs);

    double residual = (M*heights-rhs).norm();

    for(int i=0; i<n; i++)
    {
        MyMesh::VertexHandle vh = mesh_.vertex_handle(i);
        mesh_.point(vh)[1] = -heights[i];
    }

    return residual;
}

bool NetworkMesh::isBadVertex(MyMesh::VertexHandle vert)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    if(mesh_.data(vert).pinned())
    {
        return false;
    }

    for(MyMesh::VertexOHalfedgeIter vhe = mesh_.voh_iter(vert); vhe; ++vhe)
    {
        MyMesh::HalfedgeHandle heh = vhe;
        MyMesh::EdgeHandle eh = mesh_.edge_handle(heh);
        if(mesh_.data(eh).weight() > 1e-6)
        {
            return false;
        }
    }
    return true;
}

void NetworkMesh::projectOnto(const MyMesh &m)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    for(MyMesh::VertexIter v = mesh_.vertices_begin(); v != mesh_.vertices_end(); ++v)
    {
        MyMesh::Point &pt = mesh_.point(v);
        Vector3d curpos(pt[0],pt[1],pt[2]);
        Vector3d newpos = approximateClosestPoint(m, curpos);
        for(int j=0; j<3; j++)
            pt[j] = newpos[j];
    }
    invalidateMesh();
}

void NetworkMesh::projectOntoReference(ReferenceMesh &rm)
{
    auto_ptr<MeshLock> rml = rm.acquireMesh();
    projectOnto(rm.getMesh());
}

void NetworkMesh::fixBadVertices()
{
    auto_ptr<MeshLock> ml = acquireMesh();
    for(MyMesh::VertexIter v = mesh_.vertices_begin(); v != mesh_.vertices_end(); ++v)
    {
        if(isBadVertex(v))
        {
                //removeVertex(v);
            mesh_.delete_vertex(v, true);
        }
    }
    mesh_.garbage_collection();
}

double NetworkMesh::computeCotanWeights()
{
    auto_ptr<MeshLock> ml = acquireMesh();
    for(MyMesh::EdgeIter ei = mesh_.edges_begin(); ei != mesh_.edges_end(); ++ei)
    {
        if(edgePinned(ei.handle()))
        {
            mesh_.data(ei).set_weight(0);
            continue;
        }
        MyMesh::HalfedgeHandle heh[2];
        heh[0] = mesh_.halfedge_handle(ei,0);
        heh[1] = mesh_.halfedge_handle(ei,1);
        assert(heh[0].is_valid() && heh[1].is_valid());
        double tot = 0;
        for(int i=0; i<2; i++)
        {
            MyMesh::HalfedgeHandle next = mesh_.next_halfedge_handle(heh[i]);
            MyMesh::HalfedgeHandle prev = mesh_.prev_halfedge_handle(heh[i]);
            MyMesh::Point a1 = mesh_.point(mesh_.to_vertex_handle(next));
            MyMesh::Point a2 = mesh_.point(mesh_.from_vertex_handle(next));
            MyMesh::Point b1 = mesh_.point(mesh_.from_vertex_handle(prev));
            MyMesh::Point b2 = mesh_.point(mesh_.to_vertex_handle(prev));
            Vector2d a;
            Vector2d b;
            a[0] = a2[0] - a1[0];
            a[1] = a2[2] - a1[2];
            b[0] = b2[0] - b1[0];
            b[1] = b2[2] - b1[2];
            Matrix2d Lambda;
            double x = a1[0];
            double y = a1[2];
            Lambda << 1.0/(y*y),0, 0, 1.0/(x*x);
            double num = a.dot(Lambda * b);
            double denom = fabs(a[0]*b[1]- a[1]*b[0]);
            tot += num/denom;
        }
        tot /= 10*2.0;
        mesh_.data(ei).set_weight(tot);
    }
    return 0;
}
