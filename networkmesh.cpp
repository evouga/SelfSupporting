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
    copyMesh(rm.getMesh());
    rm.releaseMesh();
}

double NetworkMesh::calculateEquilibriumViolation()
{
    lockMesh();
    double result = 0;
    int n = mesh_.n_vertices();
    for(int i=0; i<n; i++)
    {
        MyMesh::VertexHandle vh = mesh_.vertex_handle(i);
        if(mesh_.is_boundary(vh))
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
    releaseMesh();
    return sqrt(result);
}

double NetworkMesh::computeBestWeights()
{
    lockMesh();
    int n = mesh_.n_vertices();
    int e = mesh_.n_edges();

    int interiorn=0;
    for(MyMesh::VertexIter it = mesh_.vertices_begin(); it != mesh_.vertices_end(); ++it)
    {
        if(!mesh_.is_boundary(it))
            interiorn++;
    }

    int boundarye=0;
    for(MyMesh::EdgeIter it = mesh_.edges_begin(); it != mesh_.edges_end(); ++it)
    {
        if(mesh_.is_boundary(it))
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
        if(mesh_.is_boundary(vh))
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
        if(mesh_.is_boundary(eh))
        {
            Md.coeffRef(row, i) = 1.0;
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
        ub[i] = std::numeric_limits<double>::infinity();
        MyMesh::EdgeHandle eh = mesh_.edge_handle(i);
        result[i] = mesh_.data(eh).weight();
    }

    SparseMatrix<double> M(Md);
    int oldid = getMeshID();
    releaseMesh();

    cont_.getSolvers().solveBCLS(M, rhs, lb, ub, result);
    double residual = std::numeric_limits<double>::infinity();

    lockMesh();
    if(getMeshID() == oldid)
    {
        for(int i=0; i<e; i++)
        {
            if(result[i] < 0 || isnan(result[i]))
                result[i] = 0;
            MyMesh::EdgeHandle eh = mesh_.edge_handle(i);
            if(mesh_.is_boundary(eh))
                result[i] = 0;
            mesh_.data(eh).set_weight(result[i]);
        }

        fixBadVertices();
        residual = calculateEquilibriumViolation();
        invalidateMesh();
    }

    releaseMesh();
    return residual;
}

double NetworkMesh::computeBestPositionsTangentLS(ReferenceMesh &rm, double alpha, double beta)
{
    rm.lockMesh();
    lockMesh();
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
        Vector3d projpt = rm.approximateClosestPoint(ptv);
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
        if(mesh_.is_boundary(vh))
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
    releaseMesh();
    rm.releaseMesh();

    cont_.getSolvers().linearSolveCG(M, rhs, result);
    double residual = std::numeric_limits<double>::infinity();

    lockMesh();
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
    releaseMesh();

    return residual;
}


double NetworkMesh::computeWeightsOnPlane(ReferenceMesh &rm, double sum)
{
    rm.lockMesh();
    lockMesh();

    copyFromReferenceMesh(rm);
    int n = mesh_.n_vertices();
    if(n == 0)
        return 0;

    int interiorn=0;
    for(int i=0; i<n; i++)
    {
        MyMesh::VertexHandle vh = mesh_.vertex_handle(i);
        if(!mesh_.is_boundary(vh))
            interiorn++;
    }

    int e = mesh_.n_edges();

    int boundarye=0;
    for(int i=0; i<e; i++)
    {
        MyMesh::EdgeHandle eh = mesh_.edge_handle(i);
        if(mesh_.is_boundary(eh))
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
        if(mesh_.is_boundary(vh))
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
        if(mesh_.is_boundary(eh))
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
        if(!mesh_.is_boundary(eh))
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

    releaseMesh();
    rm.releaseMesh();

    return residual;
}

double NetworkMesh::updateHeights()
{
    lockMesh();
    int n = mesh_.n_vertices();
    if(n == 0)
    {
        releaseMesh();
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
        if(mesh_.is_boundary(vh) || isBadVertex(vh))
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

    releaseMesh();
    return residual;
}

bool NetworkMesh::isBadVertex(MyMesh::VertexHandle vert)
{
    lockMesh();
    if(mesh_.is_boundary(vert))
    {
        releaseMesh();
        return false;
    }

    for(MyMesh::VertexOHalfedgeIter vhe = mesh_.voh_iter(vert); vhe; ++vhe)
    {
        MyMesh::HalfedgeHandle heh = vhe;
        MyMesh::EdgeHandle eh = mesh_.edge_handle(heh);
        if(mesh_.data(eh).weight() > 1e-6)
        {
            releaseMesh();
            return false;
        }
    }
    releaseMesh();
    return true;
}

void NetworkMesh::subdivide()
{
    lockMesh();
    int e = mesh_.n_edges();
    int n = mesh_.n_vertices();
    int f = mesh_.n_faces();

    MyMesh newmesh;

    // add new face vertices
    for(int i=0; i<f; i++)
    {
        MyMesh::FaceHandle fh = mesh_.face_handle(i);
        MyMesh::Point centroid;
        mesh_.calc_face_centroid(fh, centroid);
        newmesh.add_vertex(centroid);
    }

    // add new edge vertices
    for(int i=0; i<e; i++)
    {
        MyMesh::EdgeHandle eh = mesh_.edge_handle(i);
        //TODO Fix boundary case
        MyMesh::Point midp = computeEdgeMidpoint(eh);
        if(!mesh_.is_boundary(eh))
        {
            MyMesh::HalfedgeHandle heh1 = mesh_.halfedge_handle(eh, 0);
            MyMesh::HalfedgeHandle heh2 = mesh_.halfedge_handle(eh, 1);
            MyMesh::FaceHandle fh1 = mesh_.face_handle(heh1);
            MyMesh::FaceHandle fh2 = mesh_.face_handle(heh2);
            MyMesh::Point facept1 = newmesh.point(newmesh.vertex_handle(fh1.idx()));
            MyMesh::Point facept2 = newmesh.point(newmesh.vertex_handle(fh2.idx()));
            midp += (facept1+facept2)*0.5;
            midp *= 0.5;
        }
        newmesh.add_vertex(midp);
    }

    // add (modified) original vertices
    for(int i=0; i<n; i++)
    {
        MyMesh::VertexHandle vh = mesh_.vertex_handle(i);
        MyMesh::Point newpt(0,0,0);
        if(mesh_.is_boundary(vh))
        {
            //TODO fix boundary case
            for(MyMesh::VertexEdgeIter ve = mesh_.ve_iter(vh); ve; ++ve)
            {
                if(mesh_.is_boundary(ve))
                {
                    newpt += computeEdgeMidpoint(ve);
                }
            }
            newpt *= 0.5;
            newpt += mesh_.point(vh);
            newpt *= 0.5;
        }
        else
        {
            int valence = mesh_.valence(vh);
            MyMesh::Point F(0,0,0);
            MyMesh::Point R(0,0,0);
            for(MyMesh::VertexEdgeIter ve = mesh_.ve_iter(vh); ve; ++ve)
            {
                R += computeEdgeMidpoint(ve);
            }
            for(MyMesh::VertexFaceIter vf = mesh_.vf_iter(vh); vf; ++vf)
            {
                MyMesh::FaceHandle fh = vf;
                F += newmesh.point(newmesh.vertex_handle(fh.idx()));
            }
            R /= valence;
            F /= valence;
            newpt = mesh_.point(vh)*(valence-3);
            newpt += R*2.0;
            newpt += F;
            newpt /= valence;
        }
        newmesh.add_vertex(newpt);
    }

    assert((int)newmesh.n_vertices() == f + e + n);

    // Now rebuilt mesh combinatorics
    for(int i=0; i<f; i++)
    {
        MyMesh::FaceHandle fh = mesh_.face_handle(i);
        MyMesh::VertexHandle center = newmesh.vertex_handle(i);
        // each face is now several quad faces
        for(MyMesh::FaceHalfedgeIter fhe = mesh_.fh_iter(fh); fhe; ++fhe)
        {
            vector<MyMesh::VertexHandle> newface;
            MyMesh::VertexHandle tovert = mesh_.to_vertex_handle(fhe);
            MyMesh::HalfedgeHandle heh = fhe;
            MyMesh::EdgeHandle edge1 = mesh_.edge_handle(heh);
            MyMesh::FaceHalfedgeIter fhe2 = fhe;
            if(!++fhe2)
                fhe2 = mesh_.fh_iter(fh);
            MyMesh::HalfedgeHandle heh2 = fhe2;
            MyMesh::EdgeHandle edge2 = mesh_.edge_handle(heh2);

            // some nice gymnastics
            newface.push_back(center);
            newface.push_back(newmesh.vertex_handle(f+edge1.idx()));
            newface.push_back(newmesh.vertex_handle(f+e+tovert.idx()));
            newface.push_back(newmesh.vertex_handle(f+edge2.idx()));
            newmesh.add_face(newface);
        }
    }

    mesh_ = newmesh;
    invalidateMesh();
    releaseMesh();
}

void NetworkMesh::projectOntoReference(ReferenceMesh &rm)
{
    rm.lockMesh();
    lockMesh();
    for(MyMesh::VertexIter v = mesh_.vertices_begin(); v != mesh_.vertices_end(); ++v)
    {
        MyMesh::Point &pt = mesh_.point(v);
        Vector3d curpos(pt[0],pt[1],pt[2]);
        Vector3d newpos = rm.approximateClosestPoint(curpos);
        for(int j=0; j<3; j++)
            pt[j] = newpos[j];
    }
    invalidateMesh();
    releaseMesh();
    rm.releaseMesh();
}

void NetworkMesh::fixBadVertices()
{
    lockMesh();
    for(MyMesh::VertexIter v = mesh_.vertices_begin(); v != mesh_.vertices_end(); ++v)
    {
        if(isBadVertex(v))
        {
            mesh_.delete_vertex(v, true);
        }
    }
    mesh_.garbage_collection();
    releaseMesh();
}
