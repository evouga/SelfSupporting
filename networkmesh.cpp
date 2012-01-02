#include "networkmesh.h"
#include "referencemesh.h"
#include "solvers.h"
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include "eiquadprog.hpp"
#include "networkmeshrenderer.h"
#include "controller.h"
#include <set>
#include "unsupported/Eigen/IterativeSolvers"
#include <fstream>

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

double NetworkMesh::calculateEquilibriumViolation(MyMesh::VertexHandle vh)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    if(mesh_.data(vh).pinned())
            return 0;

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
    return sqrt( sum[0]*sum[0] + sum[1]*sum[1] + sum[2]*sum[2] );
}

double NetworkMesh::calculateEquilibriumViolation()
{
    auto_ptr<MeshLock> ml = acquireMesh();
    double result = 0;
    int n = mesh_.n_vertices();
    for(int i=0; i<n; i++)
    {
        MyMesh::VertexHandle vh = mesh_.vertex_handle(i);
        double viol = calculateEquilibriumViolation(vh);
        mesh_.data(vh).set_violation(viol);
        result += viol*viol;
    }
    return sqrt(result);
}

double NetworkMesh::computeBestWeights(double maxstress, double thickness)
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
        if(edgePinned(it.handle()))
            boundarye++;
    }

    if(interiorn == 0 || boundarye == 0)
        return 0;

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
            rhs[row] = 1.0;
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
        MyMesh::EdgeHandle eh = mesh_.edge_handle(i);
        double maxweight = numeric_limits<double>::infinity();
        if(maxstress!= numeric_limits<double>::infinity())
        {
            double areaofinfluence = edgeArea(eh);
            double e2 = mesh_.calc_edge_sqr_length(eh);
            maxweight = maxstress*thickness*areaofinfluence/e2;
            maxweight /= 8;
        }
        ub[i] = maxweight;
        lb[i] = 0.0;
        result[i] = mesh_.data(eh).weight();
    }

    SparseMatrix<double> M(Md);
    int oldid = getMeshID();
    ml.reset();

    //cout << 3*interiorn << " + " << boundarye << " nz: " << M.nonZeros() << " ( " << double(M.nonZeros())/(M.rows()*M.cols()) * 100.0 << "%)" << endl;

/*    DynamicSparseMatrix<double> MTM(M.transpose()*M);
    SparseMatrix<double> MTMd(MTM);
    VectorXd Mrhs = M.transpose()*rhs;
    cont_.getSolvers().linearSolveCG(MTMd, Mrhs, result);
    bool resultgood = true;
    for(int i=0; i<e; i++)
        if(result[i] < -1e-4)
        {
            cout << "BCLS " << result[i] << endl;
            resultgood = false;
            break;
        }
    if(!resultgood)
    {*/
        cont_.getSolvers().solveBCLS(M, rhs, lb, ub, result);
    /*}
    else
        cout << "no BCLS" << endl;*/

    ml = acquireMesh();
    if(getMeshID() == oldid)
    {
        for(int i=0; i<e; i++)
        {
            if(result[i] < lb[i] || isnan(result[i]))
                result[i] = lb[i];
            MyMesh::EdgeHandle eh = mesh_.edge_handle(i);
            if(edgePinned(eh))
                result[i] = 0;
            mesh_.data(eh).set_weight(result[i]);
        }
        //fixBadVertices();
        invalidateMesh();
    }
    return calculateEquilibriumViolation();
}

double NetworkMesh::computeBestPositionsTangentLS(double alpha, double beta, bool planarity)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    cout << computeBoundingCircle(computeCentroid()) << endl;
    double fac = 1.0/calculateEquilibriumViolation();
    fac *= fac;
    beta *= fac;

    int n = mesh_.n_vertices();

    if(n==0)
        return 0;

    map<int, int> vidx2midx;
    map<int, int> midx2vidx;
    int nummdofs=0;

    for(int i=0; i<n; i++)
    {
        MyMesh::VertexHandle vh = mesh_.vertex_handle(i);
        if(!mesh_.data(vh).pinned() && !mesh_.data(vh).anchored())
        {
            vidx2midx[i] = nummdofs;
            midx2vidx[nummdofs] = i;
            nummdofs++;
        }
    }

    VectorXd q0(3*nummdofs);

    for(int i=0; i<n; i++)
    {
        MyMesh::VertexHandle vh = mesh_.vertex_handle(i);
        if(!mesh_.data(vh).pinned() && !mesh_.data(vh).anchored())
        {
            const MyMesh::Point &pt = mesh_.point(vh);
            for(int j=0; j<3; j++)
                q0[3*vidx2midx[i]+j] = pt[j];
        }
    }

    DynamicSparseMatrix<double> L(3*n,3*nummdofs);
    VectorXd Lrhs(3*n);
    Lrhs.setZero();

    for(int i=0; i<n; i++)
    {

        MyMesh::VertexHandle vh = mesh_.vertex_handle(i);
        if(!mesh_.data(vh).pinned())
        {
            int valence = mesh_.valence(vh);
            if(!mesh_.data(vh).anchored())
            {
                for(int j=0; j<3; j++)
                    L.coeffRef(3*i+j, 3*vidx2midx[i]+j) = 1.0;
            }
            for(MyMesh::VertexVertexIter vv = mesh_.vv_iter(vh); vv; ++vv)
            {
                MyMesh::VertexHandle adj = vv;
                int adjidx = adj.idx();
                if(!mesh_.data(adj).pinned() && !mesh_.data(adj).anchored())
                {
                    for(int j=0; j<3; j++)
                        L.coeffRef(3*i+j, 3*vidx2midx[adjidx]+j) = -1.0/valence;
                }
                else
                {
                    MyMesh::Point pt = mesh_.point(adj);
                    for(int j=0; j<3; j++)
                        Lrhs[3*i+j] += 1.0/valence * pt[j];
                }
            }
        }
    }

    DynamicSparseMatrix<double> Md(3*nummdofs,3*nummdofs);
    for(int i=0; i<n; i++)
    {
        MyMesh::VertexHandle vh = mesh_.vertex_handle(i);
        if(!mesh_.data(vh).pinned() && !mesh_.data(vh).anchored())
        {
            MyMesh::Point normal;
            mesh_.calc_vertex_normal_correct(vh,normal);
            for(int j=0; j<3; j++)
            {
                for(int k=0; k<3; k++)
                {
                    Md.coeffRef(3*vidx2midx[i]+j,3*vidx2midx[i]+k) = normal[j]*normal[k];
                }
            }
        }
    }

    Md /= beta;
    VectorXd rhs = Md*q0;

    for(int i=0; i<n; i++)
    {
        MyMesh::VertexHandle vh = mesh_.vertex_handle(i);
        if(!mesh_.data(vh).pinned() && !mesh_.data(vh).anchored())
        {
            MyMesh::Point pt = mesh_.point(mesh_.vertex_handle(i));

            Vector3d ptv(pt[0],pt[1],pt[2]);
            Vector3d projpt = approximateClosestPoint(subdreference_, ptv);
            //Vector3d projpt = approximateClosestZParallel(subdreference_, ptv);
            for(int j=0; j<3; j++)
            {
                rhs[3*vidx2midx[i]+j] += alpha/beta*(projpt[j]);
                Md.coeffRef(3*vidx2midx[i]+j,3*vidx2midx[i]+j) += alpha/beta;
            }
        }
    }

    int numunpinned = 0;
    for(MyMesh::VertexIter vi = mesh_.vertices_begin(); vi != mesh_.vertices_end(); ++vi)
    {
        if(!mesh_.data(vi.handle()).pinned())
            numunpinned++;
    }

    DynamicSparseMatrix<double> CEd(3*nummdofs, 3*numunpinned);
    VectorXd cerhs(3*numunpinned);
    cerhs.setZero();
    int row = 0;
    for(int i=0; i<n; i++)
    {
        MyMesh::VertexHandle vh = mesh_.vertex_handle(i);
        if(!mesh_.data(vh).pinned())
        {
            for(MyMesh::VertexOHalfedgeIter voh = mesh_.voh_iter(vh); voh; ++voh)
            {
                MyMesh::HalfedgeHandle heh = voh;
                MyMesh::EdgeHandle eh = mesh_.edge_handle(heh);
                double weight = mesh_.data(eh).weight();
                MyMesh::VertexHandle adj = mesh_.to_vertex_handle(heh);
                int vidx = adj.idx();
                for(int j=0; j<3; j++)
                {
/*                    if(mesh_.data(vh).anchored())
                    {
                        MyMesh::Point pt = mesh_.point(vh);
                        ce0[3*row+j] -= weight*pt[j];
                    }
                    else
                        CEd.coeffRef(3*vidx2midx[i]+j, 3*row+j) += weight;*/
                    addToStrippedMatrix(CEd, cerhs, i, j, 3*row+j,weight,vidx2midx);

/*                    if(!mesh_.data(adj).pinned() && !mesh_.data(adj).anchored())
                        CEd.coeffRef(3*vidx2midx[vidx]+j, 3*row+j) -= weight;
                    else
                    {
                        MyMesh::Point adjpt = mesh_.point(adj);
                        ce0[3*row+j] += weight*adjpt[j];
                    }*/
                    addToStrippedMatrix(CEd, cerhs, vidx, j, 3*row+j, -weight, vidx2midx);
                }
            }
            cerhs[3*row+1] -= mesh_.data(vh).load();
            row++;
        }
    }
    assert(row == numunpinned);

    int numplanarity = 0;
    for(MyMesh::FaceIter fi = mesh_.faces_begin(); fi != mesh_.faces_end(); ++fi)
    {
        if(numFaceVerts(mesh_, fi.handle()) == 4)
        {
            numplanarity++;
        }
    }

    DynamicSparseMatrix<double> Pd(3*nummdofs, numplanarity);
    VectorXd prhs(numplanarity);
    prhs.setZero();

    row=0;
    for(MyMesh::FaceIter fi = mesh_.faces_begin(); fi != mesh_.faces_end(); ++fi)
    {
        if(numFaceVerts(mesh_, fi.handle()) == 4)
        {
            MyMesh::FaceHalfedgeIter fhi = mesh_.fh_iter(fi.handle());
            MyMesh::HalfedgeHandle prev = fhi.handle();

            MyMesh::HalfedgeHandle cur = mesh_.next_halfedge_handle(prev);
            double sumtheta = 0;
            for(int i=0; i<4; i++)
            {
                MyMesh::Point e1p1, e1p2, e2p1, e2p2;
                int curtoidx = mesh_.to_vertex_handle(cur).idx();
                int curfromidx = mesh_.from_vertex_handle(cur).idx();
                assert(curfromidx == mesh_.to_vertex_handle(prev).idx());
                int prevfromidx = mesh_.from_vertex_handle(prev).idx();

                e2p2 = mesh_.point(mesh_.to_vertex_handle(cur));
                e2p1 = mesh_.point(mesh_.from_vertex_handle(cur));
                e1p1 = e2p1;
                e1p2 = mesh_.point(mesh_.from_vertex_handle(prev));

                Vector3d e1,e2;
                for(int j=0; j<3; j++)
                {
                    e1[j] = e1p2[j]-e1p1[j];
                    e2[j] = e2p2[j]-e2p1[j];
                }

                double denom = e1.dot(e1)*e2.dot(e2)-(e1.dot(e2))*(e1.dot(e2));
                if(denom < 0)
                    denom = 0;
                denom = sqrt(denom);
                Vector3d grade1 = (e2 - e1.dot(e2)/(e1.dot(e1))*e1)/denom;
                Vector3d grade2 = (e1 - e2.dot(e1)/(e2.dot(e2))*e2)/denom;
                double arg = -e1.dot(e2)/sqrt(e1.dot(e1)*e2.dot(e2));
                if(arg < -1.0) arg = -1.0;
                if(arg > 1.0) arg = 1.0;
                double theta = acos(arg);
                sumtheta += theta;
                for(int k=0; k<3; k++)
                {
                    addToStrippedMatrix(Pd, prhs, curtoidx, k, row, grade2[k], vidx2midx);
                    //Pd.coeffRef(3*curtoidx+k,row) += grade2[k];
                    addToStrippedMatrix(Pd, prhs, curfromidx, k, row, -grade2[k], vidx2midx);
                    //Pd.coeffRef(3*curfromidx+k, row) -= grade2[k];
                    addToStrippedMatrix(Pd, prhs, curfromidx, k, row, -grade1[k], vidx2midx);
                    //Pd.coeffRef(3*curfromidx+k, row) -= grade1[k];
                    addToStrippedMatrix(Pd, prhs, prevfromidx, k, row, grade1[k], vidx2midx);
                    //Pd.coeffRef(3*prevfromidx+k,row) += grade1[k];
                }

                prev = cur;
                cur = mesh_.next_halfedge_handle(cur);
            }
            sumtheta -= 2*3.1415926535898;
            prhs[row] -= sumtheta;
            row++;
        }
    }
    assert(row == numplanarity);

    prhs += Pd.transpose()*q0;


    rhs += CEd*cerhs;
    Md += CEd*CEd.transpose();

    if(planarity)
    {
        rhs += 0.1*Pd*prhs;
        Md += 0.1*Pd*Pd.transpose();
    }

    double smoothing = 0.0;
    Md += smoothing/beta * L.transpose()*L;
    rhs += smoothing/beta * L.transpose()*Lrhs;

    VectorXd result = q0;

    int oldid = getMeshID();
    ml.reset();
    SparseMatrix<double> M(Md);
    cont_.getSolvers().linearSolveCG(M, rhs, result);
    double residual = std::numeric_limits<double>::infinity();

    ml = acquireMesh();
    if(oldid == getMeshID())
    {
        for(int i=0; i<nummdofs; i++)
        {
            MyMesh::VertexHandle vh = mesh_.vertex_handle(midx2vidx[i]);
           // if(!mesh_.data(vh).pinned() && !mesh_.data(vh).anchored())
            {
                MyMesh::Point &pt = mesh_.point(vh);
                for(int j=0; j<3; j++)
                {
                    pt[j] = result[3*i+j];
                }
            }
        }

        //double planresidual = (Pd.transpose()*result+p0).norm();
        //cout << "Planarity residual " << planresidual << endl;


        residual = calculateEquilibriumViolation();
        invalidateMesh();
    }
    return residual;
}

void NetworkMesh::addToStrippedMatrix(DynamicSparseMatrix<double> &M, VectorXd &rhs, int v, int k, int j, double val, std::map<int, int> &vidx2midx)
{
    MyMesh::VertexHandle vh = mesh_.vertex_handle(v);
    if(!mesh_.data(vh).pinned() && !mesh_.data(vh).anchored())
    {
        M.coeffRef(3*vidx2midx[v] + k, j) += val;
    }
    else
    {
        MyMesh::Point pt = mesh_.point(vh);
        rhs[j] -= pt[k]*val;
    }
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

bool NetworkMesh::fixBadVerticesNew()
{
    bool done = true;
    auto_ptr<MeshLock> ml = acquireMesh();
    for(MyMesh::VertexIter vi = mesh_.vertices_begin(); vi != mesh_.vertices_end(); ++vi)
    {
        if(isBadVertex(vi.handle()))
        {
            //cout << "Bad vert!" << endl;
            int numadj = 0;
            for(MyMesh::VertexVertexIter vvi = mesh_.vv_iter(vi.handle()); vvi; ++vvi)
            {
                numadj++;
            }
            MatrixXd LS(numadj, 3);
            VectorXd rhs(numadj);
            int row=0;
            for(MyMesh::VertexVertexIter vvi = mesh_.vv_iter(vi.handle()); vvi; ++vvi)
            {
                MyMesh::Point pt = mesh_.point(vvi.handle());
                LS(row,0) = pt[0];
                LS(row,1) = pt[2];
                LS(row,2) = 1.0;
                rhs[row] = pt[1];
                row++;
            }
            assert(row == numadj);
            Vector3d result = (LS.transpose()*LS).fullPivLu().solve(LS.transpose()*rhs);
            MyMesh::Point &centpt = mesh_.point(vi.handle());
            double newz = result[0]*centpt[0] + result[1]*centpt[2] + result[2];
            if(centpt[1] > newz)
            {
                centpt[1] = newz;
                done = false;
            }
        }
    }
    return done;
}

double NetworkMesh::enforcePlanarity()
{
    auto_ptr<MeshLock> ml = acquireMesh();

    int n = mesh_.n_vertices();

    int numplanarity = 0;
    for(MyMesh::FaceIter fi = mesh_.faces_begin(); fi != mesh_.faces_end(); ++fi)
    {
        if(numFaceVerts(mesh_, fi.handle()) == 4)
        {
            numplanarity++;
        }
    }

    DynamicSparseMatrix<double> Pd(3*n, numplanarity);
    VectorXd p0(numplanarity);
    p0.setZero();

    int row=0;
    for(MyMesh::FaceIter fi = mesh_.faces_begin(); fi != mesh_.faces_end(); ++fi)
    {
        if(numFaceVerts(mesh_, fi.handle()) == 4)
        {
            MyMesh::FaceHalfedgeIter fhi = mesh_.fh_iter(fi.handle());
            MyMesh::HalfedgeHandle prev = fhi.handle();

            MyMesh::HalfedgeHandle cur = mesh_.next_halfedge_handle(prev);
            double sumtheta = 0;
            for(int i=0; i<4; i++)
            {
                MyMesh::Point e1p1, e1p2, e2p1, e2p2;
                int curtoidx = mesh_.to_vertex_handle(cur).idx();
                int curfromidx = mesh_.from_vertex_handle(cur).idx();
                assert(curfromidx == mesh_.to_vertex_handle(prev).idx());
                int prevfromidx = mesh_.from_vertex_handle(prev).idx();

                e2p2 = mesh_.point(mesh_.to_vertex_handle(cur));
                e2p1 = mesh_.point(mesh_.from_vertex_handle(cur));
                e1p1 = e2p1;
                e1p2 = mesh_.point(mesh_.from_vertex_handle(prev));

                Vector3d e1,e2;
                for(int j=0; j<3; j++)
                {
                    e1[j] = e1p2[j]-e1p1[j];
                    e2[j] = e2p2[j]-e2p1[j];
                }

                double denom = e1.dot(e1)*e2.dot(e2)-(e1.dot(e2))*(e1.dot(e2));
                assert(denom > 0);
                denom = sqrt(denom);
                Vector3d grade1 = (e2 - e1.dot(e2)/(e1.dot(e1))*e1)/denom;
                Vector3d grade2 = (e1 - e2.dot(e1)/(e2.dot(e2))*e2)/denom;
                double arg = -e1.dot(e2)/sqrt(e1.dot(e1)*e2.dot(e2));
                assert(arg >= -1.0 && arg <= 1.0);
                double theta = acos(arg);
                sumtheta += theta;
                for(int k=0; k<3; k++)
                {
                    Pd.coeffRef(3*curtoidx+k,row) += grade2[k];
                    Pd.coeffRef(3*curfromidx+k, row) -= grade2[k];
                    Pd.coeffRef(3*curfromidx+k, row) -= grade1[k];
                    Pd.coeffRef(3*prevfromidx+k,row) += grade1[k];
                }

                prev = cur;
                cur = mesh_.next_halfedge_handle(cur);
            }
            sumtheta -= 2*3.1415926535898;
            p0[row] = sumtheta;
            row++;
        }
    }
    assert(row == numplanarity);

    VectorXd q0(3*n);
    for(MyMesh::VertexIter vi = mesh_.vertices_begin(); vi != mesh_.vertices_end(); ++vi)
    {
        for(int j=0; j<3; j++)
            q0[3*vi.handle().idx()+j] = mesh_.point(vi.handle())[j];
    }

    p0 -= Pd.transpose()*q0;

    VectorXd M(3*n);
    for(int i=0; i<3*n; i++)
        M[i] = 1.0;

    VectorXd result = q0;
    SparseMatrix<double> P(Pd);

    cont_.getSolvers().solveWeightedLSE(M, q0, P, p0, result);
    double resbefore = (P.transpose()*q0+p0).norm();
    double residual = (P.transpose()*result+p0).norm();
    cout << "res " << resbefore << " -> " << residual << endl;
    for(int i=0; i<n; i++)
    {
        MyMesh::VertexHandle vh = mesh_.vertex_handle(i);
        for(int j=0; j<3; j++)
            mesh_.point(vh)[j] = result[3*i+j];
    }

    invalidateMesh();
    return residual;
}

Matrix2d NetworkMesh::approximateStressHessian(MyMesh::FaceHandle face)
{
    set<int> uniquepts;
    for(MyMesh::FaceVertexIter fvi = mesh_.fv_iter(face); fvi; ++fvi)
    {
        uniquepts.insert(fvi.handle().idx());
    }
    for(MyMesh::FaceFaceIter ffi = mesh_.ff_iter(face); ffi; ++ffi)
    {
        for(MyMesh::FaceVertexIter fvi = mesh_.fv_iter(ffi.handle()); fvi; ++fvi)
            uniquepts.insert(fvi.handle().idx());
    }

    int numpts = uniquepts.size();
    uniquepts.clear();

    MatrixXd LS(numpts, 6);
    VectorXd rhs(numpts);

    int curpt = 0;
    for(MyMesh::FaceVertexIter fvi = mesh_.fv_iter(face); fvi; ++fvi)
    {
        if(uniquepts.count(fvi.handle().idx()))
            continue;

        uniquepts.insert(fvi.handle().idx());
        MyMesh::Point pt = mesh_.point(fvi.handle());
        LS(curpt, 0) = pt[0]*pt[0];
        LS(curpt, 1) = pt[0]*pt[2];
        LS(curpt, 2) = pt[2]*pt[2];
        LS(curpt, 3) = pt[0];
        LS(curpt, 4) = pt[2];
        LS(curpt, 5) = 1;
        rhs[curpt] = 0;
        curpt++;
    }

    for(MyMesh::FaceHalfedgeIter fhi = mesh_.fh_iter(face); fhi; ++fhi)
    {
        MyMesh::FaceHandle oppface = mesh_.opposite_face_handle(fhi.handle());
        if(!oppface.is_valid())
            continue;
        Vector2d edgev;
        MyMesh::Point top = mesh_.point(mesh_.to_vertex_handle(fhi.handle()));
        MyMesh::Point fromp = mesh_.point(mesh_.from_vertex_handle(fhi.handle()));
        edgev[0] = -(top[2]-fromp[2]);
        edgev[1] = top[0]-fromp[0];
        edgev *= mesh_.data(mesh_.edge_handle(fhi.handle())).weight();
        // z = edgev[0] x  + edgev[1] y + c
        double c1 = -edgev[0] * top[0] - edgev[1] * top[2];
        double c2 = -edgev[0] * fromp[0] - edgev[1] * fromp[2];
        double c = 0.5*(c1+c2);

        for(MyMesh::FaceVertexIter fvi = mesh_.fv_iter(oppface); fvi; ++fvi)
        {
            if(uniquepts.count(fvi.handle().idx()))
                continue;

            uniquepts.insert(fvi.handle().idx());

            MyMesh::Point pt = mesh_.point(fvi.handle());
            LS(curpt, 0) = pt[0]*pt[0];
            LS(curpt, 1) = pt[0]*pt[2];
            LS(curpt, 2) = pt[2]*pt[2];
            LS(curpt, 3) = pt[0];
            LS(curpt, 4) = pt[2];
            LS(curpt, 5) = 1;
            rhs[curpt] = edgev[0] * pt[0] + edgev[1] * pt[2] + c;
            curpt++;
        }
    }
    assert(curpt == numpts);

    VectorXd sol = (LS.transpose()*LS).ldlt().solve(LS.transpose()*rhs);
    Matrix2d hess;
    hess << 2*sol[0], sol[1], sol[1], 2*sol[2];
    return hess;
}

void NetworkMesh::computeRelativePrincipalDirections()
{
    auto_ptr<MeshLock> ml = acquireMesh();

    for(MyMesh::FaceIter fi = mesh_.faces_begin(); fi != mesh_.faces_end(); ++fi)
    {
        Matrix2d Hf = approximateStressHessian(fi.handle());
        Matrix2d Hg = approximateHessian(fi.handle());
        Matrix2d Hfx;
        Hfx << Hf(1,1), -Hf(0,1), -Hf(0,1), Hf(0,0);
        Matrix2d W = Hfx*Hg;

        double lambda1, lambda2;

        Vector2d u, v;

        computeEigenstuff(W, lambda1, lambda2, u, v);

        double discr = sqrt((lambda1-lambda2)*(lambda1-lambda2));
        mesh_.data(fi.handle()).set_umbilic(discr);

        double x, y, C;
        computeFacePlane(mesh_, fi.handle(), x,y,C);
        Vector3d u3d, v3d;
        u3d[0] = u[0];
        u3d[2] = u[1];
        u3d[1] = x*u[0] + y*u[1];

        v3d[0] = v[0];
        v3d[2] = v[1];
        v3d[1] = x*v[0] + y*v[1];

        u3d.normalize();
        v3d.normalize();
        mesh_.data(fi.handle()).set_rel_principal_dirs(u3d,v3d);
    }
}

void NetworkMesh::setupVFProperties()
{
    auto_ptr<MeshLock> ml = acquireMesh();
    OpenMesh::FPropHandleT<OpenMesh::Vec3d> vf1;
    OpenMesh::FPropHandleT<OpenMesh::Vec3d> vf2;
    mesh_.get_property_handle(vf1, "Vector");
    mesh_.get_property_handle(vf2, "Vector_2");
    if(!vf1.is_valid())
    {
        mesh_.add_property(vf1,  "Vector");
        mesh_.property(vf1).set_persistent(true);
    }
    if(!vf2.is_valid())
    {
        mesh_.add_property(vf2,  "Vector_2");
        mesh_.property(vf2).set_persistent(true);
    }
    for(MyMesh::FaceIter fi = mesh_.faces_begin(); fi != mesh_.faces_end(); ++fi)
    {
        for(int i=0; i<3; i++)
        {
            mesh_.property(vf1, fi.handle())[i] = mesh_.data(fi.handle()).rel_principal_u()[i];
            mesh_.property(vf2, fi.handle())[i] = mesh_.data(fi.handle()).rel_principal_v()[i];
        }
    }
}

void NetworkMesh::solveLaplacian()
{
    auto_ptr<MeshLock> ml = acquireMesh();
/*
    assert(mesh_.n_faces() == 1);

    // set up weights

    int e = mesh_.n_edges();

    double oddsum = 0;
    double evensum = 0;
    for(int i=0; i<e; i++)
    {
        double len = mesh_.calc_edge_length(mesh_.edge_handle(i));
        ((i%2) == 0 ? evensum : oddsum) += len;
    }
    for(int i=0; i<e; i++)
    {
        double len = mesh_.calc_edge_length(mesh_.edge_handle(i));
        double weight = len / ((i%2)==0 ? evensum : -oddsum);
        mesh_.data(mesh_.edge_handle(i)).set_weight(weight);
    }

    MyMesh::Point facecent;
    mesh_.calc_face_centroid(mesh_.faces_begin().handle(), facecent);
    //mesh_.split(mesh_.faces_begin().handle(), facecent);
    triangulate();

    for(int i=0; i<4; i++)
        triangleSubdivide();

    int n = mesh_.n_vertices();

    DynamicSparseMatrix<double> L(n,n);
    VectorXd rhs(n);
    L.setZero();
    rhs.setZero();

    int row=0;
    for(int i=0; i<n; i++)
    {
        MyMesh::VertexHandle vh = mesh_.vertex_handle(i);
        if(!mesh_.is_boundary(vh))
        {
            for(MyMesh::VertexOHalfedgeIter ve = mesh_.voh_iter(vh); ve; ++ve)
            {
                assert(!mesh_.is_boundary(ve.handle()));

                double w = computeLaplacianWeight(mesh_.edge_handle(ve.handle()));
                MyMesh::VertexHandle tov = mesh_.to_vertex_handle(ve.handle());
                L.coeffRef(row, tov.idx()) -= w;
                L.coeffRef(row, i) += w;
            }
            row++;
        }

    }


    for(MyMesh::VertexIter vi = mesh_.vertices_begin(); vi != mesh_.vertices_end(); ++vi)
    {
        if(mesh_.is_boundary(vi))
        {
            int numf = mesh_.valence(vi.handle()) - 1;
            for(MyMesh::VertexOHalfedgeIter voh = mesh_.voh_iter(vi.handle()); voh; ++voh)
            {
                if(mesh_.is_boundary(voh.handle()))
                    continue;
                MyMesh::HalfedgeHandle e1 = voh.handle();
                MyMesh::HalfedgeHandle e2 = mesh_.prev_halfedge_handle(e1);
                double w1 = computeLaplacianAlpha(e1);
                double w2 = computeLaplacianAlpha(e2);
                L.coeffRef(row, mesh_.to_vertex_handle(e1).idx()) += w1/numf;
                L.coeffRef(row, mesh_.from_vertex_handle(e1).idx()) -= w1/numf;
                L.coeffRef(row, mesh_.from_vertex_handle(e2).idx()) += w2/numf;
                L.coeffRef(row, mesh_.to_vertex_handle(e2).idx()) -= w2/numf;
            }
            double weight = 0;
            for(MyMesh::VertexEdgeIter vei = mesh_.ve_iter(vi.handle()); vei; ++vei)
            {
                if(mesh_.is_boundary(vei.handle()))
                    weight += mesh_.data(vei.handle()).weight();
            }
            rhs[row] = 0.5*weight;
            row++;
        }
    }
    assert(row == n);

    VectorXd result(n);
    result.setZero();
    SparseMatrix<double> LTL(L.transpose()*L);

    cont_.getSolvers().linearSolveLU(LTL, L.transpose()*rhs, result);

    cout << "residual " << (L*result-rhs).norm() << endl;


    double avz = 0;
    for(int i=0; i<n; i++)
    {
        avz += result[i];
    }
    avz /= n;

    for(int i=0; i<n; i++)
    {
        MyMesh::Point &pt = mesh_.point(mesh_.vertex_handle(i));
        pt[1] = result[i]-avz;
    }
*/
    computeLaplacianCurvatures();
}

double NetworkMesh::computeLaplacianWeight(MyMesh::EdgeHandle edge)
{
    //return 1.0;
    return computeLaplacianAlpha(mesh_.halfedge_handle(edge,0)) + computeLaplacianAlpha(mesh_.halfedge_handle(edge,1));
}

double NetworkMesh::computeLaplacianAlpha(MyMesh::HalfedgeHandle heh)
{
    MyMesh::HalfedgeHandle e1 = mesh_.next_halfedge_handle(heh);
    MyMesh::HalfedgeHandle e2 = mesh_.prev_halfedge_handle(heh);
    assert(e1.is_valid() && e2.is_valid());
    assert(mesh_.to_vertex_handle(e1) == mesh_.from_vertex_handle(e2));

    MyMesh::Point opp = mesh_.point(mesh_.to_vertex_handle(e1));
    MyMesh::Point p1 = mesh_.point(mesh_.from_vertex_handle(e1));
    MyMesh::Point p2 = mesh_.point(mesh_.to_vertex_handle(e2));

    double num = (p1[0]-opp[0])*(p2[0]-opp[0]) + (p1[2]-opp[2])*(p2[2]-opp[2]);
    double denom = fabs( (p1[0]-opp[0])*(p2[2]-opp[2]) - (p1[2]-opp[2])*(p2[0]-opp[0]));
    return num/denom;
}

void NetworkMesh::computeLaplacianCurvatures()
{
    for(MyMesh::VertexIter vi = mesh_.vertices_begin(); vi != mesh_.vertices_end(); ++vi)
    {
        Matrix2d H = approximateHessianVertex(vi.handle());
        Vector2d u, v;            
        u.setZero();
        v.setZero();
  //      if(!mesh_.is_boundary(vi.handle()))
        {
            double l1, l2;
            computeEigenstuff(H, l1, l2, u, v);
            u.normalize();
            v.normalize();
        }
        mesh_.data(vi.handle()).set_rel_principal_dirs(u,v);
    }
    for(MyMesh::VertexIter vi = mesh_.vertices_begin(); vi != mesh_.vertices_end(); ++vi)
    {
      mesh_.point(vi.handle())[1] = 0.0;
      mesh_.data(vi.handle()).set_violation(100);
    }
}

void NetworkMesh::triangleSubdivide()
{
    auto_ptr<MeshLock> ml = acquireMesh();

    MyMesh newmesh;

    int n = mesh_.n_vertices();
    int e = mesh_.n_edges();

    for(int i=0; i<n; i++)
    {
        MyMesh::VertexHandle oldv = mesh_.vertex_handle(i);
        MyMesh::Point newpt;
        newpt[0] = newpt[1] = newpt[2] = 0;
        int numadj=0;
        if(!mesh_.is_boundary(oldv))
        {
            for(MyMesh::VertexVertexIter vvi = mesh_.vv_iter(oldv); vvi; ++vvi)
            {
                newpt += mesh_.point(vvi.handle())/16.0;
                numadj++;
            }
        }
        newpt += mesh_.point(oldv) * (16-numadj)/16.0;
        newmesh.add_vertex(newpt);
    }
    for(int i=0; i<e; i++)
    {
        MyMesh::EdgeHandle edge = mesh_.edge_handle(i);
        if(mesh_.is_boundary(edge))
            newmesh.add_vertex(computeEdgeMidpoint(edge));
        else
        {
            MyMesh::HalfedgeHandle heh1 = mesh_.halfedge_handle(edge,0);
            MyMesh::HalfedgeHandle heh2 = mesh_.halfedge_handle(edge,1);
            MyMesh::Point newpt = mesh_.point(mesh_.to_vertex_handle(heh1))*3.0/8.0;
            newpt += mesh_.point(mesh_.from_vertex_handle(heh1))*3.0/8.0;
            newpt += mesh_.point(mesh_.to_vertex_handle(mesh_.next_halfedge_handle(heh1)))*1.0/8.0;
            newpt += mesh_.point(mesh_.to_vertex_handle(mesh_.next_halfedge_handle(heh2)))*1.0/8.0;
            newmesh.add_vertex(newpt);
        }
    }
    for(MyMesh::FaceIter fi = mesh_.faces_begin(); fi != mesh_.faces_end(); ++fi)
    {
        vector<MyMesh::VertexHandle> toaddcenter;
        for(MyMesh::FaceHalfedgeIter fhi = mesh_.fh_iter(fi.handle()); fhi; ++fhi)
        {
            int vidx = mesh_.from_vertex_handle(fhi.handle()).idx();
            MyMesh::HalfedgeHandle prev = mesh_.prev_halfedge_handle(fhi.handle());
            vector<MyMesh::VertexHandle> toadd;
            toadd.push_back(newmesh.vertex_handle(vidx));
            int eidx1 = mesh_.edge_handle(prev).idx();
            int eidx2 = mesh_.edge_handle(fhi.handle()).idx();
            toadd.push_back(newmesh.vertex_handle(n+eidx2));
            toadd.push_back(newmesh.vertex_handle(n+eidx1));
            MyMesh::FaceHandle newface = newmesh.add_face(toadd);
            toaddcenter.push_back(newmesh.vertex_handle(n+eidx2));

            //fix weights; don't bother with interior triangles
            for(MyMesh::FaceHalfedgeIter newfh = newmesh.fh_iter(newface); newfh; ++newfh)
            {
                if(newmesh.from_vertex_handle(newfh.handle()).idx() == vidx)
                {
                    newmesh.data(newmesh.edge_handle(newfh.handle())).set_weight(0.5*mesh_.data(mesh_.edge_handle(fhi.handle())).weight());
                }
                if(newmesh.to_vertex_handle(newfh.handle()).idx() == vidx)
                {
                    newmesh.data(newmesh.edge_handle(newfh.handle())).set_weight(0.5*mesh_.data(mesh_.edge_handle(prev)).weight());
                }
            }
        }
        newmesh.add_face(toaddcenter);
    }
    mesh_ = newmesh;
}

void NetworkMesh::exportVectorFields(const char *name)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    ofstream ofs(name);
    if(!ofs)
        return;

    for(MyMesh::FaceIter fi = mesh_.faces_begin(); fi != mesh_.faces_end(); ++fi)
    {
        Vector3d u = mesh_.data(fi.handle()).rel_principal_u();
        Vector3d v = mesh_.data(fi.handle()).rel_principal_v();
        ofs << u.transpose() << " " << v.transpose() << endl;
    }
}

bool NetworkMesh::exportWeights(const char *name)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    ofstream ofs(name);
    if(!ofs)
        return false;

    for(MyMesh::EdgeIter ei = mesh_.edges_begin(); ei != mesh_.edges_end(); ++ei)
    {
        MyMesh::HalfedgeHandle heh = mesh_.halfedge_handle(ei.handle(), 0);
        MyMesh::VertexHandle tov = mesh_.to_vertex_handle(heh);
        MyMesh::VertexHandle frv = mesh_.from_vertex_handle(heh);
        ofs << frv.idx() + 1 << "\t" << tov.idx()+1 << "\t" << mesh_.data(ei).weight() << endl;
    }
    return ofs;
}

bool NetworkMesh::exportReciprocalMesh(const char *name)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    MyMesh rmesh;

    int f = mesh_.n_faces();
    int n = mesh_.n_vertices();

    for(int i=0; i<f; i++)
    {
        MyMesh::Point pt(0,0,0);
        rmesh.add_vertex(pt);
    }
    for(int i=0; i<n; i++)
    {
        MyMesh::VertexHandle vh(i);
        if(!mesh_.is_boundary(vh))
        {
            vector<MyMesh::VertexHandle> toadd;
            for(MyMesh::VertexFaceIter vfi = mesh_.vf_iter(vh); vfi; ++vfi)
            {
                toadd.push_back(rmesh.vertex_handle(vfi.handle().idx()));
            }
            rmesh.add_face(toadd);
        }
    }
    for(int i=0; i<f; i++)
    {
        MyMesh::FaceHandle fh(i);
        mesh_.data(fh).set_integrated(i==0);
    }

    // TODO something less simplistic
    bool done = false;
    while(!done)
    {
        done = true;
        for(MyMesh::FaceIter fi = mesh_.faces_begin(); fi != mesh_.faces_end(); ++fi)
        {
            if(mesh_.data(fi).integrated())
            {
                for(MyMesh::FaceHalfedgeIter fhi = mesh_.fh_iter(fi.handle()); fhi; ++fhi)
                {
                    MyMesh::HalfedgeHandle opp = mesh_.opposite_halfedge_handle(fhi.handle());
                    if(mesh_.is_boundary(opp))
                        continue;
                    MyMesh::FaceHandle adj = mesh_.face_handle(opp);
                    if(!mesh_.data(adj).integrated())
                    {
                        done = false;
                        MyMesh::EdgeHandle eh = mesh_.edge_handle(opp);
                        double weight = mesh_.data(eh).weight();
                        MyMesh::VertexHandle v1 = mesh_.to_vertex_handle(opp);
                        MyMesh::VertexHandle v2 = mesh_.from_vertex_handle(opp);
                        MyMesh::Point recip = (mesh_.point(v1)-mesh_.point(v2))*weight;
                        recip[1] = 0;
                        double tmp = recip[0];
                        recip[0] = -recip[2];
                        recip[2] = tmp;
                        rmesh.point(rmesh.vertex_handle(adj.idx())) = rmesh.point(rmesh.vertex_handle(fi.handle().idx())) + recip;
                        mesh_.data(adj).set_integrated(true);
                    }
                }
            }
        }
    }
    OpenMesh::IO::Options opts;
    return OpenMesh::IO::write_mesh(rmesh, name, opts);
}
