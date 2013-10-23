#include "referencemesh.h"
#include "referencemeshrenderer.h"
#include "controller.h"
#include "solvers.h"
#include "networkmesh.h"
#include <fstream>
#include "camera.h"
#include <vector>

#include <Eigen/Sparse>

typedef Eigen::Triplet<double> T;

using namespace Eigen;
using namespace std;

ReferenceMesh::ReferenceMesh(Controller &cont) : Mesh(cont)
{
    renderer_ = new ReferenceMeshRenderer(*this);
}

ReferenceMesh::~ReferenceMesh()
{
    delete renderer_;
}

MeshRenderer &ReferenceMesh::getRenderer()
{
    return *renderer_;
}

void ReferenceMesh::copyFromNetworkMesh(NetworkMesh &nm)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    auto_ptr<MeshLock> nml = nm.acquireMesh();
    copyMesh(nm.getMesh());
}

void ReferenceMesh::buildQuadMesh(int w, int h)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    if(w<2)
        w=2;
    if(h<2)
        h=2;

    mesh_.clear();

    for(int i=0; i<h; i++)
        for(int j=0; j<w; j++)
        {
            double x = -1.0 + 0.2*j;
            double y = -1.0 + 0.2*i;
            mesh_.add_vertex(MyMesh::Point(x,0,y));
        }
    for(int i=0; i<h-1; i++)
        for(int j=0; j<w-1; j++)
        {
            std::vector<MyMesh::VertexHandle>  face_vhandles;
            face_vhandles.push_back(MyMesh::VertexHandle(i*w+j));
            face_vhandles.push_back(MyMesh::VertexHandle(i*w+j+1));
            face_vhandles.push_back(MyMesh::VertexHandle(i*w+j+w+1));
            face_vhandles.push_back(MyMesh::VertexHandle(i*w+j+w));
            mesh_.add_face(face_vhandles);
        }
    pinBoundary();
}

void ReferenceMesh::buildHexMesh(int w, int h)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    if(w<2) w=2;
    if(h<3) h=3;

    mesh_.clear();
    for(int i=0; i<h; i++)
    {
        double width = (3*w-2)/2.0;
        if( (i%2) == 0)
        {
            double x = 0.5;
            for(int j=0; j<w; j++)
            {
                double sx = -1.0 + 2.0*x/width;
                double y = -1.0 + 2.0*i/(h-1);
                mesh_.add_vertex(MyMesh::Point(sx,0,y));
                if( (j%2) == 0)
                    x += 1.0;
                else
                    x += 2.0;
            }
        }
        else
        {
            double x = 0;
            for(int j=0; j<w; j++)
            {
                double sx = -1.0 + 2.0*x/width;
                double y = -1.0 + 2.0*i/(h-1);
                mesh_.add_vertex(MyMesh::Point(sx,0,y));
                if( (j%2) == 0)
                    x += 2.0;
                else
                    x += 1.0;
            }
        }
    }
    for(int i=0; i<h-2; i++)
    {
        if( (i%2) == 0)
        {
            for(int j=0; j<w/2; j++)
            {
                std::vector<MyMesh::VertexHandle>  face_vhandles;
                face_vhandles.push_back(MyMesh::VertexHandle(i*w+2*j));
                face_vhandles.push_back(MyMesh::VertexHandle(i*w+2*j+1));
                face_vhandles.push_back(MyMesh::VertexHandle((i+1)*w + 2*j + 1));
                face_vhandles.push_back(MyMesh::VertexHandle((i+2)*w + 2*j + 1));
                face_vhandles.push_back(MyMesh::VertexHandle((i+2)*w + 2*j));
                face_vhandles.push_back(MyMesh::VertexHandle((i+1)*w + 2*j));
                mesh_.add_face(face_vhandles);
            }
        }
        else
        {
            for(int j=0; j<(w-1)/2; j++)
            {
                std::vector<MyMesh::VertexHandle>  face_vhandles;
                face_vhandles.push_back(MyMesh::VertexHandle(i*w+2*j+1));
                face_vhandles.push_back(MyMesh::VertexHandle(i*w+2*j+2));
                face_vhandles.push_back(MyMesh::VertexHandle((i+1)*w + 2*j + 2));
                face_vhandles.push_back(MyMesh::VertexHandle((i+2)*w + 2*j + 2));
                face_vhandles.push_back(MyMesh::VertexHandle((i+2)*w + 2*j+1));
                face_vhandles.push_back(MyMesh::VertexHandle((i+1)*w + 2*j+1));
                mesh_.add_face(face_vhandles);
            }
        }
    }
    pinBoundary();
}

void ReferenceMesh::buildTriMesh(int w, int h)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    if(w<2)
        w=2;
    if(h<2)
        h=2;

    mesh_.clear();

    for(int i=0; i<h; i++)
        for(int j=0; j<w; j++)
        {
            double x = -1.0 + 2.0*j/(w-1);
            double y = -1.0 + 2.0*i/(h-1);
            mesh_.add_vertex(MyMesh::Point(x,0,y));
        }
    for(int i=0; i<h-1; i++)
        for(int j=0; j<w-1; j++)
        {
            std::vector<MyMesh::VertexHandle>  face_vhandles;
            face_vhandles.push_back(MyMesh::VertexHandle(i*w+j));
            face_vhandles.push_back(MyMesh::VertexHandle(i*w+j+1));
            face_vhandles.push_back(MyMesh::VertexHandle(i*w+j+w+1));
            mesh_.add_face(face_vhandles);

            face_vhandles[1] = face_vhandles[2];
            face_vhandles[2] = MyMesh::VertexHandle(i*w+j+w);
            mesh_.add_face(face_vhandles);
        }
    pinBoundary();
}

void ReferenceMesh::computeClosestPointOnPlane(const Vector2d &pos, int &closestidx, double &closestdist)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    closestdist = std::numeric_limits<double>::infinity();
    closestidx = -1;

    int n = mesh_.n_vertices();
    for(int i=0; i<n; i++)
    {
        MyMesh::VertexHandle vh = mesh_.vertex_handle(i);
        MyMesh::Point pt = mesh_.point(vh);
        double dist = (pt[0]-pos[0])*(pt[0]-pos[0]) + (pt[2]-pos[1])*(pt[2]-pos[1]);
        if(dist < closestdist)
        {
            closestdist = dist;
            closestidx = i;
        }
    }

    closestdist = sqrt(closestdist);
}

void ReferenceMesh::jitterOnPlane()
{
    auto_ptr<MeshLock> ml = acquireMesh();
    int n = mesh_.n_vertices();
    if(n == 0)
    {
        return;
    }
    VectorXd lens(n);
    for(int i=0; i<n; i++)
    {
        MyMesh::VertexHandle vh = mesh_.vertex_handle(i);
        double minlen = std::numeric_limits<double>::infinity();
        MyMesh::Point center = mesh_.point(vh);
        for(MyMesh::VertexOHalfedgeIter voh = mesh_.voh_iter(vh); voh; ++voh)
        {
            MyMesh::Point adj = mesh_.point(mesh_.to_vertex_handle(voh));
            double len = sqrt( (center[0]-adj[0])*(center[0]-adj[0]) + (center[2]-adj[2])*(center[2]-adj[2]));
            if(len < minlen)
                minlen = len;
        }
        lens[i] = minlen;
    }

    for(int i=0; i<n; i++)
    {
        MyMesh::VertexHandle vh = mesh_.vertex_handle(i);
        double scale = lens[i] * 0.2;
        MyMesh::Point &pt = mesh_.point(vh);
        pt[0] += scale*(2.0*randomDouble()-1.0);
        pt[2] += scale*(2.0*randomDouble()-1.0);
    }
}

void ReferenceMesh::applyLaplacianDeformationHeight(int, const Vector3d &delta, int radius)
{
    auto_ptr<MeshLock> ml = acquireMesh();

    int n = mesh_.n_vertices();

    set<int> toAdd;

    for(int i=0; i<n; i++)
    {
        MyMesh::VertexHandle vh = mesh_.vertex_handle(i);
        if(mesh_.data(vh).handled())
        {
            set<int> nring;
            getNRing(i, 3, nring);
            for(set<int>::iterator it = nring.begin(); it != nring.end(); ++it)
                toAdd.insert(*it);
        }
    }

    set<int> toAlter;

    for(int i=0; i<n; i++)
    {
        MyMesh::VertexHandle vh = mesh_.vertex_handle(i);
        if(mesh_.data(vh).handled())
        {
            set<int> nring;
            this->getNRing(i, radius, nring);
            for(set<int>::iterator it = nring.begin(); it != nring.end(); ++it)
                toAlter.insert(*it);
        }
    }

    SparseMatrix<double> L(n,n);
    vector<T> Lcoeffs;
    for(set<int>::iterator it = toAlter.begin(); it != toAlter.end(); ++it)
    {
        int i = *it;
        MyMesh::VertexHandle vh = mesh_.vertex_handle(i);
        int valence = mesh_.valence(vh);
        Lcoeffs.push_back(T(i,i, 1.0));
        for(MyMesh::VertexVertexIter vv = mesh_.vv_iter(vh); vv; ++vv)
        {
            MyMesh::VertexHandle adj = vv;
            int adjidx = adj.idx();
            if(toAlter.count(adjidx) > 0)
                Lcoeffs.push_back(T(i, adjidx, -1.0/valence));
            else
            {

            }
        }
    }

    L.setFromTriplets(Lcoeffs.begin(), Lcoeffs.end());

    SparseMatrix<double> LTL(L.transpose()*L);
    VectorXd v0(n);
    for(int i=0; i<n; i++)
    {
        MyMesh::VertexHandle vh = mesh_.vertex_handle(i);
        MyMesh::Point pt = mesh_.point(vh);
        v0[i] = pt[1];
    }


    VectorXd rhs = LTL*v0;

    for(int i=0; i<n; i++)
    {
        MyMesh::VertexHandle vh = mesh_.vertex_handle(i);
        //if(mesh_.data(vh).handled())
        if(mesh_.data(vh).handled() || (radius == 6 && toAdd.count(i) > 0))
        {
            MyMesh::Point movedpt = mesh_.point(vh);
            rhs[i] += (movedpt[1]+delta[1]);
            LTL.coeffRef(i, i) += 1.0;
        }
//        else if(toAlter.count(i) > 0)
//        {
//            MyMesh::Point movedpt = mesh_.point(vh);
//            rhs[i] += (movedpt[1] + 0.5*delta[1]);
//            LTL.coeffRef(i, i) += 1.0;
//        }
        else if(mesh_.data(vh).pinned() || mesh_.data(vh).anchored())
        {
            LTL.coeffRef(i, i) += 1.0;
            rhs[i] += mesh_.point(vh)[1];
        }
    }

    SparseMatrix<double> M(LTL);

    cont_.getSolvers().linearSolveCG(M, rhs, v0);

    for(int i=0; i<n; i++)
    {
        MyMesh::VertexHandle vh = mesh_.vertex_handle(i);
        if(mesh_.data(vh).handled() || !mesh_.data(vh).pinned())
        {
            MyMesh::Point &pt = mesh_.point(vh);
            pt[1] = v0[i];
        }
    }

//    VectorXd dQ(3*n);
//    for(int i=0; i<n; i++)
//    {
//        dQ[3*i] = dQ[3*i+2] = 0;
//        MyMesh::VertexHandle vh = mesh_.vertex_handle(i);
//        MyMesh::Point pt = mesh_.point(vh);
//        dQ[3*i+1] = v0[i]-pt[1];
//    }
//    int e = mesh_.n_edges();
//    VectorXd dW(e);
//    computeBestDWeights(dQ, dW);

//    for(int i=0; i<n; i++)
//    {
//        MyMesh::VertexHandle vh = mesh_.vertex_handle(i);
//        MyMesh::Point &pt = mesh_.point(vh);
//        if(mesh_.data(vh).handled() || !mesh_.data(vh).pinned())
//            pt[1] += dQ[3*i+1];
//    }
//    for(int i=0; i<e; i++)
//    {
//        MyMesh::EdgeHandle eh = mesh_.edge_handle(i);
//        if(!edgePinned(eh))
//            mesh_.data(eh).set_weight(mesh_.data(eh).weight() + dW[i]);
//    }
}

void ReferenceMesh::applyLaplacianDeformation(int, const Vector3d &delta)
{
    auto_ptr<MeshLock> ml = acquireMesh();

    int n = mesh_.n_vertices();

    SparseMatrix<double> L(3*n,3*n);
    vector<T> Lcoeffs;
    for(int i=0; i<n; i++)
    {
        MyMesh::VertexHandle vh = mesh_.vertex_handle(i);
        int valence = mesh_.valence(vh);
        for(int j=0; j<3; j++)
            Lcoeffs.push_back(T(3*i+j, 3*i+j, 1.0));
        for(MyMesh::VertexVertexIter vv = mesh_.vv_iter(vh); vv; ++vv)
        {
            MyMesh::VertexHandle adj = vv;
            int adjidx = adj.idx();
            for(int j=0; j<3; j++)
                Lcoeffs.push_back(T(3*i+j, 3*adjidx+j, -1.0/valence));
        }
    }

    L.setFromTriplets(Lcoeffs.begin(), Lcoeffs.end());

    SparseMatrix<double> LTL(L.transpose()*L);
    VectorXd v0(3*n);
    for(int i=0; i<n; i++)
    {
        MyMesh::VertexHandle vh = mesh_.vertex_handle(i);
        MyMesh::Point pt = mesh_.point(vh);
        for(int j=0; j<3; j++)
            v0[3*i+j] = pt[j];
    }


    VectorXd rhs = LTL*v0;

    for(int i=0; i<n; i++)
    {
        MyMesh::VertexHandle vh = mesh_.vertex_handle(i);
        if(mesh_.data(vh).handled())
        {
            MyMesh::Point movedpt = mesh_.point(vh);
            for(int j=0; j<3; j++)
            {
                rhs[3*i+j] += (movedpt[j] + delta[j]);
                LTL.coeffRef(3*i+j,3*i+j) += 1.0;
            }
        }
        else if(mesh_.data(vh).pinned() || mesh_.data(vh).anchored())
        {
            for(int j=0; j<3; j++)
            {
                LTL.coeffRef(3*i + j, 3*i + j) += 1.0;
                rhs[3*i+j] += mesh_.point(vh)[j];
            }
        }
    }

    SparseMatrix<double> M(LTL);

    cont_.getSolvers().linearSolveCG(M, rhs, v0);

    for(int i=0; i<n; i++)
    {
        MyMesh::VertexHandle vh = mesh_.vertex_handle(i);
        MyMesh::Point &pt = mesh_.point(vh);
        if(mesh_.data(vh).handled() || !mesh_.data(vh).pinned())
        {
            pt[0] = v0[3*i];
            pt[1] = v0[3*i+1];
            pt[2] = v0[3*i+2];
        }
    }
}

void ReferenceMesh::applyLaplacianDeformationTop(int, const Vector3d &delta, int radius, bool excludePinned)
{
    auto_ptr<MeshLock> ml = acquireMesh();

    int n = mesh_.n_vertices();

    set<int> toApply;

    for(int i=0; i<n; i++)
    {
        MyMesh::VertexHandle vh = mesh_.vertex_handle(i);
        if(mesh_.data(vh).handled())
        {
            set<int> nring;
            getNRing(i, radius, nring);
            for(set<int>::iterator it = nring.begin(); it != nring.end(); ++it)
            {
                toApply.insert(*it);
            }
        }
    }

    SparseMatrix<double> L(2*n,2*n);
    vector<T> Lcoeffs;
    for(set<int>::iterator it = toApply.begin(); it != toApply.end(); ++it)
    {
        int i = *it;
        MyMesh::VertexHandle vh = mesh_.vertex_handle(i);
        int valence = mesh_.valence(vh);
        for(int j=0; j<2; j++)
            Lcoeffs.push_back(T(2*i+j, 2*i+j, 1.0));
        for(MyMesh::VertexVertexIter vv = mesh_.vv_iter(vh); vv; ++vv)
        {
            MyMesh::VertexHandle adj = vv;
            int adjidx = adj.idx();
            if(toApply.count(adjidx) > 0)
            {
                for(int j=0; j<2; j++)
                    Lcoeffs.push_back(T(2*i+j, 2*adjidx+j, -1.0/valence));
            }
        }
    }

    L.setFromTriplets(Lcoeffs.begin(), Lcoeffs.end());

    SparseMatrix<double> LTL(L.transpose()*L);
    VectorXd v0(2*n);
    for(int i=0; i<n; i++)
    {
        MyMesh::VertexHandle vh = mesh_.vertex_handle(i);
        MyMesh::Point pt = mesh_.point(vh);
        v0[2*i+0] = pt[0];
        v0[2*i+1] = pt[2];
    }


    VectorXd rhs = LTL*v0;

    for(int i=0; i<n; i++)
    {
        MyMesh::VertexHandle vh = mesh_.vertex_handle(i);
        if(mesh_.data(vh).handled())
        {
            MyMesh::Point movedpt = mesh_.point(vh);
            rhs[2*i+0] += (movedpt[0] + delta[0]);
            rhs[2*i+1] += (movedpt[2] + delta[2]);

            LTL.coeffRef(2*i+0,2*i+0) += 1.0;
            LTL.coeffRef(2*i+1,2*i+1) += 1.0;

        }
        else if(mesh_.data(vh).pinned() && excludePinned)
        {
            LTL.coeffRef(2*i + 0, 2*i + 0) += 1.0;
            LTL.coeffRef(2*i + 1, 2*i + 1) += 1.0;

            rhs[2*i+0] += mesh_.point(vh)[0];
            rhs[2*i+1] += mesh_.point(vh)[2];

        }
    }

    SparseMatrix<double> M(LTL);

    cont_.getSolvers().linearSolveCG(M, rhs, v0);

    for(int i=0; i<n; i++)
    {
        MyMesh::VertexHandle vh = mesh_.vertex_handle(i);
        MyMesh::Point &pt = mesh_.point(vh);
        pt[0] = v0[2*i];
        pt[2] = v0[2*i+1];
    }
}

void ReferenceMesh::setHandle(int vidx, bool state)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    int n = mesh_.n_vertices();
    assert(vidx >= 0 && vidx < n);

    mesh_.data(mesh_.vertex_handle(vidx)).set_handled(state);
}

void ReferenceMesh::setPin(int vidx, bool state)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    int n = mesh_.n_vertices();
    assert(vidx >= 0 && vidx < n);
    mesh_.data(mesh_.vertex_handle(vidx)).set_pinned(state);
}

void ReferenceMesh::deleteFace(int fidx)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    int f = mesh_.n_faces();
    assert(fidx >= 0 && fidx < f);
    MyMesh::FaceHandle fh = mesh_.face_handle(fidx);
    mesh_.delete_face(fh, true);
    mesh_.garbage_collection();
}

void ReferenceMesh::setAnchor(int vidx, bool state)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    int n = mesh_.n_vertices();
    assert(vidx >= 0 && vidx < n);
    mesh_.data(mesh_.vertex_handle(vidx)).set_anchored(state);
}

void ReferenceMesh::pinBoundary()
{
    auto_ptr<MeshLock> ml = acquireMesh();
    for(MyMesh::VertexIter vi = mesh_.vertices_begin(); vi != mesh_.vertices_end(); ++vi)
    {
        if(mesh_.is_boundary(vi.handle()))
            mesh_.data(vi.handle()).set_pinned(true);
    }
}

void ReferenceMesh::unpinBoundary()
{
    auto_ptr<MeshLock> ml = acquireMesh();
    for(MyMesh::VertexIter vi = mesh_.vertices_begin(); vi != mesh_.vertices_end(); ++vi)
    {
        if(mesh_.is_boundary(vi.handle()))
            mesh_.data(vi.handle()).set_pinned(false);
    }
}

void ReferenceMesh::swapYandZ()
{
    auto_ptr<MeshLock> ml = acquireMesh();
    for(MyMesh::VertexIter vi = mesh_.vertices_begin(); vi != mesh_.vertices_end(); ++vi)
    {
        MyMesh::Point &pt = mesh_.point(vi.handle());
        std::swap(pt[1], pt[2]);
    }
}

void ReferenceMesh::invertY()
{
    auto_ptr<MeshLock> ml = acquireMesh();
    for(MyMesh::VertexIter vi = mesh_.vertices_begin(); vi != mesh_.vertices_end(); ++vi)
    {
        MyMesh::Point &pt = mesh_.point(vi.handle());
        pt[1] = -pt[1];
    }
}

bool ReferenceMesh::addOBJ(const char *filename)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    OpenMesh::IO::Options opt;
    MyMesh addmesh;
    if(!OpenMesh::IO::read_mesh(addmesh, filename, opt))
    {
        cout << "Couldn't open file" << endl;
        return false;
    }

    if(addmesh.n_vertices() != mesh_.n_vertices())
    {
        cout << addmesh.n_vertices() << " != " << mesh_.n_vertices() << endl;
        return false;
    }

    int n = mesh_.n_vertices();
    for(int i=0; i<n; i++)
    {
        MyMesh::Point addpt = addmesh.point(addmesh.vertex_handle(i));
        int closestpt = -1;
        double closestdist = std::numeric_limits<double>::infinity();
        for(int j=0; j<n; j++)
        {
            MyMesh::Point pt = mesh_.point(mesh_.vertex_handle(j));
            double dist = (addpt[0]-pt[0])*(addpt[0]-pt[0]) + (addpt[2]-pt[2])*(addpt[2]-pt[2]);
            if(dist < closestdist)
            {
                closestdist = dist;
                closestpt = j;
            }
        }
        assert(closestpt != -1);
        assert(closestdist < 1e-8);
        MyMesh::Point &pt = mesh_.point(mesh_.vertex_handle(closestpt));
        pt[1] -= addpt[1];
    }
    return true;
}

vector<int> ReferenceMesh::selectRectangle(const Vector2d &c1, const Vector2d &c2, Camera &c)
{
    vector<int> result;
    int n = mesh_.n_vertices();
    for(int i=0; i<n; i++)
    {
        MyMesh::Point pt = mesh_.point(mesh_.vertex_handle(i));
        Eigen::Vector3d pos(pt[0], pt[1], pt[2]);
        double x,y;
        c.project(pos, x, y);
        if(x >= std::min(c1[0],c2[0]) && x <= std::max(c1[0],c2[0]) && y >= std::min(c1[1],c2[1]) && y <= std::max(c1[1],c2[1]))
        {
            result.push_back(i);
        }
    }
    return result;
}

void ReferenceMesh::averageHandledHeights()
{
    auto_ptr<MeshLock> ml = acquireMesh();
    double avz = 0.0;
    int numpts = 0;
    for(MyMesh::VertexIter vi = mesh_.vertices_begin(); vi != mesh_.vertices_end(); ++vi)
    {
        if(mesh_.data(vi).handled())
        {
            avz += mesh_.point(vi)[1];
            numpts++;
        }
    }
    avz /= numpts;
    for(MyMesh::VertexIter vi = mesh_.vertices_begin(); vi != mesh_.vertices_end(); ++vi)
    {
        if(mesh_.data(vi).handled())
        {
            mesh_.point(vi)[1] = avz;
        }
    }
}

void ReferenceMesh::trimBoundary()
{
    auto_ptr<MeshLock> ml = acquireMesh();
    vector<int> todelete;
    for(MyMesh::VertexIter vi = mesh_.vertices_begin(); vi != mesh_.vertices_end(); ++vi)
        if(mesh_.is_boundary(vi))
            todelete.push_back(vi.handle().idx());
    for(vector<int>::iterator it = todelete.begin(); it != todelete.end(); ++it)
        mesh_.delete_vertex(mesh_.vertex_handle(*it));
    mesh_.garbage_collection();
}


double ReferenceMesh::computeBestDWeights(const VectorXd &dQ, VectorXd &dW)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    int n = mesh_.n_vertices();
    int e = mesh_.n_edges();
    assert(dQ.size() == 3*n);
    dW.resize(e);
    dW.setZero();
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

    SparseMatrix<double> Md(3*interiorn+boundarye, e);
    vector<T> Mdcoeffs;
    VectorXd rhs(3*interiorn+boundarye);
    rhs.setZero();

    int row=0;
    for(int i=0; i<n; i++)
    {
        MyMesh::VertexHandle vh = mesh_.vertex_handle(i);
        if(mesh_.data(vh).pinned())
            continue;

        MyMesh::Point center = mesh_.point(vh);
        MyMesh::Point dcenter(dQ[3*vh.idx()], dQ[3*vh.idx()+1], dQ[3*vh.idx()+2]);

        for(MyMesh::VertexOHalfedgeIter voh = mesh_.voh_iter(vh); voh; ++voh)
        {

            MyMesh::HalfedgeHandle heh = voh;
            MyMesh::VertexHandle tov = mesh_.to_vertex_handle(heh);
            MyMesh::EdgeHandle eh = mesh_.edge_handle(heh);
            int eidx = eh.idx();
            MyMesh::Point adj = mesh_.point(tov);
            MyMesh::Point dadj(dQ[3*tov.idx()], dQ[3*tov.idx()+1],dQ[3*tov.idx()+2]);
            Mdcoeffs.push_back(T(row, eidx, center[0]-adj[0]));
            Mdcoeffs.push_back(T(row+1, eidx, (center[1]-adj[1])));
            Mdcoeffs.push_back(T(row+2, eidx, center[2]-adj[2]));
            double weight = mesh_.data(eh).weight();
            rhs[row] -= weight*(dcenter[0]-dadj[0]);
            rhs[row+1] -= weight*(dcenter[1]-dadj[1]);
            rhs[row+2] -= weight*(dcenter[2]-dadj[2]);
        }
        row += 3;
    }
    for(int i=0; i<e; i++)
    {
        MyMesh::EdgeHandle eh = mesh_.edge_handle(i);
        if(edgePinned(eh))
        {
            Mdcoeffs.push_back(T(row, i, 1.0));
            row++;
        }
    }
    Md.setFromTriplets(Mdcoeffs.begin(), Mdcoeffs.end());
    assert(row == 3*interiorn + boundarye);

    SparseMatrix<double> M(Md.transpose()*Md);
    cont_.getSolvers().linearSolveCG(M, Md.transpose()*rhs, dW);
    return (Md*dW-rhs).norm();
}

void ReferenceMesh::selectPinned()
{
    auto_ptr<MeshLock> ml = acquireMesh();

    for(MyMesh::VertexIter vi = mesh_.vertices_begin(); vi != mesh_.vertices_end(); ++vi)
    {
        MyMesh::VertexHandle vh = vi.handle();
        if(mesh_.data(vh).pinned())
            mesh_.data(vh).set_handled(true);
    }
}
