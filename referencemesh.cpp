#include "referencemesh.h"
#include "referencemeshrenderer.h"
#include "controller.h"
#include "solvers.h"
#include "networkmesh.h"
#include <fstream>

#include <vector>

#include <Eigen/Sparse>

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

bool ReferenceMesh::importOBJ(const char *name)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    OpenMesh::IO::Options opt;
    return OpenMesh::IO::read_mesh(mesh_, name, opt);
}

bool ReferenceMesh::exportOBJ(const char *name)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    OpenMesh::IO::Options opt;
    return OpenMesh::IO::write_mesh(mesh_, name, opt);
}

bool ReferenceMesh::loadMesh(const char *name)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    ifstream ifs(name);
    if(!ifs)
        return false;

    MyMesh newmesh;
    while(ifs)
    {
        char code;
        ifs >> code;
        if(ifs.eof())
        {
            mesh_ = newmesh;
            return true;
        }
        switch(code)
        {
            case 'v':
            {
                MyMesh::Point pt;
                ifs >> pt[0];
                ifs >> pt[1];
                ifs >> pt[2];
                double load;
                bool pinned;
                bool anchored;
                ifs >> load;
                ifs >> pinned;
                ifs >> anchored;
                MyMesh::VertexHandle vh = newmesh.add_vertex(pt);
                newmesh.data(vh).set_load(load);
                newmesh.data(vh).set_pinned(pinned);
                newmesh.data(vh).set_anchored(anchored);
                break;
            }
            case 'f':
            {
                int numpts;
                ifs >> numpts;
                vector<MyMesh::VertexHandle> toadd;
                for(int i=0; i<numpts; i++)
                {
                    int vid;
                    ifs >> vid;
                    if(vid >= (int)newmesh.n_vertices())
                        return false;
                    toadd.push_back(newmesh.vertex_handle(vid));
                }
                newmesh.add_face(toadd);
                break;
            }
            case 'e':
            {
                int v1, v2;
                ifs >> v1 >> v2;
                if(v1 >= (int)newmesh.n_vertices() || v2 >= (int)newmesh.n_vertices())
                    return false;
                bool found = false;
                for(MyMesh::HalfedgeIter hei = newmesh.halfedges_begin(); hei != newmesh.halfedges_end(); ++hei)
                {
                    if(newmesh.from_vertex_handle(hei.handle()).idx() == v1 && newmesh.to_vertex_handle(hei.handle()).idx() == v2)
                    {
                        bool crease;
                        double creaseval;
                        ifs >> crease;
                        ifs >> creaseval;
                        MyMesh::EdgeHandle eh = newmesh.edge_handle(hei.handle());
                        newmesh.data(eh).set_is_crease(crease);
                        newmesh.data(eh).set_crease_value(creaseval);
                        found = true;
                        break;
                    }
                }
                if(!found)
                    return false;
                break;
            }
            default:
                return false;
        }
    }

    return false;
}

bool ReferenceMesh::saveMesh(const char *name)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    ofstream ofs(name);
    if(!ofs)
        return false;

    for(MyMesh::VertexIter vi = mesh_.vertices_begin(); vi != mesh_.vertices_end(); ++vi)
    {
        MyMesh::Point pt = mesh_.point(vi.handle());
        double load = mesh_.data(vi.handle()).load();
        bool pinned = mesh_.data(vi.handle()).pinned();
        bool anchored = mesh_.data(vi.handle()).anchored();
        ofs << "v " << pt[0] << " " << pt[1] << " " << pt[2] << " " << load << " " << pinned << " " << anchored << endl;
        if(!ofs)
            return false;
    }

    for(MyMesh::FaceIter fi = mesh_.faces_begin(); fi != mesh_.faces_end(); ++fi)
    {
        ofs << "f ";
        int numverts = this->numFaceVerts(fi.handle());
        ofs << numverts << " ";
        for(MyMesh::FaceVertexIter fvi = mesh_.fv_iter(fi.handle()); fvi; ++fvi)
        {
            ofs << fvi.handle().idx() << " ";
        }
        ofs << endl;
        if(!ofs)
            return false;
    }

    for(MyMesh::EdgeIter ei = mesh_.edges_begin(); ei != mesh_.edges_end(); ++ei)
    {
        MyMesh::HalfedgeHandle heh = mesh_.halfedge_handle(ei.handle(), 0);
        assert(heh.is_valid());
        ofs << "e " << mesh_.from_vertex_handle(heh).idx() << " " << mesh_.to_vertex_handle(heh).idx();
        bool creased = mesh_.data(ei.handle()).is_crease();
        double creaseval = mesh_.data(ei.handle()).crease_value();
        ofs << " " << creased << " " << creaseval << endl;
        if(!ofs)
            return false;
    }

    return ofs;
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

void ReferenceMesh::applyLaplacianDeformationHeight(int, const Vector3d &delta)
{
    auto_ptr<MeshLock> ml = acquireMesh();

    int n = mesh_.n_vertices();
    DynamicSparseMatrix<double> L(n,n);
    for(int i=0; i<n; i++)
    {
        MyMesh::VertexHandle vh = mesh_.vertex_handle(i);
        int valence = mesh_.valence(vh);
        L.coeffRef(i,i) = 1.0;
        for(MyMesh::VertexVertexIter vv = mesh_.vv_iter(vh); vv; ++vv)
        {
            MyMesh::VertexHandle adj = vv;
            int adjidx = adj.idx();
            L.coeffRef(i, adjidx) = -1.0/valence;
        }
    }

    DynamicSparseMatrix<double> LTL(L.transpose()*L);
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
        if(mesh_.data(vh).anchored())
        {
            MyMesh::Point movedpt = mesh_.point(vh);
            rhs[i] += (movedpt[1]+delta[1]);
            LTL.coeffRef(i, i) += 1.0;
        }
        else if(mesh_.data(vh).pinned())
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
        MyMesh::Point &pt = mesh_.point(vh);
        if(!mesh_.data(vh).pinned())
            pt[1] = v0[i];
    }
}

void ReferenceMesh::applyLaplacianDeformation(int, const Vector3d &delta)
{
    auto_ptr<MeshLock> ml = acquireMesh();

    int n = mesh_.n_vertices();

    DynamicSparseMatrix<double> L(3*n,3*n);
    for(int i=0; i<n; i++)
    {
        MyMesh::VertexHandle vh = mesh_.vertex_handle(i);
        int valence = mesh_.valence(vh);
        for(int j=0; j<3; j++)
            L.coeffRef(3*i+j, 3*i+j) = 1.0;
        for(MyMesh::VertexVertexIter vv = mesh_.vv_iter(vh); vv; ++vv)
        {
            MyMesh::VertexHandle adj = vv;
            int adjidx = adj.idx();
            for(int j=0; j<3; j++)
                L.coeffRef(3*i+j, 3*adjidx+j) = -1.0/valence;
        }
    }

    DynamicSparseMatrix<double> LTL(L.transpose()*L);
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
        if(mesh_.data(vh).anchored())
        {
            MyMesh::Point movedpt = mesh_.point(vh);
            for(int j=0; j<3; j++)
            {
                rhs[3*i+j] += (movedpt[j] + delta[j]);
                LTL.coeffRef(3*i+j,3*i+j) += 1.0;
            }
        }
        else if(mesh_.data(vh).pinned())
        {
            LTL.coeffRef(3*i + 1, 3*i + 1) += 1.0;
            rhs[3*i+1] += mesh_.point(vh)[1];
        }
    }

    SparseMatrix<double> M(LTL);

    cont_.getSolvers().linearSolveCG(M, rhs, v0);

    for(int i=0; i<n; i++)
    {
        MyMesh::VertexHandle vh = mesh_.vertex_handle(i);
        MyMesh::Point &pt = mesh_.point(vh);
        pt[0] = v0[3*i];
        if(!mesh_.data(vh).pinned())
            pt[1] = v0[3*i+1];
        pt[2] = v0[3*i+2];
    }
}

void ReferenceMesh::setAnchor(int vidx, bool state)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    int n = mesh_.n_vertices();
    assert(vidx >= 0 && vidx < n);

    mesh_.data(mesh_.vertex_handle(vidx)).set_anchored(state);
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

void ReferenceMesh::setCrease(int eidx, bool state)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    int e = mesh_.n_edges();
    assert(eidx >= 0 && eidx < e);
    MyMesh::EdgeHandle eh = mesh_.edge_handle(eidx);
    mesh_.data(eh).set_is_crease(state);
    if(state)
        mesh_.data(eh).set_crease_value(1.0);
}

bool ReferenceMesh::isCrease(int eidx)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    int e = mesh_.n_edges();
    assert(eidx >= 0 && eidx < e);
    MyMesh::EdgeHandle eh = mesh_.edge_handle(eidx);
    return mesh_.data(eh).is_crease();
}

void ReferenceMesh::pinBoundary()
{
    auto_ptr<MeshLock> ml = acquireMesh();
    for(MyMesh::VertexIter vi = mesh_.vertices_begin(); vi != mesh_.vertices_end(); ++vi)
    {
        mesh_.data(vi.handle()).set_pinned( mesh_.is_boundary(vi.handle()) );
    }
}
