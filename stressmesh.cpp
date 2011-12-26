#include "stressmesh.h"
#include "networkmesh.h"
#include "stressmeshrenderer.h"
#include <stack>
#include <set>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

StressMesh::StressMesh(Controller &c) : Mesh(c)
{
    renderer_ = new StressMeshRenderer(*this);
}

MeshRenderer &StressMesh::getRenderer()
{
    return *renderer_;
}


void StressMesh::buildFromThrustNetwork(NetworkMesh &nm)
{
    auto_ptr<MeshLock> nml = nm.acquireMesh();
    auto_ptr<MeshLock> ml = acquireMesh();

    mesh_.clear();

    // split thrust network into polygon soup
    for(MyMesh::ConstFaceIter fi = nm.getMesh().faces_begin(); fi != nm.getMesh().faces_end(); ++fi)
    {
        vector<MyMesh::VertexHandle> toadd;
        for(MyMesh::ConstFaceVertexIter fvi = nm.getMesh().cfv_iter(fi.handle()); fvi; ++fvi)
        {
            MyMesh::Point pt = nm.getMesh().point(fvi.handle());
            MyMesh::VertexHandle newv = mesh_.add_vertex(pt);
            toadd.push_back(newv);
        }
        MyMesh::FaceHandle fh = mesh_.add_face(toadd);
        mesh_.data(fh).set_integrated(false);
    }

    for(MyMesh::FaceIter fi = mesh_.faces_begin(); fi != mesh_.faces_end(); ++fi)
    {
        if(!mesh_.data(fi).integrated())
            buildConnectedComponent(nm, fi.handle());
    }

    straightenStressSurface();
}

void StressMesh::straightenStressSurface()
{
    int n = mesh_.n_vertices();
    MatrixXd LS(n, 3);
    VectorXd rhs(n);
    int row=0;
    for(MyMesh::VertexIter fi = mesh_.vertices_begin(); fi != mesh_.vertices_end(); ++fi)
    {
        MyMesh::Point pt = mesh_.point(fi.handle());
        LS(row, 0) = pt[0];
        LS(row, 1) = pt[2];
        LS(row, 2) = 1.0;
        rhs[row] = pt[1];
        row++;
    }
    assert(row == n);

    Vector3d ans = (LS.transpose()*LS).llt().solve(LS.transpose()*rhs);
    for(int i=0; i<n; i++)
    {
        MyMesh::VertexHandle vh = mesh_.vertex_handle(i);
        MyMesh::Point &pt = mesh_.point(vh);
        pt[1] -= ans[0]*pt[0] + ans[1]*pt[2] + ans[2];
    }
}

void StressMesh::buildConnectedComponent(NetworkMesh &nm, MyMesh::FaceHandle face)
{
    stack<int> tointegrate;
    set<int> queued;
    tointegrate.push(face.idx());
    queued.insert(face.idx());

    while(!tointegrate.empty())
    {
        int nextface = tointegrate.top();
        tointegrate.pop();
        MyMesh::FaceHandle fh = mesh_.face_handle(nextface);
        assert(!mesh_.data(fh).integrated());
        integrateFace(nm, fh);

        for(MyMesh::ConstFaceFaceIter ffi = nm.getMesh().cff_iter(nm.getMesh().face_handle(fh.idx())); ffi; ++ffi)
        {
            int fidx = ffi.handle().idx();
            if(!mesh_.data(mesh_.face_handle(fidx)).integrated() && queued.count(fidx) == 0)
            {
                tointegrate.push(fidx);
                queued.insert(fidx);
            }
        }
    }
}

void StressMesh::integrateFace(NetworkMesh &nm, MyMesh::FaceHandle face)
{
    MyMesh::FaceHandle neighbor;
    bool nbfound = false;

    MyMesh::FaceHandle nmf = nm.getMesh().face_handle(face.idx());

    for(MyMesh::ConstFaceFaceIter ffi = nm.getMesh().cff_iter(nmf); ffi; ++ffi)
    {
        MyMesh::FaceHandle sfm = mesh_.face_handle(ffi.handle().idx());
        if(mesh_.data(sfm).integrated())
        {
            neighbor = ffi.handle();
            nbfound = true;
        }
    }

    if(!nbfound)
    {
        //isolated face
        //set all heights 0
        for(MyMesh::FaceVertexIter fvi = mesh_.fv_iter(face); fvi; ++fvi)
        {
            mesh_.point(fvi.handle())[1] = 0.0;
        }
    }
    else
    {
        // find halfedge
        MyMesh::HalfedgeHandle heh;
        for(MyMesh::ConstFaceHalfedgeIter fhi = nm.getMesh().cfh_iter(nmf); fhi; ++fhi)
        {
            if(nm.getMesh().opposite_face_handle(fhi.handle()) == neighbor)
            {
                heh = fhi.handle();
                break;
            }
        }
        assert(heh.is_valid());

        double a,b,c;
        MyMesh::FaceHandle sneighbor = mesh_.face_handle(neighbor.idx());
        computeFacePlane(mesh_, sneighbor, a, b, c);
        MyMesh::Point top = nm.getMesh().point(nm.getMesh().to_vertex_handle(heh));
        MyMesh::Point fromp = nm.getMesh().point(nm.getMesh().from_vertex_handle(heh));
        double vx = top[0]-fromp[0];
        double vy = top[2]-fromp[2];
        double weight = nm.getMesh().data(nm.getMesh().edge_handle(heh)).weight();
        double newa = a - vy*weight;
        double newb = b + vx*weight;

        //find vertex
        MyMesh::VertexHandle corrv;
        for(MyMesh::FaceVertexIter fvi = mesh_.fv_iter(sneighbor); fvi; ++fvi)
        {
            MyMesh::Point cpt = mesh_.point(fvi.handle());
            if(cpt[0] == top[0] && cpt[2] == top[2])
            {
                corrv = fvi.handle();
                break;
            }
        }
        assert(corrv.is_valid());
        MyMesh::Point cpt = mesh_.point(corrv);
        double newc = cpt[1] - newa*cpt[0] - newb*cpt[2];

        for(MyMesh::FaceVertexIter fvi = mesh_.fv_iter(face); fvi; ++fvi)
        {
            MyMesh::Point &pt = mesh_.point(fvi.handle());
            pt[1] = newa*pt[0] + newb*pt[2] + newc;
        }
    }

    mesh_.data(face).set_integrated(true);
}

double StressMesh::stressLoad(MyMesh::VertexHandle vert)
{
    assert(!mesh_.is_boundary(vert));
    MyMesh::Point cent = mesh_.point(vert);
    double result = 0;
    for(MyMesh::VertexOHalfedgeIter voh = mesh_.voh_iter(vert); voh; ++voh)
    {
        MyMesh::Point adj = mesh_.point(mesh_.to_vertex_handle(voh.handle()));
        result += mesh_.data(mesh_.edge_handle(voh.handle())).weight() * (cent[1]-adj[1]);
    }
    return result;
}

double StressMesh::reciprocalArea(MyMesh::VertexHandle vert)
{
    assert(!mesh_.is_boundary(vert));
    MyMesh::Point cent = mesh_.point(vert);
    double result = 0;
    double adjx = 0;
    double adjy = 0;
    for(MyMesh::VertexOHalfedgeIter voh = mesh_.voh_iter(vert); voh; ++voh)
    {
        MyMesh::Point adj = mesh_.point(mesh_.to_vertex_handle(voh.handle()));
        double weight = mesh_.data(mesh_.edge_handle(voh.handle())).weight();
        double dx = -weight*(adj[2]-cent[2]);
        double dy = weight*(adj[0]-cent[0]);
        result += 0.5*(adjx*(adjy+dy) - (adjx+dx)*adjy);
        adjx += dx;
        adjy += dy;
    }
    return result;
}

