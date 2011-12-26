#include "mesh.h"
#include <Eigen/Dense>
#include <iostream>
#include "eiquadprog.hpp"
#include <vector>
#include "solvers.h"
#include <Eigen/Sparse>
#include "meshrenderer.h"
#include "controller.h"
#include <list>
#include "function.h"
#include <fstream>

using namespace std;
using namespace Eigen;

Mesh::Mesh(Controller &cont) : cont_(cont), meshMutex_(QMutex::Recursive), meshID_(0)
{
}

Mesh::~Mesh()
{
}

const MyMesh &Mesh::getMesh()
{
    return mesh_;
}

void Mesh::lockMesh()
{
    meshMutex_.lock();
}

void Mesh::unlockMesh()
{
    meshMutex_.unlock();
}

void Mesh::copyMesh(const MyMesh &m)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    mesh_ = m;
    invalidateMesh();
}

void Mesh::clearMesh()
{
    auto_ptr<MeshLock> ml = acquireMesh();
    mesh_ = MyMesh();
    invalidateMesh();
}

void Mesh::triangulate()
{
    auto_ptr<MeshLock> ml = acquireMesh();
    mesh_.triangulate();
    invalidateMesh();
}

Vector3d Mesh::computeCentroid()
{
    auto_ptr<MeshLock> ml = acquireMesh();
    Vector3d result;
    result.setZero();
    for(MyMesh::VertexIter vi = mesh_.vertices_begin(); vi != mesh_.vertices_end(); ++vi)
    {
        for(int i=0; i<3; i++)
            result[i] += mesh_.point(vi)[i];
    }
    result /= mesh_.n_vertices();
    return result;
}

double Mesh::computeBoundingSphere(const Eigen::Vector3d &center)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    double radius = 0;
    for(MyMesh::VertexIter vi = mesh_.vertices_begin(); vi != mesh_.vertices_end(); ++vi)
    {
        double thisr = 0;
        for(int i=0; i<3; i++)
            thisr += (mesh_.point(vi)[i]-center[i])*(mesh_.point(vi)[i]-center[i]);
        if(radius < thisr)
            radius = thisr;
    }
    return sqrt(radius);
}

double Mesh::computeBoundingCircle(const Eigen::Vector3d &center)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    double radius = 0;
    for(MyMesh::VertexIter vi = mesh_.vertices_begin(); vi != mesh_.vertices_end(); ++vi)
    {
        double thisr = 0;
        for(int i=0; i<3; i++)
        {
            if(i==1)
                continue;
            thisr += (mesh_.point(vi)[i]-center[i])*(mesh_.point(vi)[i]-center[i]);
        }
        if(radius < thisr)
            radius = thisr;
    }
    return sqrt(radius);
}

MyMesh::Point Mesh::computeEdgeMidpoint(MyMesh::EdgeHandle edge)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    MyMesh::HalfedgeHandle heh = mesh_.halfedge_handle(edge, 0);
    if(!heh.is_valid())
        heh = mesh_.halfedge_handle(edge, 1);
    assert(heh.is_valid());

    MyMesh::Point result = mesh_.point(mesh_.from_vertex_handle(heh));
    result += mesh_.point(mesh_.to_vertex_handle(heh));
    result *= 0.5;
    return result;
}

void Mesh::edgeEndpoints(MyMesh::EdgeHandle edge, MyMesh::Point &p1, MyMesh::Point &p2)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    MyMesh::HalfedgeHandle heh = mesh_.halfedge_handle(edge,0);
    if(!heh.is_valid())
        heh = mesh_.halfedge_handle(edge,1);
    assert(heh.is_valid());

    p1 = mesh_.point(mesh_.from_vertex_handle(heh));
    p2 = mesh_.point(mesh_.to_vertex_handle(heh));
}

double Mesh::randomDouble()
{
    return ((double)rand()/(double)RAND_MAX);
}

void Mesh::translateVertex(int vidx, const Eigen::Vector3d &translation)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    assert(vidx >= 0 && vidx < (int)mesh_.n_vertices());

    MyMesh::VertexHandle vh = mesh_.vertex_handle(vidx);
    MyMesh::Point &pt = mesh_.point(vh);
    for(int i=0; i<3; i++)
        pt[i] += translation[i];
    invalidateMesh();
}

void Mesh::translateFace(int fidx, const Vector3d &translation)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    assert(fidx >= 0 && fidx < (int)mesh_.n_faces());

    MyMesh::FaceHandle fh = mesh_.face_handle(fidx);
    for(MyMesh::FaceVertexIter fvi = mesh_.fv_iter(fh); fvi; ++fvi)
    {
        MyMesh::Point &pt = mesh_.point(fvi);
        pt[0] += translation[0];
        pt[1] += translation[1];
        pt[2] += translation[2];
    }
    invalidateMesh();
}

double Mesh::faceAreaOnPlane(MyMesh::FaceHandle face)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    MyMesh::FaceVertexIter prevvert = mesh_.fv_iter(face);
    MyMesh::FaceVertexIter curvert = prevvert;
    double area = 0;
    while(++curvert)
    {
        MyMesh::Point pt1 = mesh_.point(prevvert);
        MyMesh::Point pt2 = mesh_.point(curvert);
        area += pt1[0]*pt2[2]-pt1[2]*pt2[0];
        prevvert = curvert;
    }
    curvert = mesh_.fv_iter(face);
    MyMesh::Point last = mesh_.point(prevvert);
    MyMesh::Point first = mesh_.point(curvert);
    area += last[0]*first[2] - last[2]*first[0];

    area /= 2.0;
    return fabs(area);
}

int Mesh::numFaceVerts(const MyMesh &mesh, MyMesh::FaceHandle face)
{
    int result = 0;
    for(MyMesh::ConstFaceVertexIter fvi = mesh.cfv_iter(face); fvi; ++fvi)
    {
        result++;
    }
    return result;
}

double Mesh::vertexAreaOnPlane(MyMesh::VertexHandle vert)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    double ans = 0;
    for(MyMesh::VertexFaceIter it = mesh_.vf_iter(vert); it; ++it)
    {
        int verts = numFaceVerts(mesh_, it);
        double facearea = faceAreaOnPlane(it);
        ans += facearea/verts;
    }
    return ans;
}

double Mesh::vertexArea(MyMesh::VertexHandle vert)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    double ans = 0;
    for(MyMesh::VertexEdgeIter vei = mesh_.ve_iter(vert); vei; ++vei)
    {
        MyMesh::EdgeHandle eh = vei;
        ans += 0.5 * edgeArea(eh);
    }
    return ans;
}

double Mesh::edgeArea(MyMesh::EdgeHandle edge)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    double tot = 0;
    for(int i=0; i<2; i++)
    {
        MyMesh::HalfedgeHandle heh = mesh_.halfedge_handle(edge, i);
        if(heh.is_valid())
        {
            MyMesh::Point centroid;
            MyMesh::FaceHandle fh = mesh_.face_handle(heh);
            if(fh.is_valid())
            {
                mesh_.calc_face_centroid(fh, centroid);
                MyMesh::Point pt1 = mesh_.point(mesh_.to_vertex_handle(heh));
                MyMesh::Point pt2 = mesh_.point(mesh_.from_vertex_handle(heh));
                Vector3d ab, ac;
                for(int j=0; j<3; j++)
                {
                    ab[j] = pt1[j]-centroid[j];
                    ac[j] = pt2[j]-centroid[j];
                }
                tot += 0.5 * sqrt( ab.dot(ab)*ac.dot(ac) - ab.dot(ac) * ab.dot(ac) );
            }
        }
    }
    return tot;
}

void Mesh::setConstantLoads(double density)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    for(MyMesh::VertexIter vi = mesh_.vertices_begin(); vi != mesh_.vertices_end(); ++vi)
    {
        mesh_.data(vi).set_load(density);
    }
    invalidateMesh();
}

void Mesh::setPlaneAreaLoads(double density)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    for(MyMesh::VertexIter vi = mesh_.vertices_begin(); vi != mesh_.vertices_end(); ++vi)
    {
        double area = vertexAreaOnPlane(vi);
        mesh_.data(vi).set_load(density*area);
    }
    invalidateMesh();
}

void Mesh::setSurfaceAreaLoads(double density)
{
    auto_ptr<MeshLock> ml = acquireMesh();

    double total = 0;
    for(MyMesh::VertexIter vi = mesh_.vertices_begin(); vi != mesh_.vertices_end(); ++vi)
    {
        double area = vertexArea(vi);
        total += density*area;
        mesh_.data(vi).set_load(density*area);
    }
    invalidateMesh();
}

void Mesh::getNRing(int vidx, int n, set<int> &nring)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    //TODO optimize away interior
    int numv = mesh_.n_vertices();
    assert(vidx >= 0 && vidx < numv);

    set<int> tmp1, tmp2;

    set<int> *result = &tmp1;
    set<int> *newresult = &tmp2;
    result->insert(vidx);

    for(int i=0; i<n; i++)
    {
        for(set<int>::iterator it = result->begin(); it != result->end(); ++it)
        {
            MyMesh::VertexHandle vh = mesh_.vertex_handle(*it);
            newresult->insert(*it);
            for(MyMesh::VertexVertexIter vv = mesh_.vv_iter(vh); vv; ++vv)
            {
                MyMesh::VertexHandle adj = vv;
                newresult->insert(adj.idx());
            }
        }
        std::swap(result, newresult);
    }
    nring = *result;
}

Vector3d Mesh::projectToFaceZ(const MyMesh &mesh, MyMesh::FaceHandle fh, const Vector3d &p)
{
    double a,b,c;
    computeFacePlane(mesh, fh, a, b, c);
    double z = a*p[0]+b*p[2]+c;
    Vector3d result(p[0],z,p[2]);
    return result;
}

Vector3d Mesh::projectToFace(const MyMesh &mesh, MyMesh::FaceHandle fh, const Vector3d &p)
{
    MyMesh::Point centroid;
    mesh.calc_face_centroid(fh, centroid);

    MyMesh::Point n(0,0,0);
    for(MyMesh::ConstFaceVertexIter fv = mesh.cfv_iter(fh); fv; ++fv)
    {
        MyMesh::VertexHandle vh = fv;
        MyMesh::Point tmp;
        mesh.calc_vertex_normal_correct(vh, tmp);
        n += tmp;
    }
    Eigen::Vector3d normal(n[0], n[1], n[2]);
    normal.normalize();

    Vector3d diff = p;
    for(int j=0; j<3; j++)
        diff[j] -= centroid[j];
    Vector3d result = p - diff.dot(normal) * normal;
    return result;
}

Vector3d Mesh::approximateClosestPoint(const MyMesh &mesh, const Vector3d &p)
{
    int closestface = -1;
    double closestdist = std::numeric_limits<double>::infinity();

    for(MyMesh::ConstFaceIter f = mesh.faces_begin(); f != mesh.faces_end(); ++f)
    {
        MyMesh::FaceHandle fh = f;
        MyMesh::Point centroid;
        mesh.calc_face_centroid(fh, centroid);
        double dist = 0;
        for(int j=0; j<3; j++)
            dist += (p[j]-centroid[j])*(p[j]-centroid[j]);
        if(dist < closestdist)
        {
            closestdist = dist;
            closestface = fh.idx();
        }
    }

    if(closestface != -1)
    {
        MyMesh::FaceHandle fh = mesh.face_handle(closestface);
        Vector3d newpos = projectToFace(mesh, fh, p);
        return newpos;
    }
    return p;
}

Vector3d Mesh::approximateClosestZParallel(const MyMesh &mesh, const Eigen::Vector3d &p)
{
    int closestface = -1;
    double closestdist = std::numeric_limits<double>::infinity();

    for(MyMesh::ConstFaceIter f = mesh.faces_begin(); f != mesh.faces_end(); ++f)
    {
        MyMesh::FaceHandle fh = f;
        MyMesh::Point centroid;
        mesh.calc_face_centroid(fh, centroid);
        double dist = (p[0]-centroid[0])*(p[0]-centroid[0])+(p[2]-centroid[2])*(p[2]-centroid[2]);
        if(dist < closestdist)
        {
            closestdist = dist;
            closestface = fh.idx();
        }
    }

    if(closestface != -1)
    {
        MyMesh::FaceHandle fh = mesh.face_handle(closestface);
        Vector3d newpos = projectToFaceZ(mesh, fh, p);
        return newpos;
    }
    return p;
}

int Mesh::getMeshID()
{
    auto_ptr<MeshLock> ml = acquireMesh();
    idMutex_.lock();
    int result = meshID_;
    idMutex_.unlock();
    return result;
}

void Mesh::invalidateMesh()
{
    auto_ptr<MeshLock> ml = acquireMesh();
    idMutex_.lock();
    meshID_++;
    idMutex_.unlock();
}

auto_ptr<MeshLock> Mesh::acquireMesh()
{
    return auto_ptr<MeshLock>(new MeshLock(*this));
}




MeshLock::MeshLock(Mesh &m) : m_(m)
{
    m.lockMesh();
}

MeshLock::~MeshLock()
{
    m_.unlockMesh();
}

void Mesh::subdivide(bool subdivideBoundary)
{
    auto_ptr<MeshLock> ml = acquireMesh();
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
        bool pinned = edgePinned(eh);

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
        MyMesh::VertexHandle newv = newmesh.add_vertex(midp);
        newmesh.data(newv).set_pinned(pinned);
    }

    // add (modified) original vertices
    for(int i=0; i<n; i++)
    {
        MyMesh::VertexHandle vh = mesh_.vertex_handle(i);
        bool pinned = mesh_.data(vh).pinned();
        MyMesh::Point newpt(0,0,0);
        if(mesh_.is_boundary(vh))
        {
            if(subdivideBoundary)
            {
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
                newpt = mesh_.point(vh);
            }
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
        MyMesh::VertexHandle newv = newmesh.add_vertex(newpt);
        newmesh.data(newv).set_pinned(pinned);
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
}

bool Mesh::edgePinned(MyMesh::EdgeHandle edge)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    MyMesh::HalfedgeHandle heh = mesh_.halfedge_handle(edge, 0);
    if(!heh.is_valid())
        heh = mesh_.halfedge_handle(edge, 1);
    assert(heh.is_valid());

    return mesh_.data(mesh_.from_vertex_handle(heh)).pinned() && mesh_.data(mesh_.to_vertex_handle(heh)).pinned();
}

bool Mesh::faceContainsPinnedVert(MyMesh::FaceHandle face)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    for(MyMesh::FaceVertexIter fvi = mesh_.fv_iter(face); fvi; ++fvi)
    {
        if(mesh_.data(fvi.handle()).pinned())
            return true;
    }
    return false;
}

void Mesh::removeVertex(MyMesh::VertexHandle vert)
{
    auto_ptr<MeshLock> ml = acquireMesh();

    MyMesh::HalfedgeHandle startheh = mesh_.voh_iter(vert).handle();

    int numboundary = 0;
    for(MyMesh::VertexOHalfedgeIter voh = mesh_.voh_iter(vert); voh; ++voh)
    {
        if(mesh_.is_boundary(mesh_.opposite_halfedge_handle(voh.handle())))
        {
            numboundary++;
            startheh = voh.handle();
            cout << "start on boundary" << endl;
        }
    }
    assert(numboundary <= 1);
    assert(startheh.is_valid());

    vector<int> neighbors;
    set<int> nbset;
    set<int> visitedset;
    visitedset.insert(vert.idx());

    set<int> adjfaces;
    for(MyMesh::VertexFaceIter vfi = mesh_.vf_iter(vert); vfi; ++vfi)
        adjfaces.insert(vfi.handle().idx());

    MyMesh::HalfedgeHandle curheh = startheh;
    while(true)
    {
        assert(curheh.is_valid());
        assert(!mesh_.is_boundary(curheh));
        assert(adjfaces.count(mesh_.face_handle(curheh).idx()) > 0);

        //if the to vertex is a bona fide one-ring boundary vertex
        int idx = mesh_.to_vertex_handle(curheh).idx();
        if(mesh_.is_boundary(mesh_.opposite_halfedge_handle(curheh)) || adjfaces.count(mesh_.opposite_face_handle(curheh).idx()) == 0)
        {
            cout << "next " << idx << endl;
            neighbors.push_back(idx);
            nbset.insert(idx);
        }
        visitedset.insert(idx);

        curheh = mesh_.next_halfedge_handle(curheh);
        //if we've arrived back at the start, flip
        if(mesh_.to_vertex_handle(curheh) == vert)
        {
            curheh = mesh_.opposite_halfedge_handle(curheh);
            cout << "flip" << endl;
        }

        if(curheh == startheh)
            break;
        if(mesh_.is_boundary(curheh))
        {
            cout << "stop due to bdry" << endl;
            break;
        }
    }

    for(set<int>::iterator it = visitedset.begin(); it != visitedset.end(); ++it)
    {
        if(!nbset.count(*it))
            mesh_.delete_vertex(mesh_.vertex_handle(*it), false);
    }

    vector<MyMesh::VertexHandle> nbhandles;
    for(vector<int>::iterator it = neighbors.begin(); it != neighbors.end(); ++it)
    {
        MyMesh::VertexHandle vh = mesh_.vertex_handle(*it);
        assert(vh.is_valid());
        nbhandles.push_back(vh);
    }

    if(nbhandles.size() > 2)
    {
        mesh_.add_face(nbhandles);
    }

    mesh_.garbage_collection();

    invalidateMesh();
}

void Mesh::computeFacePlane(const MyMesh &mesh, MyMesh::FaceHandle face, double &a, double &b, double &c)
{
    int numv = numFaceVerts(mesh, face);

    MatrixXd M(numv,3);
    VectorXd rhs(numv);
    int row = 0;
    for(MyMesh::ConstFaceVertexIter fvi = mesh.cfv_iter(face); fvi; ++fvi)
    {
        MyMesh::Point pt = mesh.point(fvi.handle());
        M(row,0) = pt[0];
        M(row,1) = pt[2];
        M(row,2) = 1.0;
        rhs[row] = pt[1];
        row++;
    }
    assert(row == numv);
    Vector3d sol = (M.transpose()*M).fullPivLu().solve(M.transpose()*rhs);
    a = sol[0];
    b = sol[1];
    c = sol[2];
}

double Mesh::isotropicDihedralAngle(MyMesh::EdgeHandle edge)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    assert(!mesh_.is_boundary(edge));
    MyMesh::HalfedgeHandle heh = mesh_.halfedge_handle(edge,0);
    MyMesh::FaceHandle fh1 = mesh_.face_handle(heh);
    assert(fh1.is_valid());
    double a1,b1,c;
    computeFacePlane(mesh_,fh1, a1, b1, c);
    MyMesh::FaceHandle fh2 = mesh_.opposite_face_handle(heh);
    assert(fh2.is_valid());
    double a2,b2;
    computeFacePlane(mesh_,fh2, a2, b2, c);
    return sqrt((a2-a1)*(a2-a1) + (b2-b1)*(b2-b1));
}

Matrix2d Mesh::approximateHessian(MyMesh::FaceHandle face)
{
    auto_ptr<MeshLock> ml = acquireMesh();

    set<int> uniquepts;
    for(MyMesh::FaceVertexIter fvi = mesh_.fv_iter(face); fvi; ++fvi)
    {
        uniquepts.insert(fvi.handle().idx());
    }
    for(MyMesh::FaceFaceIter ffi = mesh_.ff_iter(face); ffi; ++ffi)
    {
        for(MyMesh::FaceVertexIter fvi = mesh_.fv_iter(ffi.handle()); fvi; ++fvi)
        {
            uniquepts.insert(fvi.handle().idx());
        }
    }

    int numpts = uniquepts.size();

    MatrixXd LS(numpts, 6);
    VectorXd rhs(numpts);

    uniquepts.clear();

    int curpt = 0;
    for(MyMesh::FaceVertexIter fvi = mesh_.fv_iter(face); fvi; ++fvi)
    {
        if(uniquepts.count(fvi.handle().idx()) > 0)
            continue;
        uniquepts.insert(fvi.handle().idx());
        MyMesh::Point pt = mesh_.point(fvi.handle());
        LS(curpt, 0) = pt[0]*pt[0];
        LS(curpt, 1) = pt[0]*pt[2];
        LS(curpt, 2) = pt[2]*pt[2];
        LS(curpt, 3) = pt[0];
        LS(curpt, 4) = pt[2];
        LS(curpt, 5) = 1;
        rhs[curpt] = pt[1];
        curpt++;
    }

    for(MyMesh::FaceFaceIter ffi = mesh_.ff_iter(face); ffi; ++ffi)
    {
        for(MyMesh::FaceVertexIter fvi = mesh_.fv_iter(ffi.handle()); fvi; ++fvi)
        {
            if(uniquepts.count(fvi.handle().idx()) > 0)
                continue;
            uniquepts.insert(fvi.handle().idx());

            MyMesh::Point pt = mesh_.point(fvi.handle());
            LS(curpt, 0) = pt[0]*pt[0];
            LS(curpt, 1) = pt[0]*pt[2];
            LS(curpt, 2) = pt[2]*pt[2];
            LS(curpt, 3) = pt[0];
            LS(curpt, 4) = pt[2];
            LS(curpt, 5) = 1;
            rhs[curpt] = pt[1];
            curpt++;
        }
    }
    assert(curpt == numpts);

    VectorXd sol = (LS.transpose()*LS).ldlt().solve(LS.transpose()*rhs);
    Matrix2d hess;
    hess << 2*sol[0], sol[1], sol[1], 2*sol[2];
    return hess;
}

Matrix2d Mesh::approximateHessianVertex(MyMesh::VertexHandle vert)
{
    auto_ptr<MeshLock> ml = acquireMesh();

    int numpts = mesh_.valence(vert);

    MatrixXd LS(numpts, 5);
    VectorXd rhs(numpts);

    int curpt = 0;

    MyMesh::Point cpt = mesh_.point(vert);

    for(MyMesh::VertexVertexIter vvi = mesh_.vv_iter(vert); vvi; ++vvi)
    {
        MyMesh::Point pt = mesh_.point(vvi.handle());
        LS(curpt, 0) = (pt[0]-cpt[0])*(pt[0]-cpt[0]);
        LS(curpt, 1) = (pt[0]-cpt[0])*(pt[2]-cpt[2]);
        LS(curpt, 2) = (pt[2]-cpt[2])*(pt[2]-cpt[2]);
        LS(curpt, 3) = (pt[0]-cpt[0]);
        LS(curpt, 4) = (pt[2]-cpt[2]);
        rhs[curpt] = pt[1]-cpt[1];
        curpt++;
    }
    assert(curpt == numpts);

    VectorXd sol = (LS.transpose()*LS).ldlt().solve(LS.transpose()*rhs);
    Matrix2d hess;
    hess << 2*sol[0], sol[1], sol[1], 2*sol[2];
    return hess;
}


bool Mesh::loadMesh(const char *name)
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
                bool handled;
                ifs >> load;
                ifs >> pinned;
                ifs >> handled;
                MyMesh::VertexHandle vh = newmesh.add_vertex(pt);
                newmesh.data(vh).set_load(load);
                newmesh.data(vh).set_pinned(pinned);
                newmesh.data(vh).set_handled(handled);
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
                        double weight;
                        ifs >> crease;
                        ifs >> creaseval;
                        ifs >> weight;
                        MyMesh::EdgeHandle eh = newmesh.edge_handle(hei.handle());
                        newmesh.data(eh).set_weight(weight);
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

bool Mesh::saveMesh(const char *name)
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
        bool handled = mesh_.data(vi.handle()).handled();
        ofs << "v " << pt[0] << " " << pt[1] << " " << pt[2] << " " << load << " " << pinned << " " << handled << endl;
        if(!ofs)
            return false;
    }

    for(MyMesh::FaceIter fi = mesh_.faces_begin(); fi != mesh_.faces_end(); ++fi)
    {
        ofs << "f ";
        int numverts = this->numFaceVerts(mesh_, fi.handle());
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
        double weight = mesh_.data(ei.handle()).weight();
        ofs << " " << 0 << " " << 0 << " " << weight << endl;
        if(!ofs)
            return false;
    }

    return ofs;
}

bool Mesh::importOBJ(const char *name)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    OpenMesh::IO::Options opt;
    return OpenMesh::IO::read_mesh(mesh_, name, opt);
}

bool Mesh::exportOBJ(const char *name)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    OpenMesh::IO::Options opt;
    return OpenMesh::IO::write_mesh(mesh_, name, opt);
}

void Mesh::computeEigenstuff(const Matrix2d &M, double &lambda1, double &lambda2, Vector2d &u, Vector2d &v)
{
    double a = M(0,0);
    double b = M(0,1);
    double c = M(1,0);
    double d = M(1,1);

    double discr = (a-d)*(a-d)+4*b*c;
    lambda1 = 0.5*(a+d+sqrt(discr));
    lambda2 = 0.5*(a+d-sqrt(discr));

    if(fabs(b) < 1e-8)
    {
        if(fabs(c) < 1e-8)
        {
            if(a > d)
            {
                u[0] = v[1] = 1.0;
                u[1] = v[0] = 0.0;
            }
            else
            {
                u[0] = v[1] = 0.0;
                u[1] = v[0] = 1.0;
            }
        }
        else
        {
            u[1] = v[1] = 1.0;
            u[0] = (lambda1-d)/c;
            v[0] = (lambda2-d)/c;
        }
    }
    else
    {
        u[0] = v[0] = 1.0;
        u[1] = (lambda1-a)/b;
        v[1] = (lambda2-a)/b;
    }
}

void Mesh::computePlaneBarycentricCoordinates(const Eigen::Vector2d &pt, MyMesh::FaceHandle face, double &alpha, double &beta)
{
    assert(numFaceVerts(mesh_, face) == 3);

    MyMesh::Point verts[3];
    int idx=0;
    for(MyMesh::FaceVertexIter fvi = mesh_.fv_iter(face); fvi; ++fvi)
    {
        verts[idx] = mesh_.point(fvi.handle());
        idx++;
    }
    assert(idx==3);

    double a,b,c,d;
    a = verts[1][0]-verts[0][0];
    b = verts[2][0]-verts[0][0];
    c = verts[1][2]-verts[0][2];
    d = verts[2][2]-verts[0][2];
    double det = a*d-b*c;
    Matrix2d E;
    E << d,-b,-c,a;
    Vector2d rhs(det*(pt[0]-verts[0][0]), det*(pt[1]-verts[0][2]));
    Vector2d bary = E*rhs;
    alpha = bary[0];
    beta = bary[1];
}

int Mesh::findEnclosingPlaneTriangle(const Eigen::Vector2d &pt, double &alpha, double &beta)
{
    for(MyMesh::FaceIter fi = mesh_.faces_begin(); fi != mesh_.faces_end(); ++fi)
    {
        if(numFaceVerts(mesh_, fi.handle()) == 3)
        {
            computePlaneBarycentricCoordinates(pt, fi.handle(), alpha, beta);
            if(alpha >= 0 && beta >= 0 && 1-alpha-beta >= 0)
                return fi.handle().idx();
        }
    }
    return -1;
}
