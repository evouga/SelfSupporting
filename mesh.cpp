#include "mesh.h"
#include <Eigen/Dense>
#include <iostream>
#include "eiquadprog.hpp"
#include <vector>
#include "solvers.h"
#include <Eigen/Sparse>
#include "meshrenderer.h"
#include "controller.h"


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

int Mesh::numFaceVerts(MyMesh::FaceHandle face)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    int result = 0;
    for(MyMesh::FaceVertexIter fvi = mesh_.fv_iter(face); fvi; ++fvi)
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
        int verts = numFaceVerts(it);
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

void Mesh::setPlaneAreaLoads()
{
    auto_ptr<MeshLock> ml = acquireMesh();
    for(MyMesh::VertexIter vi = mesh_.vertices_begin(); vi != mesh_.vertices_end(); ++vi)
    {
        double area = vertexAreaOnPlane(vi);
        mesh_.data(vi).set_load(area);
    }
    invalidateMesh();
}

void Mesh::setSurfaceAreaLoads()
{
    auto_ptr<MeshLock> ml = acquireMesh();

    for(MyMesh::VertexIter vi = mesh_.vertices_begin(); vi != mesh_.vertices_end(); ++vi)
    {
        double area = vertexArea(vi);
        mesh_.data(vi).set_load(area);
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

Vector3d Mesh::projectToFace(MyMesh::FaceHandle fh, const Vector3d &p)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    MyMesh::Point centroid;
    mesh_.calc_face_centroid(fh, centroid);

    MyMesh::Point n(0,0,0);
    for(MyMesh::ConstFaceVertexIter fv = mesh_.cfv_iter(fh); fv; ++fv)
    {
        MyMesh::VertexHandle vh = fv;
        MyMesh::Point tmp;
        mesh_.calc_vertex_normal_correct(vh, tmp);
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

Vector3d Mesh::approximateClosestPoint(const Vector3d &p)
{
    auto_ptr<MeshLock> ml = acquireMesh();
    int closestface = -1;
    double closestdist = std::numeric_limits<double>::infinity();

    for(MyMesh::ConstFaceIter f = mesh_.faces_begin(); f != mesh_.faces_end(); ++f)
    {
        MyMesh::FaceHandle fh = f;
        MyMesh::Point centroid;
        mesh_.calc_face_centroid(fh, centroid);
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
        MyMesh::FaceHandle fh = mesh_.face_handle(closestface);
        Vector3d newpos = projectToFace(fh, p);
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

