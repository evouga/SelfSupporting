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

Mesh::Mesh(Controller &cont) : cont_(cont), meshMutex_(QMutex::Recursive)
{
}

Mesh::~Mesh()
{
}

const MyMesh &Mesh::getMesh()
{
    lockMesh();
    return mesh_;
}

void Mesh::lockMesh()
{
    meshMutex_.lock();
}

void Mesh::releaseMesh()
{
    meshMutex_.unlock();
}

void Mesh::copyMesh(const MyMesh &m)
{
    lockMesh();
    mesh_ = m;
    releaseMesh();
}

void Mesh::clearMesh()
{
    lockMesh();
    mesh_ = MyMesh();
    releaseMesh();
}

Vector3d Mesh::computeCentroid()
{
    lockMesh();
    Vector3d result;
    result.setZero();
    for(MyMesh::VertexIter vi = mesh_.vertices_begin(); vi != mesh_.vertices_end(); ++vi)
    {
        for(int i=0; i<3; i++)
            result[i] += mesh_.point(vi)[i];
    }
    result /= mesh_.n_vertices();
    releaseMesh();
    return result;
}

double Mesh::computeBoundingSphere(const Eigen::Vector3d &center)
{
    lockMesh();
    double radius = 0;
    for(MyMesh::VertexIter vi = mesh_.vertices_begin(); vi != mesh_.vertices_end(); ++vi)
    {
        double thisr = 0;
        for(int i=0; i<3; i++)
            thisr += (mesh_.point(vi)[i]-center[i])*(mesh_.point(vi)[i]-center[i]);
        if(radius < thisr)
            radius = thisr;
    }
    releaseMesh();
    return sqrt(radius);
}

double Mesh::computeBoundingCircle(const Eigen::Vector3d &center)
{
    lockMesh();
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
    releaseMesh();
    return sqrt(radius);
}

MyMesh::Point Mesh::computeEdgeMidpoint(MyMesh::EdgeHandle edge)
{
    lockMesh();
    MyMesh::HalfedgeHandle heh = mesh_.halfedge_handle(edge, 0);
    if(!heh.is_valid())
        heh = mesh_.halfedge_handle(edge, 1);
    assert(heh.is_valid());

    MyMesh::Point result = mesh_.point(mesh_.from_vertex_handle(heh));
    result += mesh_.point(mesh_.to_vertex_handle(heh));
    result *= 0.5;
    releaseMesh();
    return result;
}

void Mesh::edgeEndpoints(MyMesh::EdgeHandle edge, MyMesh::Point &p1, MyMesh::Point &p2)
{
    lockMesh();
    MyMesh::HalfedgeHandle heh = mesh_.halfedge_handle(edge,0);
    if(!heh.is_valid())
        heh = mesh_.halfedge_handle(edge,1);
    assert(heh.is_valid());

    p1 = mesh_.point(mesh_.from_vertex_handle(heh));
    p2 = mesh_.point(mesh_.to_vertex_handle(heh));
    releaseMesh();
}

double Mesh::randomDouble()
{
    return ((double)rand()/(double)RAND_MAX);
}

void Mesh::translateVertex(int vidx, const Eigen::Vector3d &translation)
{
    lockMesh();
    assert(vidx >= 0 && vidx < (int)mesh_.n_vertices());

    MyMesh::VertexHandle vh = mesh_.vertex_handle(vidx);
    MyMesh::Point &pt = mesh_.point(vh);
    for(int i=0; i<3; i++)
        pt[i] += translation[i];
    releaseMesh();
}

void Mesh::translateFace(int fidx, const Vector3d &translation)
{
    lockMesh();
    assert(fidx >= 0 && fidx < (int)mesh_.n_faces());

    MyMesh::FaceHandle fh = mesh_.face_handle(fidx);
    for(MyMesh::FaceVertexIter fvi = mesh_.fv_iter(fh); fvi; ++fvi)
    {
        MyMesh::Point &pt = mesh_.point(fvi);
        pt[0] += translation[0];
        pt[1] += translation[1];
        pt[2] += translation[2];
    }
    releaseMesh();
}

double Mesh::faceAreaOnPlane(MyMesh::FaceHandle face)
{
    lockMesh();
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
    releaseMesh();
    return fabs(area);
}

int Mesh::numFaceVerts(MyMesh::FaceHandle face)
{
    lockMesh();
    int result = 0;
    for(MyMesh::FaceVertexIter fvi = mesh_.fv_iter(face); fvi; ++fvi)
    {
        result++;
    }
    releaseMesh();
    return result;
}

double Mesh::vertexAreaOnPlane(MyMesh::VertexHandle vert)
{
    lockMesh();
    double ans = 0;
    for(MyMesh::VertexFaceIter it = mesh_.vf_iter(vert); it; ++it)
    {
        int verts = numFaceVerts(it);
        double facearea = faceAreaOnPlane(it);
        ans += facearea/verts;
    }
    releaseMesh();
    return ans;
}

void Mesh::setPlaneAreaLoads()
{
    lockMesh();
    for(MyMesh::VertexIter vi = mesh_.vertices_begin(); vi != mesh_.vertices_end(); ++vi)
    {
        double area = vertexAreaOnPlane(vi);
        mesh_.data(vi).set_load(area);
    }
    releaseMesh();
}

void Mesh::getNRing(int vidx, int n, set<int> &nring)
{
    lockMesh();
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
    releaseMesh();
}

Vector3d Mesh::projectToFace(MyMesh::FaceHandle fh, const Vector3d &p)
{
    lockMesh();
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
    releaseMesh();
    return result;
}

Vector3d Mesh::approximateClosestPoint(const Vector3d &p)
{
    lockMesh();
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
        releaseMesh();
        return newpos;
    }
    releaseMesh();
    return p;
}
