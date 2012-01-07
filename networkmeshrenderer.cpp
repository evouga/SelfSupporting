#include "networkmeshrenderer.h"
#include "networkmesh.h"

using namespace std;
using namespace Eigen;

NetworkMeshRenderer::NetworkMeshRenderer(Mesh &m) : MeshRenderer(m)
{

}

void NetworkMeshRenderer::render3D()
{
    auto_ptr<MeshLock> ml = m_.acquireMesh();
    const MyMesh &mesh_ = m_.getMesh();
    glDisable(GL_LIGHTING);
    glEnable(GL_DITHER);

    double maxweight = 0;
    for(MyMesh::ConstEdgeIter e = mesh_.edges_begin(); e != mesh_.edges_end(); ++e)
        maxweight = std::max(maxweight, mesh_.data(e).weight());
    if(maxweight < 1e-8)
    {
        return;
    }

    for(MyMesh::ConstEdgeIter e = mesh_.edges_begin(); e != mesh_.edges_end(); ++e)
    {
        double weight = mesh_.data(e).weight();
        weight *= 4.0/maxweight;
        if(weight != 0)
        {
            glLineWidth(weight);
        }

        glBegin(GL_LINES);
        if(m_.edgePinned(e.handle()))
            glColor4f(1.0, 0.0, 0.0, 1.0);
        else if(mesh_.data(e).weight() < -1e-6)
            glColor4f(0.0, 1.0, 1.0, 1.0);
        else if(mesh_.data(e).weight() < 1e-6)
            glColor4f(1.0, 0.0, 1.0, 1.0);
        else
            glColor4f(0.4, 0.1, 0.0, 1.0);
        MyMesh::Point p1, p2;
        m_.edgeEndpoints(e, p1, p2);
        glVertex3f(p1[0],p1[1],p1[2]);
        glVertex3f(p2[0],p2[1],p2[2]);
        glEnd();
    }
}

void NetworkMeshRenderer::renderSurface()
{
    auto_ptr<MeshLock> ml = m_.acquireMesh();
    const MyMesh &mesh_ = m_.getMesh();

    glEnable(GL_LIGHTING);
    glEnable(GL_DITHER);

    glPolygonOffset(1.0, 1.0);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    MyMesh::ConstFaceIter f, fEnd = mesh_.faces_end();
    int i=0;
    for (f = mesh_.faces_begin(); f != fEnd; ++f,i++) {

        //double umbilic = 1-2.0*mesh_.data(f).umbilic();
        glBegin(GL_POLYGON);
        for (MyMesh::ConstFaceVertexIter v = mesh_.cfv_iter(f); v; ++v) {
            if(((NetworkMesh &)m_).isBadVertex(v.handle()))
                glColor4f(1.0,0.0,0.0,1.0);
            else
                glColor4f(156/255., 213/255., 86/255., 1);

            double viol = 1+log(mesh_.data(v.handle()).violation())/24.0;
            glColor4f(viol, 213/255., 86/255., 1);

            if(mesh_.data(v.handle()).handled())
                glColor4f(1.0,0.0,0.0,1.0);
            //glColor4f(umbilic, 0, 0, 1.0);



            MyMesh::Point pt = mesh_.point(v);
            MyMesh::Point n;
            mesh_.calc_vertex_normal_correct(v, n);
            n.normalize();
            glNormal3d(n[0], -fabs(n[1]), n[2]);
            glVertex3d(pt[0],pt[1],pt[2]);

        }
        glEnd();
    }
    glDisable(GL_POLYGON_OFFSET_FILL);

}

void NetworkMeshRenderer::renderVertConjugateVectors3D()
{
    auto_ptr<MeshLock> ml = m_.acquireMesh();
    //glEnable(GL_LINE_SMOOTH);
    glDisable(GL_LIGHTING);
    const MyMesh &mesh = m_.getMesh();

    for(MyMesh::ConstVertexIter vi = mesh.vertices_begin(); vi != mesh.vertices_end(); ++vi)
    {
        MyMesh::Point pt = mesh.point(vi);
        Vector2d u = mesh.data(vi.handle()).rel_principal_u();
        Vector2d v = mesh.data(vi.handle()).rel_principal_v();
        double minlen = std::numeric_limits<double>::infinity();
        for(MyMesh::ConstVertexEdgeIter ve = mesh.cve_iter(vi); ve; ++ve)
        {
            double len = mesh.calc_edge_length(ve.handle());
            if(len < minlen)
                minlen = len;
        }

        double radius = 0.4*minlen;

        glLineWidth(0.1);

        glBegin(GL_LINES);
        glColor3f(1.0, 0.0, 0.0);
        glVertex3d(pt[0]-0.5*radius*u[0], pt[1], pt[2]-0.5*radius*u[1]);
        glVertex3d(pt[0]+radius*u[0], pt[1], pt[2]+radius*u[1]);
        glColor3f(0.0, 0.5, 0.0);
        glVertex3d(pt[0]-0.5*radius*v[0], pt[1], pt[2]-0.5*radius*v[1]);
        glVertex3d(pt[0]+radius*v[0], pt[1], pt[2]+radius*v[1]);
        glEnd();
    }

}


void NetworkMeshRenderer::renderConjugateVectors3D()
{
    auto_ptr<MeshLock> ml = m_.acquireMesh();
    //glEnable(GL_LINE_SMOOTH);
    const MyMesh &mesh = m_.getMesh();

    for(MyMesh::ConstFaceIter f = mesh.faces_begin(); f != mesh.faces_end(); ++f)
    {
        const double PI = 3.1415926535898;
        MyMesh::Point pt;
        mesh.calc_face_centroid(f.handle(), pt);
        Vector3d u = mesh.data(f.handle()).rel_principal_u();
        Vector3d v = mesh.data(f.handle()).rel_principal_v();
        double area = m_.faceAreaOnPlane(f.handle());
        double radius = 0.8*sqrt(area/PI);
        double radiusu = radius / sqrt(u[0]*u[0]+u[2]*u[2]);
        double radiusv = radius / sqrt(v[0]*v[0]+v[2]*v[2]);
        glLineWidth(0.1);

        glBegin(GL_LINES);
        glColor3f(1.0, 0.0, 0.0);
        glVertex3d(pt[0]-0.5*radiusu*u[0], pt[1] - 0.5*radiusu*u[1], pt[2]-0.5*radiusu*u[2]);
        glVertex3d(pt[0]+radiusu*u[0], pt[1] + radiusu*u[1], pt[2]+radiusu*u[2]);
        glColor3f(0.0, 0.5, 0.0);
        glVertex3d(pt[0]-0.5*radiusv*v[0], pt[1] - 0.5*radiusv*v[1], pt[2]-0.5*radiusv*v[2]);
        glVertex3d(pt[0]+radiusv*v[0], pt[1] + radiusv*v[1], pt[2]+radiusv*v[2]);
        glEnd();
    }

}


void NetworkMeshRenderer::render2D()
{
    auto_ptr<MeshLock> ml = m_.acquireMesh();
    glEnable(GL_LINE_SMOOTH);
    const MyMesh &mesh = m_.getMesh();
    glColor4f(0.4, 0.1, 0.0, 1.0);
    double maxweight = 0;
    for(MyMesh::ConstEdgeIter e = mesh.edges_begin(); e != mesh.edges_end(); ++e)
        maxweight = std::max(maxweight, mesh.data(e).weight());
    if(maxweight < 1e-8)
    {
        return;
    }
    for(MyMesh::ConstEdgeIter e = mesh.edges_begin(); e != mesh.edges_end(); ++e)
    {
        MyMesh::HalfedgeHandle heh = mesh.halfedge_handle(e,0);
        if(!heh.is_valid())
            heh = mesh.halfedge_handle(e,1);
        assert(heh.is_valid());
        MyMesh::Point pt1 = mesh.point(mesh.to_vertex_handle(heh));
        MyMesh::Point pt2 = mesh.point(mesh.from_vertex_handle(heh));

        double weight = mesh.data(e).weight();
        weight *= 4.0/maxweight;
        if(weight != 0)
        {
            glLineWidth(weight);

            glBegin(GL_LINES);
            glVertex2d(pt1[0], pt1[2]);
            glVertex2d(pt2[0], pt2[2]);
            glEnd();
        }
    }
}

void NetworkMeshRenderer::renderConjugateVectors2D()
{
    auto_ptr<MeshLock> ml = m_.acquireMesh();
    glEnable(GL_LINE_SMOOTH);
    const MyMesh &mesh = m_.getMesh();

    for(MyMesh::ConstFaceIter f = mesh.faces_begin(); f != mesh.faces_end(); ++f)
    {
        const double PI = 3.1415926535898;
        MyMesh::Point pt;
        mesh.calc_face_centroid(f.handle(), pt);
        Vector3d u = mesh.data(f.handle()).rel_principal_u();
        Vector3d v = mesh.data(f.handle()).rel_principal_v();
        double area = m_.faceAreaOnPlane(f.handle());
        double radius = 0.8*sqrt(area/PI);
        glLineWidth(0.1);

        glBegin(GL_LINES);
        glColor3f(1.0, 0.0, 0.0);
        glVertex2d(pt[0]-0.5*radius*u[0], pt[2]-0.5*radius*u[2]);
        glVertex2d(pt[0]+radius*u[0], pt[2]+radius*u[2]);
        glColor3f(0.0, 0.5, 0.0);
        glVertex2d(pt[0]-0.5*radius*v[0], pt[2]-0.5*radius*v[2]);
        glVertex2d(pt[0]+radius*v[0], pt[2]+radius*v[2]);
        glEnd();
    }
}
