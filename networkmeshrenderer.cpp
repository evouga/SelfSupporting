#include "networkmeshrenderer.h"
#include "networkmesh.h"

using namespace std;
using namespace Eigen;

const double PI = 3.1415926535898;

NetworkMeshRenderer::NetworkMeshRenderer(Mesh &m) : MeshRenderer(m)
{
}

void NetworkMeshRenderer::render3D(double time, int mode, double modeAmp)
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
        else
            glLineWidth(1.0);

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
        ((NetworkMesh &)m_).edgeEndpointsWithMode(e, p1, p2, mode, modeAmp*sin(time));
        glVertex3f(p1[0],p1[1],p1[2]);
        glVertex3f(p2[0],p2[1],p2[2]);
        glEnd();
    }

    for(MyMesh::ConstVertexIter v = mesh_.vertices_begin(); v != mesh_.vertices_end(); ++v)
    {
        if(mesh_.data(v.handle()).outofenvelope())
        {
            glColor4f(0,0,0,1.0);
        }
        else
            continue;

        drawSphere(v.handle().idx());
    }
}

void NetworkMeshRenderer::renderSurface(double time, int mode, double modeAmp)
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

        glBegin(GL_POLYGON);
        for (MyMesh::ConstFaceVertexIter v = mesh_.cfv_iter(f); v; ++v) {




            MyMesh::Point pt;
            ((NetworkMesh &)m_).pointWithMode(v.handle(), pt, mode, modeAmp*sin(time));

            double modeMag = ((NetworkMesh &)m_).pointModeValue(v.handle(), mode);
            Vector3d color = colormap(modeMag, -1, 1);

            glColor4f(color[0], color[1], color[2], 1.0);

            //glColor4f(156/255., 213/255., 86/255., 1);


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


void NetworkMeshRenderer::render2D(double , int, double)
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

Vector3d NetworkMeshRenderer::colormap(double val, double min, double max) const
{
    double mapped = (val-min)/(max-min);
    return colormap(mapped);
}

Vector3d NetworkMeshRenderer::colormap(double val) const
{
    Vector3d hsl;
    hsl[0] = (1.0-val)*4.0*PI/3.0;
    hsl[1] = 1.0;
    hsl[2] = 0.5*val;
    return HSLtoRGB(hsl);
}

Vector3d NetworkMeshRenderer::HSLtoRGB(const Vector3d &hsl) const
{
    double chroma = (1.0 - fabs(2.0*hsl[2]-1.0))*hsl[1];
    double Hprime = hsl[0]*6.0/(2.0*PI);
    double hmod = fmod(Hprime, 2.0);
    double x = chroma*(1.0 - fabs(hmod-1.0));

    Vector3d rgb;

    if(Hprime < 1.0)
    {
        rgb[0] = chroma;
        rgb[1] = x;
        rgb[2] = 0;
    }
    else if(Hprime < 2.0)
    {
        rgb[0] = x;
        rgb[1] = chroma;
        rgb[2] = 0;
    }
    else if(Hprime < 3.0)
    {
        rgb[0] = 0;
        rgb[1] = chroma;
        rgb[2] = x;
    }
    else if(Hprime < 4.0)
    {
        rgb[0] = 0;
        rgb[1] = x;
        rgb[2] = chroma;
    }
    else if(Hprime < 5.0)
    {
        rgb[0] = x;
        rgb[1] = 0;
        rgb[2] = chroma;
    }
    else
    {
        rgb[0] = chroma;
        rgb[1] = 0;
        rgb[2] = x;
    }

    double m = hsl[2]-0.5*chroma;
    for(int i=0; i<3; i++)
        rgb[i] += m;
    return rgb;
}
