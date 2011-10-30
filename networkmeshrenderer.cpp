#include "networkmeshrenderer.h"
#include "networkmesh.h"

NetworkMeshRenderer::NetworkMeshRenderer(Mesh &m) : MeshRenderer(m)
{

}

void NetworkMeshRenderer::render3D()
{
    const MyMesh &mesh_ = m_.getMesh();
    glDisable(GL_LIGHTING);
    glEnable(GL_DITHER);

    glLineWidth(2.0);
    glBegin(GL_LINES);
    for(MyMesh::ConstEdgeIter e = mesh_.edges_begin(); e != mesh_.edges_end(); ++e)
    {
        if(mesh_.data(e).weight() < 1e-6)
            glColor4f(1.0,0.0,0.0,1.0);
        else
            glColor4f(0.4, 0.1, 0.0, 1.0);
        MyMesh::Point p1, p2;
        m_.edgeEndpoints(e, p1, p2);
        glVertex3f(p1[0],p1[1],p1[2]);
        glVertex3f(p2[0],p2[1],p2[2]);
    }
    glEnd();
    m_.releaseMesh();
}

void NetworkMeshRenderer::renderSurface()
{
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
            glColor4f(156/255., 213/255., 86/255., 1);
\
            MyMesh::Point pt = mesh_.point(v);
            MyMesh::Point n;
            mesh_.calc_vertex_normal_correct(v, n);
            n.normalize();
            glNormal3d(n[0], n[1], n[2]);
            glVertex3d(pt[0],pt[1],pt[2]);

        }
        glEnd();
    }
    glDisable(GL_POLYGON_OFFSET_FILL);

    m_.releaseMesh();
}

void NetworkMeshRenderer::render2D()
{

}
