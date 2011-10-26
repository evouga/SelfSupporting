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

void NetworkMeshRenderer::render2D()
{

}
