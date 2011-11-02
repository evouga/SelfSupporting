#include "referencemeshrenderer.h"

using namespace std;

ReferenceMeshRenderer::ReferenceMeshRenderer(Mesh &m) : MeshRenderer(m)
{
}

void ReferenceMeshRenderer::render2D()
{
    auto_ptr<MeshLock> ml = m_.acquireMesh();
    const MyMesh &mesh_ = m_.getMesh();
    glColor3d(0,0,1);
    glLineWidth(1.0);
    glBegin(GL_LINES);
    for(MyMesh::ConstEdgeIter ei = mesh_.edges_begin(); ei != mesh_.edges_end(); ++ei)
    {
        MyMesh::HalfedgeHandle heh = mesh_.halfedge_handle(ei, 0);
        MyMesh::Point pt1 = mesh_.point(mesh_.to_vertex_handle(heh));
        MyMesh::Point pt2 = mesh_.point(mesh_.from_vertex_handle(heh));
        glVertex2d(pt1[0], pt1[2]);
        glVertex2d(pt2[0], pt2[2]);
    }
    glEnd();
}

void ReferenceMeshRenderer::render3D()
{
    renderSurface3D(RF_FACES | RF_WIREFRAME);
}

void ReferenceMeshRenderer::renderPick3D()
{
    renderSurface3D(RF_FACES | RF_VERTICES | RF_PICKING);
}

void ReferenceMeshRenderer::renderSurface3D(int renderFlags)
{
    auto_ptr<MeshLock> ml = m_.acquireMesh();
    const MyMesh &mesh_ = m_.getMesh();
    if(renderFlags & RF_PICKING)
    {
        glDisable(GL_LIGHTING);
        glDisable(GL_DITHER);
    }
    else
    {
        glEnable(GL_LIGHTING);
        glEnable(GL_DITHER);
    }

    if(renderFlags & RF_FACES)
    {
        glPolygonOffset(1.0, 1.0);
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        MyMesh::ConstFaceIter f, fEnd = mesh_.faces_end();
        int i=0;
        for (f = mesh_.faces_begin(); f != fEnd; ++f,i++) {

            glBegin(GL_POLYGON);
            for (MyMesh::ConstFaceVertexIter v = mesh_.cfv_iter(f); v; ++v) {
                if(renderFlags & RF_PICKING)
                {
                    float r,g,b;
                    encodeAsColor(i, Mesh::PT_FACE, r, g, b);;
                    glColor3f(r,g,b);
                }
                else
                {
                    glColor4f(156/255., 186/255., 214/255.,.8);
                }

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
    }
    if(renderFlags & RF_WIREFRAME)
    {

        assert(!(renderFlags & RF_PICKING)); //TODO
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glLineWidth(1.0);
        MyMesh::ConstFaceIter f, fEnd = mesh_.faces_end();
        int i=0;
        for (f = mesh_.faces_begin(); f != fEnd; ++f,i++) {
            glColor4f(0.0, 0.0, 0.0, 0.8);
            glBegin(GL_POLYGON);
            for (MyMesh::ConstFaceVertexIter v = mesh_.cfv_iter(f); v; ++v) {
                MyMesh::Point pt = mesh_.point(v);
                glVertex3d(pt[0],pt[1],pt[2]);

            }
            glEnd();
        }
    }
    GLUquadricObj *sphere = gluNewQuadric();
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    int i=0;
    for(MyMesh::ConstVertexIter v = mesh_.vertices_begin(); v != mesh_.vertices_end(); ++v, i++)
    {
        if(renderFlags & RF_PICKING)
        {
            if(renderFlags & RF_VERTICES)
            {
                float r, g, b;
                encodeAsColor(i, Mesh::PT_VERTEX, r, g, b);
                glColor3f(r,g,b);
            }
            else
                continue;
        }
        else if(mesh_.data(v).anchored())
        {
            glColor3f(0, .4, 0);
        }
        else if(renderFlags & RF_VERTICES)
        {
            glColor3f(0,0,0);
        }
        else
            continue;
        MyMesh::Point pt = mesh_.point(v);
        glPushMatrix();
        glTranslatef(pt[0],pt[1],pt[2]);
        gluSphere(sphere, 0.04, 10, 10);
        glPopMatrix();
    }
    gluDeleteQuadric(sphere);
}
