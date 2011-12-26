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
    renderSurface3D(RF_FACES | RF_VERTICES | RF_WIREFRAME | RF_PICKING);
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
                glNormal3d(n[0], -fabs(n[1]), n[2]);
                glVertex3d(pt[0],pt[1],pt[2]);

            }
            glEnd();
        }

        glDisable(GL_POLYGON_OFFSET_FILL);
    }
    if(renderFlags & RF_WIREFRAME)
    {
        //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        if(renderFlags & RF_PICKING)
            glLineWidth(4.0);
        else
            glLineWidth(1.0);
        glBegin(GL_LINES);
        for(MyMesh::ConstEdgeIter ei = mesh_.edges_begin(); ei != mesh_.edges_end(); ++ei)
        {
            if(renderFlags & RF_PICKING)
            {
                float r,g,b;
                encodeAsColor(ei.handle().idx(), Mesh::PT_EDGE, r, g, b);;
                glColor3f(r,g,b);
            }
            else if(m_.edgePinned(ei.handle()))
                glColor4f(1.0, 0.0, 0.0, 0.8);
            else
                glColor4f(0.0, 0.0, 0.0, 0.8);
            MyMesh::Point pt1, pt2;
            m_.edgeEndpoints(ei.handle(), pt1, pt2);
            glVertex3d(pt1[0], pt1[1], pt1[2]);
            glVertex3d(pt2[0], pt2[1], pt2[2]);
        }
        glEnd();
        /*MyMesh::ConstFaceIter f, fEnd = mesh_.faces_end();
        int i=0;
        for (f = mesh_.faces_begin(); f != fEnd; ++f,i++) {
            glColor4f(0.0, 0.0, 0.0, 0.8);
            glBegin(GL_POLYGON);
            for (MyMesh::ConstFaceVertexIter v = mesh_.cfv_iter(f); v; ++v) {
                MyMesh::Point pt = mesh_.point(v);
                glVertex3d(pt[0],pt[1],pt[2]);

            }
            glEnd();
        }*/
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
        else if(mesh_.data(v).handled() && mesh_.data(v).pinned())
        {
            glColor3f(1.0, 0.4, 0.0);
        }
        else if(mesh_.data(v).anchored())
        {
            glColor3f(0.7, 0.7, 0.0);
        }
        else if(mesh_.data(v).handled())
        {
            glColor3f(0, .4, 0);
        }
        else if(mesh_.data(v).pinned())
        {
            glColor3f(1.0, 0.0, 0.0);
        }
        else if(renderFlags & RF_VERTICES)
        {
            glColor3f(0,0,0);
        }
        else
            continue;
        MyMesh::Point pt = mesh_.point(v);
        double radius = std::numeric_limits<double>::infinity();
        for(MyMesh::ConstVertexEdgeIter vei = mesh_.cve_iter(v); vei; ++vei)
        {
            double len = 0.3*mesh_.calc_edge_length(vei.handle());
            if(len < radius)
                radius = len;
        }
        glPushMatrix();
        glTranslatef(pt[0],pt[1],pt[2]);
        gluSphere(sphere, radius, 10, 10);
        glPopMatrix();
    }
    gluDeleteQuadric(sphere);
}
