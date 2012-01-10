#ifndef MESHRENDERER_H
#define MESHRENDERER_H

#include "mesh.h"
#include <QtOpenGL>

class MeshRenderer
{
public:
    MeshRenderer(Mesh &m);
    ~MeshRenderer();

    virtual void render3D()=0;
    virtual void render2D()=0;
    static void encodeAsColor(unsigned int n, Mesh::PrimType type, float &r, float &g, float &b);
    static void decodeFromColor(unsigned int &n, Mesh::PrimType &type, float r, float g, float b);
    static void decodeFromColor(unsigned int &n, Mesh::PrimType &type, GLubyte *pixelbuf);

    virtual void drawSphere(int vertex);

protected:
    Mesh &m_;
    GLUquadric *quadric_;
};

#endif // MESHRENDERER_H
