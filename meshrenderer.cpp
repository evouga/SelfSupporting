#include "meshrenderer.h"
#include "mesh.h"
#include <QtOpenGL>

MeshRenderer::MeshRenderer(Mesh &m) : m_(m)
{
}


void MeshRenderer::encodeAsColor(unsigned int n, Mesh::PrimType type, float &r, float &g, float &b)
{
    unsigned int max = (1<<22);
    int typem;
    assert(n<max);
    switch(type)
    {
        case Mesh::PT_NONE:
            typem = 0;
            break;
        case Mesh::PT_VERTEX:
            typem = 1;
            break;
        case Mesh::PT_EDGE:
            typem = 2;
            break;
        case Mesh::PT_FACE:
            typem = 3;
            break;
        default:
            assert(!"Bad primitive type");
    }
    unsigned int toencode = (typem <<22) + n;
    r = (toencode % 256)/255.0;
    toencode >>= 8;
    g = (toencode % 256)/255.0;
    toencode >>= 8;
    b = (toencode % 256)/255.0;
}

void MeshRenderer::decodeFromColor(unsigned int &n, Mesh::PrimType &type, float r, float g, float b)
{
    GLubyte pixels[3];
    pixels[0] = 255.0 * r;
    pixels[1] = 255.0 * g;
    pixels[2] = 255.0 * b;
    decodeFromColor(n, type, pixels);
}

void MeshRenderer::decodeFromColor(unsigned int &n, Mesh::PrimType &type, GLubyte *pixelbuf)
{
    unsigned int decoded = (pixelbuf[2]<<16) + (pixelbuf[1]<<8) + pixelbuf[0];
    int typei = (decoded >> 22);
    switch(typei)
    {
        case 0:
            type = Mesh::PT_NONE;
            break;
        case 1:
            type = Mesh::PT_VERTEX;
            break;
        case 2:
            type = Mesh::PT_EDGE;
            break;
        case 3:
            type = Mesh::PT_FACE;
            break;
        default:
            assert(!"Bad primitive type");
    }
    n = decoded & ( (1<<22)-1 );
}
