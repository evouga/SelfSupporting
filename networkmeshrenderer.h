#ifndef NETWORKMESHRENDERER_H
#define NETWORKMESHRENDERER_H

#include "meshrenderer.h"

class NetworkMeshRenderer : public MeshRenderer
{
public:
    NetworkMeshRenderer(Mesh &m);

    void render3D();
    void render2D();
    void renderSurface();
    void renderConjugateVectors3D();
    void renderConjugateVectors2D();
};

#endif // NETWORKMESHRENDERER_H
