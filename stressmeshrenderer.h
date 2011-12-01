#ifndef STRESSMESHRENDERER_H
#define STRESSMESHRENDERER_H

#include "meshrenderer.h"

class StressMeshRenderer : public MeshRenderer
{
public:
    StressMeshRenderer(Mesh &m);

    void render3D();
    void render2D();
};
#endif // STRESSMESHRENDERER_H
