#ifndef STRESSMESHRENDERER_H
#define STRESSMESHRENDERER_H

#include "meshrenderer.h"

class StressMeshRenderer : public MeshRenderer
{
public:
    StressMeshRenderer(Mesh &m);

    void render3D(double time, int mode, double modeAmp);
    void render2D(double time, int mode, double modeAmp);
};
#endif // STRESSMESHRENDERER_H
