#ifndef NETWORKMESHRENDERER_H
#define NETWORKMESHRENDERER_H

#include "meshrenderer.h"

class NetworkMeshRenderer : public MeshRenderer
{
public:
    NetworkMeshRenderer(Mesh &m);

    void render3D(double time, double modeAmp);
    void render2D(double time, double modeAmp);
    void renderSurface(double time, double modeAmp);
    void renderConjugateVectors3D();
    void renderConjugateVectors2D();
};

#endif // NETWORKMESHRENDERER_H
