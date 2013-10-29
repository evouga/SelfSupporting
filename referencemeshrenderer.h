#ifndef REFERENCEMESHRENDERER_H
#define REFERENCEMESHRENDERER_H

#include "meshrenderer.h"

class ReferenceMeshRenderer : public MeshRenderer
{
public:
    ReferenceMeshRenderer(Mesh &m);

    void renderPick3D();
    void render3D(double time, int mode, double modeAmp);
    void render2D(double time, int mode, double modeAmp);

private:

    enum RenderFlags3D {RF_NONE=0, RF_PICKING=1, RF_FACES=2, RF_WIREFRAME=4, RF_VERTICES=8};

    void renderSurface3D(int renderFlags);

};

#endif // REFERENCEMESHRENDERER_H
