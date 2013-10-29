#ifndef NETWORKMESHRENDERER_H
#define NETWORKMESHRENDERER_H

#include "meshrenderer.h"

class NetworkMeshRenderer : public MeshRenderer
{
public:
    NetworkMeshRenderer(Mesh &m);

    void render3D(double time, int mode, double modeAmp);
    void render2D(double time, int mode, double modeAmp);
    void renderSurface(double time, int mode, double modeAmp);
    void renderConjugateVectors3D();
    void renderConjugateVectors2D();

private:
    Eigen::Vector3d colormap(double val, double min, double max) const;
    Eigen::Vector3d colormap(double val) const;
    Eigen::Vector3d HSLtoRGB(const Eigen::Vector3d &hsl) const;
};

#endif // NETWORKMESHRENDERER_H
