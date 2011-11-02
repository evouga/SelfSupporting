#ifndef REFERENCEMESH_H
#define REFERENCEMESH_H

#include "mesh.h"

class ReferenceMeshRenderer;
class Controller;
class NetworkMesh;

class ReferenceMesh : public Mesh
{
public:
    ReferenceMesh(Controller &cont);
    ~ReferenceMesh();

    MeshRenderer &getRenderer();

    void copyFromNetworkMesh(NetworkMesh &nm);

    void computeClosestPointOnPlane(const Eigen::Vector2d &pos, int &closestidx, double &closestdist);
    void jitterOnPlane();
    void jitter();

    bool loadMesh(const char *name);
    bool saveMesh(const char *name);
    void buildQuadMesh(int w, int h);
    void buildTriMesh(int w, int h);
    void buildHexMesh(int w, int h);

    void applyLaplacianDeformation(int vidx, const Eigen::Vector3d &delta);
    void applyLaplacianDeformationHeight(int vidx, const Eigen::Vector3d &delta);

    void setAnchor(int vidx, bool state);
    void deleteFace(int fidx);

private:
    ReferenceMeshRenderer *renderer_;
};

#endif // REFERENCEMESH_H
