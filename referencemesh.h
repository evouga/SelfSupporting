#ifndef REFERENCEMESH_H
#define REFERENCEMESH_H

#include "mesh.h"

class ReferenceMeshRenderer;
class Controller;
class NetworkMesh;
class Camera;

class ReferenceMesh : public Mesh
{
public:
    ReferenceMesh(Controller &cont);
    ~ReferenceMesh();

    MeshRenderer &getRenderer();

    void copyFromNetworkMesh(NetworkMesh &nm);
    bool addOBJ(const char *filename);

    void computeClosestPointOnPlane(const Eigen::Vector2d &pos, int &closestidx, double &closestdist);
    void jitterOnPlane();
    void jitter();

    void buildQuadMesh(int w, int h);
    void buildTriMesh(int w, int h);
    void buildHexMesh(int w, int h);

    double computeBestDWeights(const Eigen::VectorXd &dQ, Eigen::VectorXd &dW);

    void applyLaplacianDeformation(int vidx, const Eigen::Vector3d &delta);
    void applyLaplacianDeformationHeight(int vidx, const Eigen::Vector3d &delta, int radius);
    void applyLaplacianDeformationTop(int vidx, const Eigen::Vector3d &delta, int radius, bool excludePinned);

    void setHandle(int vidx, bool state);
    void deleteFace(int fidx);

    void setPin(int vidx, bool state);
    void setAnchor(int vidx, bool state);

    void pinBoundary();
    void unpinBoundary();
    void swapYandZ();
    void invertY();

    void averageHandledHeights();
    void trimBoundary();
    void selectPinned();

    std::vector<int> selectRectangle(const Eigen::Vector2d &c1, const Eigen::Vector2d &c2, Camera &c);

private:

    ReferenceMeshRenderer *renderer_;
};

#endif // REFERENCEMESH_H
