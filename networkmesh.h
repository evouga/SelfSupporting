#ifndef NETWORKMESH_H
#define NETWORKMESH_H

#include "mesh.h"

class ReferenceMesh;
class NetworkMeshRenderer;

class NetworkMesh : public Mesh
{
public:
    NetworkMesh(Controller &cont);
    ~NetworkMesh();

    MeshRenderer &getRenderer();

    void copyFromReferenceMesh(const ReferenceMesh &rm);

    double computeWeightsOnPlane(const ReferenceMesh &rm, double sum);
    double updateHeights();

    // Given a (possibly non-self-supporting) 3D mesh, finds non-negative weights that come as close as possible to
    // satisfying the force-equilibrium constraints
    double computeBestWeights();

    // Given non-negative weights, finds the closest mesh to the given mesh that is self-supporting with those weights
    double computeBestPositionsTangentLS(const ReferenceMesh &rm, double alpha, double beta);

    void projectOntoReference(const ReferenceMesh &rm);


    void subdivide();
    bool isBadVertex(MyMesh::VertexHandle vert);

    double calculateEquilibriumViolation();


private:
    void fixBadVertices();

    NetworkMeshRenderer *renderer_;

};

#endif // NETWORKMESH_H
