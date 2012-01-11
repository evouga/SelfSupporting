#ifndef NETWORKMESH_H
#define NETWORKMESH_H

#include "mesh.h"
#include <Eigen/Sparse>

class ReferenceMesh;
class NetworkMeshRenderer;

class NetworkMesh : public Mesh
{
public:
    NetworkMesh(Controller &cont);
    ~NetworkMesh();

    MeshRenderer &getRenderer();

    void copyFromReferenceMesh(ReferenceMesh &rm);
    void saveSubdivisionReference();

    double computeWeightsOnPlane(ReferenceMesh &rm, double sum);
    double updateHeights();

    // Given a (possibly non-self-supporting) 3D mesh, finds non-negative weights that come as close as possible to
    // satisfying the force-equilibrium constraints
    double computeBestWeights(double maxstress, double thickness, double tol);


    // Given non-negative weights, finds the closest mesh to the given mesh that is self-supporting with those weights
    double computeBestPositionsTangentLS(double alpha, double beta, double thickness, bool planarity);

    double enforcePlanarity();

    void projectOntoReference(ReferenceMesh &rm);
    void projectOnto(MyMesh &m);

    bool isBadVertex(MyMesh::VertexHandle vert);

    double calculateEquilibriumViolation();
    double calculateEquilibriumViolation(MyMesh::VertexHandle vh);
    void computeRelativePrincipalDirections();

    void setupVFProperties();
    void exportVectorFields(const char *name);
    bool exportReciprocalMesh(const char *name);
    bool exportWeights(const char *name);


private:
    Eigen::Matrix2d approximateStressHessian(MyMesh::FaceHandle face);
    void fixBadVertices();
    bool fixBadVerticesNew();

    double computeLaplacianWeight(MyMesh::EdgeHandle edge);
    double computeLaplacianAlpha(MyMesh::HalfedgeHandle heh);
    void triangleSubdivide();
    void addToStrippedMatrix(Eigen::DynamicSparseMatrix<double> &M, Eigen::VectorXd &rhs, int v, int k, int j, double val, std::map<int, int> &vidx2midx);
    double planarityViolation();
    void distanceFromReference(MyMesh &rm, double thickness);
    void computeSubdReferenceCentroids();

    MyMesh subdreference_;

    NetworkMeshRenderer *renderer_;

};

#endif // NETWORKMESH_H
