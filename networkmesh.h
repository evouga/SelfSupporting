#ifndef NETWORKMESH_H
#define NETWORKMESH_H

#include "mesh.h"
#include <Eigen/Sparse>
#include "IpTNLP.hpp"

class ReferenceMesh;
class NetworkMeshRenderer;
class NetworkMesh;

class NMOPT : public Ipopt::TNLP
{
public:
    NMOPT(NetworkMesh &nm) : nm_(nm) {}

    virtual bool get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
                              Ipopt::Index& nnz_h_lag, IndexStyleEnum& index_style);

    virtual bool get_bounds_info(Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u,
                                 Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u);

    virtual bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number* x,
                                    bool init_z, Ipopt::Number* z_L, Ipopt::Number* z_U,
                                    Ipopt::Index m, bool init_lambda, Ipopt::Number* lambda);

    virtual bool eval_f(Ipopt::Index n, const Ipopt::Number* x,
                        bool new_x, Ipopt::Number& obj_value);

    virtual bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                             Ipopt::Number* grad_f);

    virtual bool eval_g(Ipopt::Index n, const Ipopt::Number* x,
                        bool new_x, Ipopt::Index m, Ipopt::Number* g);

    virtual bool eval_jac_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                            Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow,
                            Ipopt::Index *jCol, Ipopt::Number* values);
    virtual bool eval_h(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                        Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number* lambda,
                        bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index* iRow,
                        Ipopt::Index* jCol, Ipopt::Number* values);
    virtual void finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n,
                                   const Ipopt::Number* x, const Ipopt::Number* z_L,
                                   const Ipopt::Number* z_U, Ipopt::Index m, const Ipopt::Number* g,
                                   const Ipopt::Number* lambda, Ipopt::Number obj_value,
                                   const Ipopt::IpoptData* ip_data,
                                   Ipopt::IpoptCalculatedQuantities* ip_cq);
private:
    NetworkMesh &nm_;
};

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
    double computeBestPositionsTangentLS(double alpha, double beta, double thickness, bool planarity, bool projectVertically);
    double computeBestPositionsBCLS(double alpha, double beta, double thickness, bool planarity, bool projectVertically);

    double optimizeIPOPT();

    double enforcePlanarity();

    void projectOntoReference(ReferenceMesh &rm);
    void projectOnto(MyMesh &m);

    bool isBadVertex(MyMesh::VertexHandle vert);

    double calculateEquilibriumViolation();
    double calculateTotalEquilibriumViolation();
    double calculateEquilibriumViolation(MyMesh::VertexHandle vh);
    void computeRelativePrincipalDirections();

    void setupVFProperties();
    void exportVectorFields(const char *name);
    bool exportReciprocalMesh(const char *name);
    bool exportWeights(const char *name);

    void edgeFlip();


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
    void flip(MyMesh::EdgeHandle &eh);

    MyMesh subdreference_;

    NetworkMeshRenderer *renderer_;

};

#endif // NETWORKMESH_H
