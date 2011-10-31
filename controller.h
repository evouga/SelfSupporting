#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Eigen/Core>
#include <QString>

class MainWindow;
class ReferenceMesh;
class NetworkMesh;
class Solvers;
class NetworkThread;

struct Params
{
    QString statusmsg;
    double alpha;
    double beta;
    int weightsum;
    double nmresidual;
};

class Controller
{
public:
    Controller(MainWindow &w);
    ~Controller();

    enum EditMode {EM_CAMERA, EM_FREEHANDLE, EM_HEIGHTHANDLE, EM_DELETEFACE};

    void initialize();

    Solvers &getSolvers();

    void computeWeightsFromTopView();
    void resetNetworkMesh();
    void iterateNetwork();
    void projectNetwork();
    void loadMesh(const char *filename);
    void jitterMesh();
    void subdivideMesh();
    Eigen::Vector3d computeMeshCentroid();
    double computeMeshBoundingSphere(const Eigen::Vector3d &center);
    double computeMeshBoundingCircle(const Eigen::Vector3d &center);

    void computeClosestPointOnPlane(const Eigen::Vector2d &pos, int &closestidx, double &closestdist);
    int selectFace(const Eigen::Vector2d &pos);
    void translateVertex(int vidx, const Eigen::Vector3d &translation);
    void translateFace(int fidx, const Eigen::Vector3d &translation);
    void dragVertex(int vidx, const Eigen::Vector3d &translation);
    void dragVertexHeight(int vidx, const Eigen::Vector3d &translation);
    void buildQuadMesh(int w, int h);
    void buildTriMesh(int w, int h);

    void setAnchor(int vidx);
    void clearAnchor(int vidx);
    void deleteFace(int fidx);

    EditMode getEditMode();

    void renderMesh3D();
    void renderPickMesh3D();
    void renderMesh2D();

    void takeScreenshot();

    void updateGLWindows();
    const Params &getParams();

    NetworkThread *getNT();

private:
    void resetParams();

    MainWindow &w_;
    ReferenceMesh *rm_;
    NetworkMesh *nm_;
    Solvers *solvers_;
    NetworkThread *nt_;

    Params p_;
};

#endif // CONTROLLER_H
