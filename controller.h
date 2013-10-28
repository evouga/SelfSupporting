#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Eigen/Core>
#include <QString>
#include <vector>
#include <QObject>

class MainWindow;
class ReferenceMesh;
class NetworkMesh;
class Solvers;
class NetworkThread;
class StressMesh;
class Camera;
class QTimer;

struct Params
{
    QString statusmsg;
    double alpha;
    double beta;
    int weightsum;
    double nmresidual;

    bool enforceMaxWeight;
    bool planarity;
    double maxStress;
    double density;
    double thickness;
    double extramass;
    int verts;
    int edges;
    bool projectVertically;
    int influence;
    bool excludePinned;
    double modeAmplitude;
};

class Controller : public QObject
{
    Q_OBJECT

public:
    Controller(MainWindow &w);
    ~Controller();

    enum EditMode {EM_CAMERA, EM_FREEHANDLE, EM_HEIGHTHANDLE, EM_DELETEFACE, EM_PIN, EM_ANCHOR, EM_TOPHANDLE, EM_EDGECOLLAPSE};

    void initialize();

    Solvers &getSolvers();

    void computeWeightsFromTopView();
    void resetNetworkMesh();
    void iterateNetwork();
    void computeBestWeights();
    void computeBestPositions();
    void computeBestHeights();

    void projectNetwork();
    void planarizeThrustNetwork();
    void loadMesh(const char *filename);
    void saveMesh(const char *filename);
    void saveNetwork(const char *filename);
    void saveNetworkEverything(const char *filename);
    void saveNetworkWeights(const char *filename);
    void importOBJ(const char *filename);
    void exportOBJ(const char *filename);
    void addMesh(const char *filename);
    void exportNetworkOBJ(const char *filename);
    void jitterMesh();
    void copyThrustNetwork();
    void subdivideMesh(bool andboundary);
    void subdivideReferenceMesh(bool andboundary);
    void subdivideReferenceMeshLoop(bool andboundary);
    void triangulateThrustNetwork();
    void calculateMode();
    Eigen::Vector3d computeMeshCentroid();
    double computeMeshBoundingSphere(const Eigen::Vector3d &center);
    double computeMeshBoundingCircle(const Eigen::Vector3d &center);

    void computeClosestPointOnPlane(const Eigen::Vector2d &pos, int &closestidx, double &closestdist);
    int selectFace(const Eigen::Vector2d &pos);
    void translateVertex(int vidx, const Eigen::Vector3d &translation);
    void translateFace(int fidx, const Eigen::Vector3d &translation);
    void dragVertex(int vidx, const Eigen::Vector3d &translation);
    void dragVertexHeight(int vidx, const Eigen::Vector3d &translation);
    void dragVertexTop(int vidx, const Eigen::Vector3d &translation);
    void buildQuadMesh(int w, int h);
    void buildTriMesh(int w, int h);
    void buildHexMesh(int w, int h);

    void setHandle(std::vector<int> &vidx);
    void clearHandle(std::vector<int> &vidx);
    void setPin(std::vector<int> &vidxs);
    void clearPin(std::vector<int> &vidxs);
    void setAnchor(std::vector<int> &vidx);
    void clearAnchor(std::vector<int> &vidx);
    void deleteFace(int fidx);
    void edgeCollapse(int eidx);
    void deselectAll();
    void pinSelected();

    EditMode getEditMode();

    void renderMesh3D();
    void renderPickMesh3D();
    void renderMesh2D();
    void edgeFlip();

    void takeScreenshot();
    void setAutoIterate(bool state);
    void setEnforcePlanarity(bool state);
    void enforceMaxWeight(bool state);
    void setMaxStress(double stress);
    void setDensity(double density);
    void setThickness(double thickness);
    void setExtraMass(double extramass);
    void setProjectVertically(bool state);
    void laplacianTest();
    void computeConjugateDirs();
    void setInfluence(int influence);
    void setExcludePinned(bool state);
    void setModeAmplitude(int value);

    void pinReferenceBoundary();
    void unpinReferenceBoundary();
    void trimReferenceBoundary();
    void swapYandZ();
    void invertY();

    void averageHeights();
    void selectPinned();
    void dilate();

    void updateGLWindows();
    const Params &getParams();
    void envelopeTest();

    NetworkThread *getNT();
    std::vector<int> selectRectangle(const Eigen::Vector2d &c1, const Eigen::Vector2d &c2, Camera &c);

public slots:
    void tick();

private:
    void resetParams();

    MainWindow &w_;
    ReferenceMesh *rm_;
    NetworkMesh *nm_;
    StressMesh *sm_;
    Solvers *solvers_;
    NetworkThread *nt_;

    Params p_;

    QTimer *timer_;
    double time_;
};

#endif // CONTROLLER_H
