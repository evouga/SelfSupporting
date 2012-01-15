#ifndef MESH_H
#define MESH_H

#include <OpenMesh/Core/IO/MeshIO.hh>   // Must be included before mesh kernel
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <set>
#include <QMutex>
#include <memory>

#include <Eigen/Core>

class Solvers;
class MeshRenderer;
class Controller;

struct MyTraits : public OpenMesh::DefaultTraits
{
    VertexAttributes(OpenMesh::Attributes::Status | OpenMesh::Attributes::Normal);
    FaceAttributes(OpenMesh::Attributes::Status | OpenMesh::Attributes::Normal);
    EdgeAttributes(OpenMesh::Attributes::Status);

    FaceTraits
    {
      private:
        bool integrated_;
        Eigen::Vector3d u_, v_;
        Point centroid_;

      public:
        FaceT() : integrated_(false) {}

        bool integrated() const {return integrated_;}
        Eigen::Vector3d rel_principal_u() const {return u_;}
        Eigen::Vector3d rel_principal_v() const {return v_;}
        void set_integrated(bool status) {integrated_ = status;}
        void set_rel_principal_dirs(const Eigen::Vector3d &u, const Eigen::Vector3d &v) {u_ = u; v_ = v;}
        Point centroid() const {return centroid_;}
        void set_centroid(const Point &pt) {centroid_ = pt;}
    };

    EdgeTraits
    {
      private:
        double weight_;

      public:
        EdgeT() : weight_(0) {}

        double weight() const {return weight_; }
        void set_weight(double weight) {weight_ = weight;}
    };

    VertexTraits
    {
      private:
        double load_;
        double violation_;
        unsigned char flags_;

    public:
        enum VertexFlags {
            VF_NONE = 0,
            VF_ANCHORED = 1,
            VF_PINNED = 2,
            VF_HANDLED = 4,
            VF_OUTENVELOPE = 8
        };

        VertexT() : load_(0), violation_(0), flags_(VF_NONE) {}

        bool handled() const {return flags_ & VF_HANDLED; }
        void set_handled(bool handled) {handled ? flags_ |= VF_HANDLED : flags_ &= ~VF_HANDLED;}
        double load() const {return load_; }
        void set_load(double load) {load_ = load;}
        bool pinned() const {return flags_ & VF_PINNED;}
        void set_pinned(bool status) {status ? flags_ |= VF_PINNED : flags_ &= ~VF_PINNED; }
        bool anchored() const {return flags_ & VF_ANCHORED;}
        void set_anchored(bool status) {status ? flags_ |= VF_ANCHORED : flags_ &= ~VF_ANCHORED; }
        double violation() const {return violation_;}
        void set_violation(double viol) {violation_ = viol;}
        bool outofenvelope() const {return flags_ & VF_OUTENVELOPE;}
        void set_outofenvelope(bool status) {status ? flags_ |= VF_OUTENVELOPE : flags_ &= ~VF_OUTENVELOPE; }
    };
};

typedef OpenMesh::PolyMesh_ArrayKernelT<MyTraits> MyMesh;

class Mesh;

class MeshLock
{
public:
    MeshLock(Mesh &m);
    ~MeshLock();

private:
    Mesh &m_;
    MeshLock(const MeshLock &other);
    MeshLock &operator=(const MeshLock &other);
};

class Mesh
{
public:
    Mesh(Controller &cont);
    virtual ~Mesh();

    enum PrimType {PT_NONE, PT_VERTEX, PT_EDGE, PT_FACE};

    MyMesh &getMesh();

    std::auto_ptr<MeshLock> acquireMesh();

    void clearMesh();

    bool loadMesh(const char *name);
    bool saveMesh(const char *name);
    bool importOBJ(const char *name);
    bool exportOBJ(const char *name);


    virtual MeshRenderer &getRenderer()=0;

    Eigen::Vector3d computeCentroid();
    double computeBoundingSphere(const Eigen::Vector3d &center);
    double computeBoundingCircle(const Eigen::Vector3d &center);

    void translateVertex(int vidx, const Eigen::Vector3d &translation);
    void translateFace(int fidx, const Eigen::Vector3d &translation);

    void getNRing(int vidx, int n, std::set<int> &nring);

    void setPlaneAreaLoads(double density, double thickness);
    void setSurfaceAreaLoads(double density, double thickness, double extramass);
    void setConstantLoads(double density, double thickness);

    void subdivide(bool subdivideBoundary);
    void triangleSubdivide(bool smoothBoundary);
    void triangulate();

    static Eigen::Vector3d projectToFace(const MyMesh &mesh, MyMesh::FaceHandle fh, const Eigen::Vector3d &p);
    static Eigen::Vector3d projectToFaceZ(const MyMesh &mesh, MyMesh::FaceHandle fh, const Eigen::Vector3d &p);
    static Eigen::Vector3d approximateClosestPoint(const MyMesh &mesh, const Eigen::Vector3d &p);
    static Eigen::Vector3d approximateClosestZParallel(const MyMesh &mesh, const Eigen::Vector3d &p);
    void edgeEndpoints(MyMesh::EdgeHandle edge, MyMesh::Point &p1, MyMesh::Point &p2);
    bool edgePinned(MyMesh::EdgeHandle edge);
    bool faceContainsPinnedVert(MyMesh::FaceHandle face);

    // Finds z = ax + by + c for a given face
    static void computeFacePlane(const MyMesh &mesh, MyMesh::FaceHandle face, double &a, double &b, double &c);
    double isotropicDihedralAngle(MyMesh::EdgeHandle edge);
    double dihedralAngle(MyMesh::EdgeHandle edge);

    Eigen::Matrix2d approximateHessian(MyMesh::FaceHandle face);
    Eigen::Matrix2d approximateHessianVertex(MyMesh::VertexHandle vert);
    double faceAreaOnPlane(MyMesh::FaceHandle face);
    void dilate(double factor);

    friend class MeshLock;
protected:
    void copyMesh(const MyMesh &m);
    static double randomDouble();

    double vertexAreaOnPlane(MyMesh::VertexHandle vert);
    double vertexArea(MyMesh::VertexHandle vert);
    double edgeArea(MyMesh::EdgeHandle edge);
    static int numFaceVerts(const MyMesh &mesh, MyMesh::FaceHandle face);
    MyMesh::Point computeEdgeMidpoint(MyMesh::EdgeHandle edge);
    void removeVertex(MyMesh::VertexHandle vert);

    void computePlaneBarycentricCoordinates(const Eigen::Vector2d &pt, MyMesh::FaceHandle face, double &alpha, double &beta);
    int findEnclosingPlaneTriangle(const Eigen::Vector2d &pt, double &alpha, double &beta);
    void computeEigenstuff(const Eigen::Matrix2d &M, double &lambda1, double &lambda2, Eigen::Vector2d &v1, Eigen::Vector2d &v2);
    static void computeCentroids(MyMesh &m);

    MyMesh mesh_;
    Controller &cont_;

    int getMeshID();
    void invalidateMesh();

private:
    void lockMesh();
    void unlockMesh();

    QMutex meshMutex_;

    QMutex idMutex_;
    int meshID_;

};

#endif // MESH_H
