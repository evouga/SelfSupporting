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
    VertexAttributes(OpenMesh::Attributes::Status);
    FaceAttributes(OpenMesh::Attributes::Status);
    EdgeAttributes(OpenMesh::Attributes::Status);

    FaceTraits
    {
      private:
        bool integrated_;
        Eigen::Vector3d u_, v_;

      public:
        FaceT() : integrated_(false) {}

        bool integrated() const {return integrated_;}
        Eigen::Vector3d rel_principal_u() const {return u_;}
        Eigen::Vector3d rel_principal_v() const {return v_;}
        void set_integrated(bool status) {integrated_ = status;}
        void set_rel_principal_dirs(const Eigen::Vector3d &u, const Eigen::Vector3d &v) {u_ = u; v_ = v;}
    };

    EdgeTraits
    {
      private:
        double weight_;

        bool iscrease_;
        double creaseang_;

      public:
        EdgeT() : weight_(0), iscrease_(false), creaseang_(0) {}

        double weight() const {return weight_; }
        void set_weight(double weight) {weight_ = weight;}

        bool is_crease() const {return iscrease_; }
        void set_is_crease(bool status) {iscrease_ = status; }

        double crease_value() const {return creaseang_; }
        void set_crease_value(double angle) {creaseang_ = angle; }
    };

    VertexTraits
    {
      private:
        double load_;
        double violation_;
        bool anchored_;
        bool pinned_;
        public:
        VertexT() : load_(0), violation_(0), anchored_(false), pinned_(false) {}

        bool anchored() const {return anchored_; }
        void set_anchored(bool anchored) {anchored_ = anchored;}
        double load() const {return load_; }
        void set_load(double load) {load_ = load;}
        bool pinned() const {return pinned_;}
        void set_pinned(bool status) {pinned_ = status; }
        double violation() const {return violation_;}
        void set_violation(double viol) {violation_ = viol;}
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

    const MyMesh &getMesh();

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

    void setPlaneAreaLoads(double density);
    void setSurfaceAreaLoads(double density);

    void subdivide();
    void triangulate();

    static Eigen::Vector3d projectToFace(const MyMesh &mesh, MyMesh::FaceHandle fh, const Eigen::Vector3d &p);
    static Eigen::Vector3d approximateClosestPoint(const MyMesh &mesh, const Eigen::Vector3d &p);
    void edgeEndpoints(MyMesh::EdgeHandle edge, MyMesh::Point &p1, MyMesh::Point &p2);
    bool edgePinned(MyMesh::EdgeHandle edge);
    bool faceContainsPinnedVert(MyMesh::FaceHandle face);

    // Finds z = ax + by + c for a given face
    void computeFacePlane(MyMesh::FaceHandle face, double &a, double &b, double &c);
    double isotropicDihedralAngle(MyMesh::EdgeHandle edge);
    double dihedralAngle(MyMesh::EdgeHandle edge);

    Eigen::Matrix2d approximateHessian(MyMesh::FaceHandle face);
    double faceAreaOnPlane(MyMesh::FaceHandle face);

    friend class MeshLock;
protected:
    void copyMesh(const MyMesh &m);
    static double randomDouble();

    double vertexAreaOnPlane(MyMesh::VertexHandle vert);
    double vertexArea(MyMesh::VertexHandle vert);
    double edgeArea(MyMesh::EdgeHandle edge);
    int numFaceVerts(MyMesh::FaceHandle face);
    MyMesh::Point computeEdgeMidpoint(MyMesh::EdgeHandle edge);

    void removeVertex(MyMesh::VertexHandle vert);

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
