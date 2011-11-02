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
        bool anchored_;
      public:
        VertexT() : load_(0), anchored_(false) {}

        bool anchored() const {return anchored_; }
        void set_anchored(bool anchored) {anchored_ = anchored;}
        double load() const {return load_; }
        void set_load(double load) {load_ = load;}
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

    //void copyMesh(const MyMesh &m);
    void clearMesh();

    virtual MeshRenderer &getRenderer()=0;

    Eigen::Vector3d computeCentroid();
    double computeBoundingSphere(const Eigen::Vector3d &center);
    double computeBoundingCircle(const Eigen::Vector3d &center);

    void translateVertex(int vidx, const Eigen::Vector3d &translation);
    void translateFace(int fidx, const Eigen::Vector3d &translation);

    void getNRing(int vidx, int n, std::set<int> &nring);

    void setPlaneAreaLoads();
    void setSurfaceAreaLoads();

    Eigen::Vector3d projectToFace(MyMesh::FaceHandle fh, const Eigen::Vector3d &p);
    Eigen::Vector3d approximateClosestPoint(const Eigen::Vector3d &p);
    void edgeEndpoints(MyMesh::EdgeHandle edge, MyMesh::Point &p1, MyMesh::Point &p2);


    friend class MeshLock;
protected:
    void copyMesh(const MyMesh &m);
    static double randomDouble();

    double faceAreaOnPlane(MyMesh::FaceHandle face);
    double vertexAreaOnPlane(MyMesh::VertexHandle vert);
    double vertexArea(MyMesh::VertexHandle vert);
    double edgeArea(MyMesh::EdgeHandle edge);
    int numFaceVerts(MyMesh::FaceHandle face);
    MyMesh::Point computeEdgeMidpoint(MyMesh::EdgeHandle edge);

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
