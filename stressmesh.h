#ifndef STRESSMESH_H
#define STRESSMESH_H

#include "mesh.h"

class StressMeshRenderer;
class NetworkMesh;

class StressMesh : public Mesh
{
public:
    StressMesh(Controller &c);

    virtual MeshRenderer &getRenderer();

    void buildFromThrustNetwork(NetworkMesh &nm);

private:
    void buildConnectedComponent(NetworkMesh &nm, MyMesh::FaceHandle fi);
    void integrateFace(NetworkMesh &nm, MyMesh::FaceHandle face);
    void straightenStressSurface();


    double stressLoad(MyMesh::VertexHandle vert);
    double reciprocalArea(MyMesh::VertexHandle vert);

    StressMeshRenderer *renderer_;
};

#endif // STRESSMESH_H
