#include "controller.h"
#include "mesh.h"
#include "mainwindow.h"
#include <QMessageBox>
#include <QString>
#include "meshrenderer.h"
#include "referencemesh.h"
#include "networkmesh.h"
#include "referencemeshrenderer.h"
#include "solvers.h"
#include "networkthread.h"
#include "stressmesh.h"
#include "networkmeshrenderer.h"
#include <QDateTime>
#include <fstream>
#include <string>
#include <sstream>
#include "camera.h"

using namespace Eigen;
using namespace std;

Controller::Controller(MainWindow &w) : w_(w)
{
    rm_ = new ReferenceMesh(*this);
    nm_ = new NetworkMesh(*this);
    sm_ = new StressMesh(*this);
    solvers_ = new Solvers();
    nt_ = new NetworkThread(*this);
}

Controller::~Controller()
{
    nt_->stop();
    nt_->wait(ULONG_MAX);
    delete nt_;
    delete rm_;
    delete nm_;
    delete sm_;
    delete solvers_;
}

NetworkThread *Controller::getNT()
{
    return nt_;
}

void Controller::initialize()
{
    resetParams();
    buildQuadMesh(10,10);
    nt_->start();
}

Solvers &Controller::getSolvers()
{
    return *solvers_;
}

void Controller::importOBJ(const char *filename)
{
    if(!rm_->importOBJ(filename))
    {
        QString msg = "Couldn't read file " + QString(filename) + ". No mesh was imported.";
        QMessageBox::warning(&w_, "Couldn't Read File", msg, QMessageBox::Ok);
        return;
    }
    p_.statusmsg = "Imported geometry from " + QString(filename) + ".";
    resetNetworkMesh();
 //   w_.centerCameras();
}

void Controller::addMesh(const char *filename)
{
    if(!rm_->addOBJ(filename))
    {
        QString msg = "Couldn't add " + QString(filename) + " to reference mesh.";
        QMessageBox::warning(&w_, "Couldn't Add Mesh", msg, QMessageBox::Ok);
        return;
    }
    p_.statusmsg = "Added mesh " + QString(filename) + ".";
    resetNetworkMesh();
    //w_.centerCameras();
}

void Controller::loadMesh(const char *filename)
{
    if(!rm_->loadMesh(filename))
    {
        QString msg = "Couldn't read file " + QString(filename) + ". No mesh was loaded.";
        QMessageBox::warning(&w_, "Couldn't Read File", msg, QMessageBox::Ok);
        return;
    }
    p_.statusmsg = "Loaded mesh " + QString(filename) + ".";
    resetNetworkMesh();
    //w_.centerCameras();
}

void Controller::saveMesh(const char *filename)
{
    if(!rm_->saveMesh(filename))
    {
        QString msg = "Couldn't write file " + QString(filename) + ". Save failed.";
        QMessageBox::warning(&w_, "Couldn't Write File", msg, QMessageBox::Ok);
        return;
    }
    p_.statusmsg = "Saved mesh " + QString(filename) + ".";
}

void Controller::saveNetwork(const char *filename)
{
    if(!nm_->saveMesh(filename))
    {
        QString msg = "Couldn't write file " + QString(filename) + ". Save failed.";
        QMessageBox::warning(&w_, "Couldn't Write File", msg, QMessageBox::Ok);
        return;
    }
    p_.statusmsg = "Saved thrust network " + QString(filename) + ".";
}

void Controller::saveNetworkWeights(const char *filename)
{
    if(!nm_->exportWeights(filename))
    {
        QString msg = "Couldn't write weights file " + QString(filename) + ". Save failed.";
        QMessageBox::warning(&w_, "Couldn't Write File", msg, QMessageBox::Ok);
        return;
    }
}

void Controller::saveNetworkEverything(const char *filename)
{
    string name(filename);
    string objname = name + "-n.obj";
    if(!nm_->exportOBJ(objname.c_str()))
    {
        QString msg = "Couldn't write network mesh file " + QString(objname.c_str()) + ". Save failed.";
        QMessageBox::warning(&w_, "Couldn't Write File", msg, QMessageBox::Ok);
        return;
    }
    computeConjugateDirs();
    string stressname = name + "-n-stress.obj";
    if(!sm_->exportOBJ(stressname.c_str()))
    {
        QString msg = "Couldn't write stress network file " + QString(stressname.c_str()) + ". Save failed.";
        QMessageBox::warning(&w_, "Couldn't Write File", msg, QMessageBox::Ok);
        return;
    }
    string vfname = name + "-n-vf.txt";
    nm_->exportVectorFields(vfname.c_str());
    string reciprocalname = name + "-n-recip.obj";
    if(!nm_->exportReciprocalMesh(reciprocalname.c_str()))
    {
        QString msg = "Couldn't write reciprocal mesh file " + QString(reciprocalname.c_str()) + ". Save failed.";
        QMessageBox::warning(&w_, "Couldn't Write File", msg, QMessageBox::Ok);
        return;
    }
}


void Controller::exportOBJ(const char *filename)
{
    if(!rm_->exportOBJ(filename))
    {
        QString msg = "Couldn't write file " + QString(filename) + ". Save failed.";
        QMessageBox::warning(&w_, "Couldn't Write File", msg, QMessageBox::Ok);
        return;
    }
    p_.statusmsg = "Exported geometry to " + QString(filename) + ".";
}

void Controller::exportNetworkOBJ(const char *filename)
{
    nm_->setupVFProperties();
    if(!nm_->exportOBJ(filename))
    {
        QString msg = "Couldn't write file " + QString(filename) + ". Save failed.";
        QMessageBox::warning(&w_, "Couldn't Write File", msg, QMessageBox::Ok);
        return;
    }
    p_.statusmsg = "Exported network geometry to " + QString(filename) + ".";
}

void Controller::computeWeightsFromTopView()
{
    rm_->setPlaneAreaLoads(p_.density, p_.thickness);
    double wresidual = nm_->computeWeightsOnPlane(*rm_, p_.weightsum);
    double hresidual = nm_->updateHeights();
    p_.statusmsg = "Computed weights, with residual " + QString::number(wresidual) + ", heights with residual " + QString::number(hresidual) + ".";
    w_.updateGLWindows();
}

void Controller::resetNetworkMesh()
{
    nm_->copyFromReferenceMesh(*rm_);
    resetParams();
    p_.statusmsg = "Reset network mesh to reference mesh.";
    p_.edges = nm_->getMesh().n_edges();
    p_.verts = nm_->getMesh().n_vertices();
    w_.updateGLWindows();
}

void Controller::computeConjugateDirs()
{
    sm_->buildFromThrustNetwork(*nm_);
    nm_->computeRelativePrincipalDirections();
    w_.updateGLWindows();
}

void Controller::projectNetwork()
{
    nm_->projectOntoReference(*rm_);
    w_.updateGLWindows();
}

void Controller::iterateNetwork()
{
    nm_->setSurfaceAreaLoads(p_.density, p_.thickness, p_.extramass);
    double maxstress = p_.maxStress;
    if(!p_.enforceMaxWeight)
        maxstress = std::numeric_limits<double>::infinity();
    nm_->computeBestWeights(maxstress, p_.thickness);
    double residualp = nm_->computeBestPositionsTangentLS(p_.alpha, p_.beta, p_.thickness, p_.planarity);

//    p_.statusmsg = "Projected onto best weights and positions. Residual after calculating best weights " + QString::number(residualw)
//            + ", and after adjusting position " + QString::number(residualp) + "." + " Alpha: " + QString::number(p_.alpha) + " Beta: " + QString::number(p_.beta);
    p_.alpha /= 2.;
    p_.alpha = std::max(p_.alpha, 1e-15);
    if(p_.nmresidual < 1.1*residualp)
    {
        p_.beta *= 1.1;
        p_.beta = std::min(p_.beta, 1e15);
        cout << "increasing beta to " << p_.beta << endl;
    }
    p_.nmresidual = residualp;
//    for(MyMesh::VertexIter vi = rm_->getMesh().vertices_begin(); vi != rm_->getMesh().vertices_end(); ++vi)
//    {
//        if(rm_->getMesh().data(vi.handle()).handled())
//            cout << "handle " << vi.handle().idx()+1 << endl;
//    }
}

void Controller::computeBestWeights()
{
    nm_->setSurfaceAreaLoads(p_.density, p_.thickness, p_.extramass);
    double maxstress = p_.maxStress;
    if(!p_.enforceMaxWeight)
        maxstress = std::numeric_limits<double>::infinity();
    p_.nmresidual = nm_->computeBestWeights(maxstress, p_.thickness);
    updateGLWindows();
}

void Controller::computeBestPositions()
{
    double residualp = nm_->computeBestPositionsTangentLS(p_.alpha, p_.beta, p_.thickness, p_.planarity);
    p_.alpha /= 2.;
    p_.alpha = std::max(p_.alpha, 1e-15);
    p_.beta *= 2.;
    p_.beta = std::min(p_.beta, 1e15);
    p_.nmresidual = residualp;
    updateGLWindows();
}

void Controller::computeBestHeights()
{
    nm_->updateHeights();
    updateGLWindows();
}

void Controller::jitterMesh()
{
    rm_->jitterOnPlane();
    resetNetworkMesh();
    p_.statusmsg = "Randomly perturbed reference mesh.";
    w_.updateGLWindows();
}

void Controller::subdivideMesh(bool andboundary)
{
    nm_->subdivide(andboundary);
    nm_->saveSubdivisionReference();
    p_.alpha = 1;
    p_.beta = .1;
    p_.nmresidual = std::numeric_limits<double>::infinity();
    p_.statusmsg = "Subdivided thrust network.";
    p_.verts = nm_->getMesh().n_vertices();
    p_.edges = nm_->getMesh().n_edges();
    w_.updateGLWindows();
}

void Controller::subdivideReferenceMesh(bool andboundary)
{
    rm_->subdivide(andboundary);
    resetParams();
    p_.statusmsg = "Subdivided reference mesh.";
    w_.updateGLWindows();
}

void Controller::subdivideReferenceMeshLoop(bool andboundary)
{
    rm_->triangleSubdivide(andboundary);
    resetParams();
    p_.statusmsg = "Subdivided reference mesh.";
    w_.updateGLWindows();
}


Vector3d Controller::computeMeshCentroid()
{
    return rm_->computeCentroid();
}

double Controller::computeMeshBoundingCircle(const Vector3d &center)
{
    return rm_->computeBoundingCircle(center);
}

double Controller::computeMeshBoundingSphere(const Vector3d &center)
{
    return rm_->computeBoundingSphere(center);
}

void Controller::translateVertex(int vidx, const Vector3d &translation)
{
    rm_->translateVertex(vidx, translation);
    resetNetworkMesh();
    updateGLWindows();
}

void Controller::translateFace(int fidx, const Vector3d &translation)
{
    rm_->translateFace(fidx, translation);
    resetNetworkMesh();
    updateGLWindows();
}

void Controller::dragVertex(int vidx, const Vector3d &translation)
{
    rm_->applyLaplacianDeformation(vidx, translation);
    resetNetworkMesh();
    updateGLWindows();
}

void Controller::dragVertexHeight(int vidx, const Vector3d &translation)
{
    rm_->applyLaplacianDeformationHeight(vidx, translation);
    resetNetworkMesh();
    updateGLWindows();
}

void Controller::dragVertexTop(int vidx, const Vector3d &translation)
{
    rm_->applyLaplacianDeformationTop(vidx, translation);
    resetNetworkMesh();
    updateGLWindows();
}

void Controller::computeClosestPointOnPlane(const Vector2d &pos, int &closestidx, double &closestdist)
{
    rm_->computeClosestPointOnPlane(pos, closestidx, closestdist);
}

void Controller::renderMesh2D()
{
    if(w_.showReferenceMesh())
        rm_->getRenderer().render2D();
    if(w_.showNetworkMesh())
    {
        nm_->getRenderer().render2D();
        if(w_.showConjugateVectors())
            ((NetworkMeshRenderer &)nm_->getRenderer()).renderConjugateVectors2D();
    }

}

void Controller::renderMesh3D()
{
    if(w_.showNetworkSurface())
        ((NetworkMeshRenderer &)nm_->getRenderer()).renderSurface();
    if(w_.showNetworkMesh())    
        nm_->getRenderer().render3D();
    if(w_.showConjugateVectors())
        ((NetworkMeshRenderer &)nm_->getRenderer()).renderConjugateVectors3D();
    if(w_.showReferenceMesh())
        rm_->getRenderer().render3D();
    if(w_.showStressSurface())
        sm_->getRenderer().render3D();
}

void Controller::renderPickMesh3D()
{
    ((ReferenceMeshRenderer &)rm_->getRenderer()).renderPick3D();
}

void Controller::buildQuadMesh(int w, int h)
{
    rm_->buildQuadMesh(w,h);
    resetNetworkMesh();
    w_.centerCameras();
    w_.updateGLWindows();
}

void Controller::buildTriMesh(int w, int h)
{
    rm_->buildTriMesh(w, h);
    resetNetworkMesh();
    w_.centerCameras();
    w_.updateGLWindows();
}

void Controller::buildHexMesh(int w, int h)
{
    rm_->buildHexMesh(w,h);
    resetNetworkMesh();
    w_.centerCameras();
    w_.updateGLWindows();
}

void Controller::updateGLWindows()
{
    w_.updateGLWindows();
}

void Controller::setHandle(vector<int> &vidx)
{
    for(vector<int>::iterator it = vidx.begin(); it != vidx.end(); ++it)
        rm_->setHandle(*it, true);
    w_.updateGLWindows();
}

void Controller::clearHandle(vector<int> &vidx)
{
    for(vector<int>::iterator it = vidx.begin(); it != vidx.end(); ++it)
        rm_->setHandle(*it, false);
    w_.updateGLWindows();
}

void Controller::setPin(vector<int> & vidxs)
{
    for(vector<int>::iterator it = vidxs.begin(); it != vidxs.end(); ++it)
        rm_->setPin(*it, true);
    resetNetworkMesh();
    w_.updateGLWindows();
}

void Controller::clearPin(vector<int> &vidxs)
{
    for(vector<int>::iterator it = vidxs.begin(); it != vidxs.end(); ++it)
        rm_->setPin(*it, false);
    resetNetworkMesh();
    w_.updateGLWindows();
}

void Controller::setAnchor(std::vector<int> &vidx)
{
    for(vector<int>::iterator it = vidx.begin(); it != vidx.end(); ++it)
        rm_->setAnchor(*it, true);
    resetNetworkMesh();
    w_.updateGLWindows();
}

void Controller::clearAnchor(std::vector<int> &vidx)
{
    for(vector<int>::iterator it = vidx.begin(); it != vidx.end(); ++it)
        rm_->setAnchor(*it, false);
    resetNetworkMesh();
    w_.updateGLWindows();
}

void Controller::deleteFace(int fidx)
{
    rm_->deleteFace(fidx);
    resetNetworkMesh();
    w_.updateGLWindows();
}

void Controller::resetParams()
{
    p_.statusmsg = "Ready";
    p_.alpha = 1;
    p_.beta = .1;
    p_.weightsum = 50;
    p_.nmresidual = std::numeric_limits<double>::infinity();
    w_.reportParams();
}

const Params &Controller::getParams()
{
    return p_;
}

void Controller::takeScreenshot()
{
    qint64 secs = QDateTime::currentMSecsSinceEpoch()/1000;
    stringstream ss;
    ss << "screen-" << secs << ".png";
    w_.save3DScreenshot(ss.str());
    p_.statusmsg = "Took screenshot " + QString::fromStdString(ss.str()) + ".";
}

Controller::EditMode Controller::getEditMode()
{
    return w_.getEditMode();
}

void Controller::setAutoIterate(bool state)
{
    if(state)
        nt_->unpause();
    else
        nt_->pause();
}

void Controller::setEnforcePlanarity(bool state)
{
    p_.planarity = state;
}

void Controller::enforceMaxWeight(bool state)
{
    p_.enforceMaxWeight = state;
}

void Controller::setMaxStress(double stress)
{
    p_.maxStress = stress;
}

void Controller::setDensity(double density)
{
    p_.density = density;
}

void Controller::setThickness(double thickness)
{
    p_.thickness = thickness;
}

void Controller::setExtraMass(double extramass)
{
    p_.extramass = extramass;
}

void Controller::copyThrustNetwork()
{
    rm_->copyFromNetworkMesh(*nm_);
    //resetNetworkMesh();
    updateGLWindows();
}

void Controller::triangulateThrustNetwork()
{
    nm_->triangulate();
    p_.beta = .2;
    p_.nmresidual = std::numeric_limits<double>::infinity();
    p_.statusmsg = "Triangulated thrust network.";
    p_.verts = nm_->getMesh().n_vertices();
    p_.edges = nm_->getMesh().n_edges();
    w_.updateGLWindows();
}

void Controller::planarizeThrustNetwork()
{
    nm_->enforcePlanarity();
    w_.updateGLWindows();
}

void Controller::pinReferenceBoundary()
{
    rm_->pinBoundary();
    resetNetworkMesh();
    w_.updateGLWindows();
}

void Controller::unpinReferenceBoundary()
{
    rm_->unpinBoundary();
    resetNetworkMesh();
    w_.updateGLWindows();
}

void Controller::trimReferenceBoundary()
{
    rm_->trimBoundary();
    resetNetworkMesh();
    w_.updateGLWindows();
}

void Controller::swapYandZ()
{
    rm_->swapYandZ();
    resetNetworkMesh();
    w_.updateGLWindows();
}

void Controller::invertY()
{
    rm_->invertY();
    resetNetworkMesh();
    w_.updateGLWindows();
}

vector<int> Controller::selectRectangle(const Vector2d &c1, const Vector2d &c2, Camera &c)
{
    return rm_->selectRectangle(c1, c2, c);
}

void Controller::averageHeights()
{
    rm_->averageHandledHeights();
}

void Controller::dilate()
{
    rm_->dilate(0.5);
    resetNetworkMesh();
    w_.updateGLWindows();
}

void Controller::envelopeTest()
{
    auto_ptr<MeshLock> rml = rm_->acquireMesh();
    auto_ptr<MeshLock> ml = nm_->acquireMesh();
    set<MyMesh::FaceHandle> todelete;
    for(MyMesh::VertexIter vi = nm_->getMesh().vertices_begin(); vi != nm_->getMesh().vertices_end(); ++vi)
    {
        if(nm_->getMesh().data(vi.handle()).outofenvelope())
        {
            MyMesh::VertexHandle vert = rm_->getMesh().vertex_handle(vi.handle().idx());
            for(MyMesh::VertexFaceIter vfi = rm_->getMesh().vf_iter(vert); vfi; ++vfi)
            {
                todelete.insert(vfi.handle());
            }
        }
    }
    for(set<MyMesh::FaceHandle>::iterator it = todelete.begin(); it != todelete.end(); ++it)
        rm_->getMesh().delete_face(*it, true);
    rm_->getMesh().garbage_collection();
}
