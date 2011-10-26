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

using namespace Eigen;

Controller::Controller(MainWindow &w) : w_(w)
{
    rm_ = new ReferenceMesh(*this);
    nm_ = new NetworkMesh(*this);
    solvers_ = new Solvers();
    nt_ = new NetworkThread(*this);
    resetParams();
}

Controller::~Controller()
{
    delete rm_;
    delete nm_;
    delete solvers_;
}

void Controller::initialize()
{
    buildQuadMesh(10,10);
    nt_->start();
}

Solvers &Controller::getSolvers()
{
    return *solvers_;
}

void Controller::loadMesh(const char *filename)
{
    if(!rm_->loadMesh(filename))
    {
        QString msg = "Could't read file " + QString(filename) + ". No mesh was loaded.";
        QMessageBox::warning(&w_, "Couldn't Read File", msg, QMessageBox::Ok);
        return;
    }
    resetNetworkMesh();
    //computeWeightsFromTopView();
    w_.centerCameras();
}

void Controller::computeWeightsFromTopView()
{
    rm_->setPlaneAreaLoads();
    double wresidual = nm_->computeWeightsOnPlane(*rm_, p_.weightsum);
    double hresidual = nm_->updateHeights();
    p_.statusmsg = "Computed weights, with residual " + QString::number(wresidual) + ", heights with residual " + QString::number(hresidual) + ".";
    w_.updateGLWindows();
}

void Controller::resetNetworkMesh()
{
    nm_->copyFromReferenceMesh(*rm_);
    resetParams();
    w_.updateGLWindows();
}

void Controller::projectNetwork()
{
    nm_->projectOntoReference(*rm_);
    w_.updateGLWindows();
}

void Controller::iterateNetwork()
{
    nm_->setPlaneAreaLoads();
    //TODO fix
    double residualw = nm_->computeBestWeights();
    double residualp = nm_->computeBestPositionsTangentLS(*rm_, 0.01*p_.alpha, 0.1*p_.beta);
    p_.statusmsg = "Projected onto best weights and positions. Residual after calculating best weights " + QString::number(residualw)
            + ", and after adjusting position " + QString::number(residualp) + ".";
    p_.alpha /= 2;
    p_.beta *= 2;
    w_.updateGLWindows();
}

void Controller::jitterMesh()
{
    rm_->jitterOnPlane();
    resetNetworkMesh();
    w_.updateGLWindows();
}

void Controller::subdivideMesh()
{
    nm_->subdivide();
    p_.beta = 2;
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

void Controller::computeClosestPointOnPlane(const Vector2d &pos, int &closestidx, double &closestdist)
{
    rm_->computeClosestPointOnPlane(pos, closestidx, closestdist);
}

void Controller::renderMesh2D()
{
    rm_->getRenderer().render2D();
}

void Controller::renderMesh3D()
{
    if(w_.showNetworkMesh())
        nm_->getRenderer().render3D();
    if(w_.showReferenceMesh())
        rm_->getRenderer().render3D();
}

void Controller::renderPickMesh3D()
{
    ((ReferenceMeshRenderer &)rm_->getRenderer()).renderPick3D();
}

void Controller::buildQuadMesh(int w, int h)
{
    rm_->buildQuadMesh(w,h);
    resetNetworkMesh();
    //computeWeightsFromTopView();
    w_.centerCameras();
    w_.updateGLWindows();
}

void Controller::buildTriMesh(int w, int h)
{
    rm_->buildTriMesh(w, h);
    resetNetworkMesh();
    //computeWeightsFromTopView();
    w_.centerCameras();
    w_.updateGLWindows();
}

void Controller::updateGLWindows()
{
    w_.updateGLWindows();
}

void Controller::setAnchor(int vidx)
{
    rm_->setAnchor(vidx, true);
}

void Controller::clearAnchor(int vidx)
{
    rm_->setAnchor(vidx, false);
}

void Controller::resetParams()
{
    p_.statusmsg = "Ready";
    p_.alpha = 10;
    p_.beta = 2;
    p_.weightsum = 50;
}

const Params &Controller::getParams()
{
    return p_;
}
