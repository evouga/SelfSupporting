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

using namespace Eigen;

Controller::Controller(MainWindow &w) : w_(w)
{
    rm_ = new ReferenceMesh(*this);
    nm_ = new NetworkMesh(*this);
    solvers_ = new Solvers();
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
    double wresidual = nm_->computeWeightsOnPlane(*rm_, w_.getWeightSliderValue());
    double hresidual = nm_->updateHeights();
    QString msg = "Computed weights, with residual " + QString::number(wresidual) + ", heights with residual " + QString::number(hresidual) + ".";
    w_.setStatusBar(msg);
    w_.updateGLWindows();
}

void Controller::resetNetworkMesh()
{
    nm_->copyFromReferenceMesh(*rm_);
    w_.setAlphaSliderValue(.1);
    w_.setBetaSliderValue(.2);
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
    double alpha = w_.getAlphaSliderValue();
    double beta = w_.getBetaSliderValue();
    double residualw = nm_->computeBestWeights();
    double residualp = nm_->computeBestPositionsTangentLS(*rm_, alpha, beta);
    QString msg = "Projected onto best weights and positions. Residual after calculating best weights " + QString::number(residualw)
            + ", and after adjusting position " + QString::number(residualp) + ".";
    w_.setStatusBar(msg);
    w_.setAlphaSliderValue(alpha*0.5);
    w_.setBetaSliderValue(2*beta);
    w_.updateGLWindows();
}

void Controller::jitterMesh()
{
    rm_->jitterOnPlane();
    w_.updateGLWindows();
}

void Controller::subdivideMesh()
{
    nm_->subdivide();
    w_.setBetaSliderValue(.2);
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
    updateGLWindows();
}

void Controller::translateFace(int fidx, const Vector3d &translation)
{
    rm_->translateFace(fidx, translation);
    updateGLWindows();
}

void Controller::dragVertex(int vidx, const Vector3d &translation)
{
    rm_->applyLaplacianDeformation(vidx, translation);
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
