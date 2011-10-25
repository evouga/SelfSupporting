#include "openglpanel2d.h"
#include <algorithm>
#include "mesh.h"
#include <Eigen/Core>
#include <iostream>
#include <QMouseEvent>
#include "mainwindow.h"

using namespace Eigen;

OpenGLPanel2D::OpenGLPanel2D(QWidget *parent) :
    QGLWidget(parent), cont_(NULL), scale_(1.0), draggingidx_(-1)
{

}

void OpenGLPanel2D::setController(Controller &c)
{
    cont_ = &c;
}

void OpenGLPanel2D::initializeGL()
{
    glShadeModel(GL_FLAT);
    glClearColor(1.0, 1.0, 1.0, 0.0);
    glClearDepth(1.0);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
}

void OpenGLPanel2D::resizeGL(int w, int h)
{
    width_ = w;
    height_ = h;
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    double side = std::min(w,h);
    glOrtho(-w/side, w/side, -h/side, h/side, 4.0, -15.0);
    glMatrixMode(GL_MODELVIEW);
}

void OpenGLPanel2D::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    glTranslatef(0.0, 0.0, 0.0);
    glScalef(scale_,scale_,scale_);

    if(cont_)
        cont_->renderMesh2D();
}

void OpenGLPanel2D::centerCamera()
{
    if(!cont_)
        return;
    Vector3d centroid = cont_->computeMeshCentroid();
    scale_ = 1.0/cont_->computeMeshBoundingCircle(centroid);
}

void OpenGLPanel2D::mousePressEvent(QMouseEvent *event)
{
    if(!cont_)
        return;
    scaleMousePos(event->pos().x(),event->pos().y(), dragPos_[0], dragPos_[1]);
    int idx;
    double dist;
    cont_->computeClosestPointOnPlane(dragPos_, idx, dist);
    if(dist < 0.02/scale_)
    {
        draggingidx_ = idx;
    }
    updateGL();
}

void OpenGLPanel2D::mouseMoveEvent(QMouseEvent *event)
{
    if(draggingidx_ == -1)
        return;

    double sx, sy;
    scaleMousePos(event->pos().x(), event->pos().y(), sx, sy);
    Vector3d translation(sx-dragPos_[0], 0, sy-dragPos_[1]);
    cont_->translateVertex(draggingidx_, translation);
    dragPos_[0] = sx;
    dragPos_[1] = sy;
}

void OpenGLPanel2D::mouseReleaseEvent(QMouseEvent *)
{
    draggingidx_ = -1;
}

void OpenGLPanel2D::scaleMousePos(int x, int y, double &sx, double &sy)
{
    sx = (2.0 * x / double(width_-1) - 1.0)/scale_;
    sy = (2.0 * (height_ - y -1) / double(height_-1) - 1.0)/scale_;
}
