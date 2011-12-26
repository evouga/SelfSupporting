#include "openglpanel3d.h"
#include <iostream>
#include <QtGui>
#include <Eigen/Core>
#include "mesh.h"
#include "controller.h"
#include "meshrenderer.h"
#include "YImage.hpp"

using namespace Eigen;

OpenGLPanel3D::OpenGLPanel3D(QWidget *parent) :
    QGLWidget(parent), cont_(NULL), c_(), translator_(c_, 1.0), rotator_(c_), zoomer_(c_, 10.0), selector_(c_)
{
}

void OpenGLPanel3D::setController(Controller &c)
{
    cont_ = &c;
    selector_.setController(c);
}

void OpenGLPanel3D::initializeGL()
{
    glShadeModel(GL_SMOOTH);
    glClearColor(1.0, 1.0, 1.0, 0.0);
    glClearDepth(1.0);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);
}

void OpenGLPanel3D::resizeGL(int w, int h)
{
    c_.setPerpective(60.0, 1.0);
    c_.setViewport(w, h);
}

void OpenGLPanel3D::paintGL()
{
    if(selector_.selectionQueued())
    {
        // pick face
        glClearColor(0.0, 0.0, 0.0, 0.0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glColor3f(0.0, 0.0, 0.0);
        c_.applyViewport();
        c_.applyProjection();
        c_.applyLookAt();
        if(cont_)
            cont_->renderPickMesh3D();
        const Vector2d &pos = selector_.getQueuedPos();
        GLubyte pixels[3];
        c_.getPixelAt(pos, pixels);
        unsigned int n;
        Mesh::PrimType type;
        MeshRenderer::decodeFromColor(n, type, pixels);
        selector_.primitiveSelected(type, n);
    }

    glClearColor(1.0, 1.0, 1.0, 0.0);
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glColor3f (0.0, 0.0, 0.0);

    c_.applyViewport();
    c_.applyProjection();
    c_.applyLookAt();

    static GLfloat lightPosition[4] = { 0.0, -1.0, 0.0, 0.0 };
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);


    if(cont_)
        cont_->renderMesh3D();

    if(takeScreenshot_)
    {
        takeScreenshot_ = false;
        GLint view[4] ;
        glGetIntegerv( GL_VIEWPORT, view ) ;

        YImage img ;
        img.resize( view[2], view[3] ) ;
        //glReadBuffer( GL_FRONT );
        glReadPixels( view[0], view[1], view[2], view[3], GL_RGBA, GL_UNSIGNED_BYTE, img.data() ) ;
        //glReadBuffer( GL_BACK );

        img.flip() ;
        img.save( ssFilename_.c_str() ) ;
    }
}

void OpenGLPanel3D::scaleMousePos(int x, int y, double &scaledx, double &scaledy) const
{
    int w, h;
    c_.getViewport(w,h);
    scaledx = 2.0 * x / double(w-1) - 1.0;
    scaledy = 2.0 * (h - y -1) / double(h-1) - 1.0;
}

OpenGLPanel3D::MouseAction OpenGLPanel3D::deduceAction(QMouseEvent *event)
{
    Controller::EditMode curmode = cont_->getEditMode();

    if( (curmode == Controller::EM_CAMERA) || (event->modifiers() & Qt::ControlModifier) )
    {
        if(event->buttons() & Qt::LeftButton)
        {
            if(event->modifiers() & Qt::ShiftModifier)
                return MA_TRANSLATE;
            return MA_ROTATE;
        }
        else if(event->buttons() & Qt::RightButton)
            return MA_ZOOM;
        return MA_NONE;
    }
    else if(curmode == Controller::EM_FREEHANDLE)
    {
        if(event->buttons() & Qt::LeftButton)
        {
            return MA_ADDHANDLEFREE;
        }
        else if(event->buttons() & Qt::RightButton)
        {
            return MA_DELETEHANDLE;
        }
        return MA_NONE;
    }
    else if(curmode == Controller::EM_HEIGHTHANDLE)
    {
        if(event->buttons() & Qt::LeftButton)
        {
            return MA_ADDHANDLEHEIGHT;
        }
        else if(event->buttons() & Qt::RightButton)
        {
            return MA_DELETEHANDLE;
        }
        return MA_NONE;
    }
    else if(curmode == Controller::EM_TOPHANDLE)
    {
        if(event->buttons() & Qt::LeftButton)
        {
            return MA_ADDHANDLETOP;
        }
        else if(event->buttons() & Qt::RightButton)
        {
            return MA_DELETEHANDLE;
        }
        return MA_NONE;
    }
    else if(curmode == Controller::EM_PIN)
    {
        if(event->buttons() & Qt::LeftButton)
        {
            return MA_ADDPIN;
        }
        else if(event->buttons() & Qt::RightButton)
        {
            return MA_DELETEPIN;
        }
    }
    else if(curmode == Controller::EM_ANCHOR)
    {
        if(event->buttons() & Qt::LeftButton)
        {
            return MA_ADDANCHOR;
        }
        else if(event->buttons() & Qt::RightButton)
        {
            return MA_DELETEANCHOR;
        }
    }
    else if(curmode == Controller::EM_DELETEFACE)
    {
        if(event->buttons() & Qt::LeftButton)
            return MA_DELETEFACE;
        return MA_NONE;
    }

    return MA_NONE;
}

void OpenGLPanel3D::mousePressEvent(QMouseEvent *event)
{
    int x = event->pos().x();
    int y = event->pos().y();
    Vector2d pos;
    scaleMousePos(x,y,pos[0],pos[1]);

    MouseAction ma = deduceAction(event);

    switch(ma)
    {
        case MA_TRANSLATE:
        {
            translator_.startTranslation(pos);
            break;
        }
        case MA_ROTATE:
        {
            rotator_.startRotation(pos);
            break;
        }
        case MA_ZOOM:
        {
            zoomer_.startZoom(pos);
            break;
        }
        case MA_ADDHANDLEFREE:
        {
            // queue pos for rendering
            selector_.startEditing(pos, Selector::SM_DRAGFREE);
            cont_->updateGLWindows();
            break;
        }
        case MA_ADDHANDLEHEIGHT:
        {
            selector_.startEditing(pos, Selector::SM_DRAGHEIGHT);
            cont_->updateGLWindows();
            break;
        }
        case MA_ADDHANDLETOP:
        {
            selector_.startEditing(pos, Selector::SM_DRAGTOP);
            cont_->updateGLWindows();
            break;
        }
        case MA_DELETEHANDLE:
        {
            selector_.startEditing(pos, Selector::SM_CLEARHANDLE);
            cont_->updateGLWindows();
            break;
        }
        case MA_ADDPIN:
        {
            selector_.startEditing(pos, Selector::SM_ADDPIN);
            cont_->updateGLWindows();
            break;
        }
        case MA_DELETEPIN:
        {
            selector_.startEditing(pos, Selector::SM_DELETEPIN);
            cont_->updateGLWindows();
            break;
        }
        case MA_ADDANCHOR:
        {
            selector_.startEditing(pos, Selector::SM_ADDANCHOR);
            cont_->updateGLWindows();
            break;
        }
        case MA_DELETEANCHOR:
        {
            selector_.startEditing(pos, Selector::SM_DELETEANCHOR);
            cont_->updateGLWindows();
            break;
        }
        case MA_DELETEFACE:
        {
            selector_.startEditing(pos, Selector::SM_DELETEFACE);
            cont_->updateGLWindows();
            break;
        }
        default:
            break;
    }
}

void OpenGLPanel3D::mouseMoveEvent(QMouseEvent *event)
{
    int x = event->pos().x();
    int y = event->pos().y();
    Vector2d pos;
    scaleMousePos(x,y,pos[0],pos[1]);
    translator_.updateTranslation(pos);
    rotator_.updateRotation(pos);
    zoomer_.updateZoom(pos);
    selector_.updateEditing(pos);
    updateGL();
}

void OpenGLPanel3D::mouseReleaseEvent(QMouseEvent *event)
{
    int x = event->pos().x();
    int y = event->pos().y();
    Vector2d pos;
    scaleMousePos(x,y,pos[0],pos[1]);
    translator_.stopTranslation();
    rotator_.stopRotation();
    zoomer_.stopZoom();
    selector_.stopEditing(pos);
}

void OpenGLPanel3D::centerCamera()
{
    if(!cont_)
        return;
    Vector3d centroid = cont_->computeMeshCentroid();
    double radius = cont_->computeMeshBoundingSphere(centroid);
    c_.setCenter(centroid);
    translator_.setScale(2*radius);
    zoomer_.setScale(2*radius);

    c_.setDefault3D(2*radius);
    lightPos_ = c_.getEye();
}

void OpenGLPanel3D::saveScreenshot(const std::string &filename)
{
    takeScreenshot_ = true;
    ssFilename_ = filename;
}
