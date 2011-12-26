#ifndef OpenGLPanel3D_H
#define OpenGLPanel3D_H

#include <QGLWidget>
#include "camera.h"
#include "translator.h"
#include "rotator.h"
#include "zoomer.h"
#include "selector.h"
#include <Eigen/Core>
#include <string>

class Controller;

class OpenGLPanel3D : public QGLWidget
{
    Q_OBJECT
public:
    explicit OpenGLPanel3D(QWidget *parent = 0);
    void setController(Controller &c);
    void centerCamera();
    void saveScreenshot(const std::string &filename);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
    enum MouseAction { MA_NONE, MA_TRANSLATE, MA_ROTATE, MA_ZOOM, MA_ADDHANDLEFREE, MA_ADDHANDLEHEIGHT, MA_DELETEHANDLE, MA_DELETEFACE, MA_ADDPIN, MA_DELETEPIN, MA_ADDHANDLETOP, MA_ADDANCHOR, MA_DELETEANCHOR};

    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);

    void scaleMousePos(int x, int y, double &scaledx, double &scaledy) const;
    MouseAction deduceAction(QMouseEvent *event);


private:
    Controller *cont_;
    Camera c_;
    Translator translator_;
    Rotator rotator_;
    Zoomer zoomer_;
    Selector selector_;
    Eigen::Vector3d lightPos_;

    bool takeScreenshot_;
    std::string ssFilename_;

signals:

public slots:

};

#endif // OpenGLPanel3D_H
