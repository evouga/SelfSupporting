#ifndef OPENGLPANEL2D_H
#define OPENGLPANEL2D_H

#include <QGLWidget>
#include <Eigen/Core>

class Controller;

class OpenGLPanel2D : public QGLWidget
{
    Q_OBJECT
public:
    explicit OpenGLPanel2D(QWidget *parent = 0);

    void setController(Controller &c);
    void centerCamera();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:

    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();

    void mousePressEvent(QMouseEvent *);
    void mouseMoveEvent(QMouseEvent *);
    void mouseReleaseEvent(QMouseEvent *);


    void scaleMousePos(int x, int y, double &sx, double &sy);

signals:

public slots:

private:
    Controller *cont_;
    double scale_;
    int width_, height_;
    int draggingidx_;
    Eigen::Vector2d dragPos_;
};

#endif // OPENGLPANEL2D_H
