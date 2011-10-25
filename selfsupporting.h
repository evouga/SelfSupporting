#ifndef SELFSUPPORTING_H
#define SELFSUPPORTING_H

#include "mesh.h"

#include <QWidget>

namespace Ui {
    class SelfSupporting;
}

class SelfSupporting : public QWidget
{
    Q_OBJECT

public:
    explicit SelfSupporting(QWidget *parent = 0);
    ~SelfSupporting();

protected:
    void moveEvent(QMoveEvent *);
    void resizeEvent(QResizeEvent *);

private:
    Ui::SelfSupporting *ui;
    Mesh mesh_;
};

#endif // SELFSUPPORTING_H
