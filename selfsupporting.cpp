#include "selfsupporting.h"
#include "ui_selfsupporting.h"
#include <iostream>

SelfSupporting::SelfSupporting(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::SelfSupporting)
{
    ui->setupUi(this);
    if(!mesh_.loadMesh("/home/etienne/plane.obj"))
    {
        //TODO error message
    }
    ui->GLPanel3D->setMesh(mesh_);
    ui->GLPanel2D->setMesh(mesh_);
}

SelfSupporting::~SelfSupporting()
{
    delete ui;
}

void SelfSupporting::moveEvent(QMoveEvent *)
{
    ui->GLPanel3D->updateGL();
    ui->GLPanel2D->updateGL();
}


void SelfSupporting::resizeEvent(QResizeEvent *)
{
    ui->GLPanel3D->updateGL();
    ui->GLPanel2D->updateGL();
}

