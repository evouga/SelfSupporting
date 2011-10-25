#include "newmeshdialog.h"
#include "ui_newmeshdialog.h"
#include "controller.h"
#include <QIntValidator>
#include <iostream>

NewMeshDialog::NewMeshDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::NewMeshDialog)
{
    ui->setupUi(this);
    ui->horizEdit->setValidator(new QIntValidator(ui->horizEdit));
    ui->vertEdit->setValidator(new QIntValidator(ui->vertEdit));
}

NewMeshDialog::~NewMeshDialog()
{
    delete ui;
}

void NewMeshDialog::buildNewMesh(Controller &c)
{
    int w = std::max(2,ui->horizEdit->text().toInt());
    int h = std::max(2,ui->vertEdit->text().toInt());
    switch(ui->TypeBox->currentIndex())
    {
    case 0:
    {
        c.buildTriMesh(w,h);
        break;
    }
    case 1:
    {
        c.buildQuadMesh(w,h);
        break;
    }
    default:
        break;
    }
}
