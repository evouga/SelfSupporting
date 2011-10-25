#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFileDialog>
#include <QMessageBox>
#include <QKeyEvent>
#include "newmeshdialog.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow), c_(*this)
{
    ui->setupUi(this);
    ui->GLPanel3D->setController(c_);
    ui->GLPanel2D->setController(c_);
    c_.initialize();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::centerCameras()
{
    ui->GLPanel2D->centerCamera();
    ui->GLPanel3D->centerCamera();
}

double MainWindow::getWeightSliderValue()
{
    return ui->weightSumSlider->value();
}

double MainWindow::getBetaSliderValue()
{
    return 0.1*ui->betaSlider->value();
}

double MainWindow::getAlphaSliderValue()
{
    return 0.01*ui->alphaSlider->value();
}

void MainWindow::setAlphaSliderValue(double value)
{
    ui->alphaSlider->setValue(100*value);
}

void MainWindow::setBetaSliderValue(double value)
{
    ui->betaSlider->setValue(10*value);
}

void MainWindow::on_actionLoad_Mesh_triggered()
{
    QString filename = QFileDialog::getOpenFileName(this, "Load Mesh", ".", "Mesh Files (*.obj)");
    if(!filename.isNull())
    {
        c_.loadMesh(filename.toStdString().c_str());
    }
}

void MainWindow::moveEvent(QMoveEvent *)
{
}

void MainWindow::resizeEvent(QResizeEvent *)
{
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    if(event->key() == 'q' || event->key() == 'Q')
        close();
}

void MainWindow::updateGLWindows()
{
    ui->GLPanel2D->updateGL();
    ui->GLPanel3D->updateGL();
}

void MainWindow::setStatusBar(QString &str)
{
    ui->statusbar->showMessage(str);
}

void MainWindow::on_jitterButton_clicked()
{
    c_.jitterMesh();
}

void MainWindow::on_weightSumSlider_valueChanged(int )
{
}

void MainWindow::on_actionNew_Mesh_triggered()
{
    NewMeshDialog *nmd = new NewMeshDialog();
    if(nmd->exec())
    {
        nmd->buildNewMesh(c_);
    }
    delete nmd;
}

void MainWindow::on_computeHeightButton_clicked()
{
    c_.computeWeightsFromTopView();
}

void MainWindow::on_projectButton_clicked()
{
    c_.projectNetwork();
}

void MainWindow::on_subdivideButton_clicked()
{
    c_.subdivideMesh();
}

void MainWindow::on_referenceCheckBox_clicked()
{
    c_.updateGLWindows();
}

void MainWindow::on_networkCheckBox_clicked()
{
    c_.updateGLWindows();
}

bool MainWindow::showReferenceMesh()
{
    return ui->referenceCheckBox->isChecked();
}

bool MainWindow::showNetworkMesh()
{
    return ui->networkCheckBox->isChecked();
}

void MainWindow::on_resetButton_clicked()
{
     c_.resetNetworkMesh();
}

void MainWindow::on_iterateButton_clicked()
{
    c_.iterateNetwork();
}
