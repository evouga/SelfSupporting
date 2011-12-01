#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFileDialog>
#include <QMessageBox>
#include <QKeyEvent>
#include "newmeshdialog.h"
#include "networkthread.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow), c_(*this)
{
    ui->setupUi(this);
    ui->GLPanel3D->setController(c_);
    ui->GLPanel2D->setController(c_);
    c_.initialize();
    connect(c_.getNT(), SIGNAL(updateUI()), this, SLOT(updateUI()));
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

void MainWindow::on_actionLoad_Mesh_triggered()
{
    QString filename = QFileDialog::getOpenFileName(this, "Load Reference Mesh", ".", "SSS Files (*.sss)");
    if(!filename.isNull())
    {
        c_.loadMesh(filename.toStdString().c_str());
    }
}

void MainWindow::on_actionImport_OBJ_triggered()
{
    QString filename = QFileDialog::getOpenFileName(this, "Import Reference Geometry", ".", "OBJ Files (*.obj)");
    if(!filename.isNull())
    {
        c_.importOBJ(filename.toStdString().c_str());
    }
}

void MainWindow::on_actionSave_Mesh_triggered()
{
    QFileDialog savedialog(this, "Save Reference Mesh", ".", "SSS Files (*.sss)");
    savedialog.setFileMode(QFileDialog::AnyFile);
    savedialog.setDefaultSuffix("sss");
    savedialog.setViewMode(QFileDialog::List);
    savedialog.setAcceptMode(QFileDialog::AcceptSave);
    if(savedialog.exec())
    {
        QStringList filenames = savedialog.selectedFiles();
        if(filenames.size() > 0)
        {
            QString filename = filenames[0];
            c_.saveMesh(filename.toStdString().c_str());
        }
    }
}


void MainWindow::on_actionExport_OBJ_triggered()
{
    QFileDialog savedialog(this, "Export Reference Geometry", ".", "Mesh Files (*.obj)");
    savedialog.setFileMode(QFileDialog::AnyFile);
    savedialog.setDefaultSuffix("obj");
    savedialog.setViewMode(QFileDialog::List);
    savedialog.setAcceptMode(QFileDialog::AcceptSave);
    if(savedialog.exec())
    {
        QStringList filenames = savedialog.selectedFiles();
        if(filenames.size() > 0)
        {
            QString filename = filenames[0];
            c_.exportOBJ(filename.toStdString().c_str());
        }
    }
}


void MainWindow::on_actionExport_OM_triggered()
{
    QFileDialog savedialog(this, "Export Thrust Network Geometry", ".", "Mesh Files (*.om)");
    savedialog.setFileMode(QFileDialog::AnyFile);
    savedialog.setDefaultSuffix("om");
    savedialog.setViewMode(QFileDialog::List);
    savedialog.setAcceptMode(QFileDialog::AcceptSave);
    if(savedialog.exec())
    {
        QStringList filenames = savedialog.selectedFiles();
        if(filenames.size() > 0)
        {
            QString filename = filenames[0];
            c_.exportNetworkOBJ(filename.toStdString().c_str());
        }
    }
}

void MainWindow::on_actionExport_Network_triggered()
{
    QFileDialog savedialog(this, "Save Thrust Network", ".", "STN Files (*.stn)");
    savedialog.setFileMode(QFileDialog::AnyFile);
    savedialog.setDefaultSuffix("stn");
    savedialog.setViewMode(QFileDialog::List);
    savedialog.setAcceptMode(QFileDialog::AcceptSave);
    if(savedialog.exec())
    {
        QStringList filenames = savedialog.selectedFiles();
        if(filenames.size() > 0)
        {
            QString filename = filenames[0];
            c_.saveNetwork(filename.toStdString().c_str());
        }
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
    const Params &p = c_.getParams();
    ui->statusbar->showMessage(p.statusmsg);
    if(isinf(p.nmresidual))
        ui->residualEdit->setText("(unknown)");
    else
        ui->residualEdit->setText(QString::number(p.nmresidual));
    ui->GLPanel2D->updateGL();
    ui->GLPanel3D->updateGL();
    update();
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

bool MainWindow::showNetworkSurface()
{
    return ui->networkSurfaceCheckBox->isChecked();
}

bool MainWindow::showStressSurface()
{
    return ui->stressCheckBox->isChecked();
}

bool MainWindow::showConjugateVectors()
{
    return ui->conjugateVectorsCheckBox->isChecked();
}

void MainWindow::updateUI()
{
    updateGLWindows();
}

void MainWindow::save3DScreenshot(const std::string &filename)
{
    ui->GLPanel3D->saveScreenshot(filename);
    updateGLWindows();
}

Controller::EditMode MainWindow::getEditMode()
{
    if(ui->cameraButton->isChecked())
        return Controller::EM_CAMERA;
    else if(ui->freeformHandleButton->isChecked())
        return Controller::EM_FREEHANDLE;
    else if(ui->heightHandleButton->isChecked())
        return Controller::EM_HEIGHTHANDLE;
    else if(ui->deleteFaceButton->isChecked())
        return Controller::EM_DELETEFACE;
    else if(ui->pinButton->isChecked())
        return Controller::EM_PIN;
    else if(ui->creaseButton->isChecked())
        return Controller::EM_CREASE;
    assert(false);
}

void MainWindow::on_actionCenter_Cameras_triggered()
{
    centerCameras();
    updateGLWindows();
}

void MainWindow::on_fitCheckBox_stateChanged(int arg1)
{
    if(arg1 == Qt::Checked)
        c_.setAutoIterate(true);
    else
        c_.setAutoIterate(false);
}

void MainWindow::on_actionIterate_Fit_triggered()
{
    c_.iterateNetwork();
    updateGLWindows();
}

void MainWindow::on_actionProject_Onto_Reference_triggered()
{
    c_.projectNetwork();
}

void MainWindow::on_actionTake_Screenshot_triggered()
{
    c_.takeScreenshot();
}

void MainWindow::on_actionReset_triggered()
{
    c_.resetNetworkMesh();
}

void MainWindow::on_actionJitter_triggered()
{
    c_.jitterMesh();
}

void MainWindow::on_maxWeightCheckBox_stateChanged(int arg1)
{
    c_.enforceMaxWeight(arg1 == Qt::Checked);
    c_.resetNetworkMesh();
}

void MainWindow::on_maxWeightHorizontalSlider_valueChanged(int value)
{
    c_.setMaxWeight(value);
    c_.resetNetworkMesh();
}

void MainWindow::reportParams()
{
    c_.enforceMaxWeight(ui->maxWeightCheckBox->isChecked());
    c_.setMaxWeight(ui->maxWeightHorizontalSlider->value());
    c_.setDensity(ui->densitySlider->value());
}

void MainWindow::on_actionSubdivide_triggered()
{
    c_.subdivideMesh();
}

void MainWindow::on_actionCopy_Thrust_Network_triggered()
{
    c_.copyThrustNetwork();
}

void MainWindow::on_pushButton_clicked()
{
    c_.laplacianTest();
}

void MainWindow::on_actionSubdivideRM_triggered()
{
    c_.subdivideReferenceMesh();
}

void MainWindow::on_actionTriangulate_triggered()
{
    c_.triangulateThrustNetwork();
}

void MainWindow::on_actionMake_Planar_triggered()
{
    c_.planarizeThrustNetwork();
}

void MainWindow::on_densitySlider_valueChanged(int value)
{
    c_.setDensity(value);
    c_.resetNetworkMesh();
}


void MainWindow::on_conjugateVectorsCheckBox_clicked()
{
    c_.updateGLWindows();
}

void MainWindow::on_actionPin_Boundary_triggered()
{
    c_.pinReferenceBoundary();
}

void MainWindow::on_actionUnpin_Boundary_triggered()
{
    c_.unpinReferenceBoundary();
}
