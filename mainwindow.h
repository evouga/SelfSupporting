#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QApplication>
#include "controller.h"
#include <string>

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void updateGLWindows();
    void centerCameras();

    bool showReferenceMesh();
    bool showNetworkMesh();
    bool showNetworkSurface();
    bool showStressSurface();
    bool showConjugateVectors();

    void save3DScreenshot(const std::string &filename);

    void reportParams();

    Controller::EditMode getEditMode();

private slots:

    void on_actionLoad_Mesh_triggered();

    void on_weightSumSlider_valueChanged(int value);

    void on_actionNew_Mesh_triggered();

    void on_computeHeightButton_clicked();

    void on_referenceCheckBox_clicked();

    void on_networkCheckBox_clicked();

    void updateUI();

    void on_actionCenter_Cameras_triggered();

    void on_fitCheckBox_stateChanged(int arg1);

    void on_actionIterate_Fit_triggered();

    void on_actionProject_Onto_Reference_triggered();

    void on_actionTake_Screenshot_triggered();

    void on_actionReset_triggered();

    void on_actionJitter_triggered();

    void on_maxWeightCheckBox_stateChanged(int arg1);

    void on_actionSubdivide_triggered();

    void on_actionSave_Mesh_triggered();

    void on_actionCopy_Thrust_Network_triggered();

    void on_pushButton_clicked();

    void on_actionSubdivideRM_triggered();

    void on_actionTriangulate_triggered();

    void on_actionExport_OBJ_triggered();

    void on_actionImport_OBJ_triggered();

    void on_actionMake_Planar_triggered();

    void on_actionExport_OM_triggered();

    void on_actionExport_Network_triggered();

    void on_conjugateVectorsCheckBox_clicked();

    void on_actionPin_Boundary_triggered();

    void on_actionUnpin_Boundary_triggered();

    void on_actionSwap_Y_and_Z_triggered();

    void on_actionInvert_Y_triggered();

    void on_networkSurfaceCheckBox_clicked();

    void on_stressCheckBox_clicked();

    void on_actionWeights_triggered();

    void on_actionPositions_triggered();

    void on_actionAdd_Mesh_triggered();

    void on_actionHeights_triggered();

    void on_avgHeightsButton_clicked();

    void on_enforcePlanarityCheckBox_clicked();

    void on_actionTrim_triggered();

    void on_actionCompute_Conjugate_Dirs_triggered();

    void on_actionSmooth_Boundary_triggered();

    void on_actionSmooth_Boundary_2_triggered();

    void on_actionExport_Everything_triggered();

    void on_actionExport_Weights_triggered();

    void on_maxStressEdit_editingFinished();

    void on_densityEdit_editingFinished();

    void on_thicknessEdit_editingFinished();

    void on_dilateButton_clicked();

protected:
    void moveEvent(QMoveEvent *);
    void resizeEvent(QResizeEvent *);
    void keyPressEvent(QKeyEvent *);

private:
    Ui::MainWindow *ui;
    Controller c_;
};

#endif // MAINWINDOW_H
