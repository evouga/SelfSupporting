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

    void on_maxWeightHorizontalSlider_valueChanged(int value);

    void on_actionSubdivide_triggered();

    void on_actionSave_Mesh_triggered();

    void on_actionCopy_Thrust_Network_triggered();

    void on_pushButton_clicked();

    void on_actionSubdivideRM_triggered();

    void on_actionTriangulate_triggered();

    void on_actionExport_OBJ_triggered();

    void on_actionImport_OBJ_triggered();

protected:
    void moveEvent(QMoveEvent *);
    void resizeEvent(QResizeEvent *);
    void keyPressEvent(QKeyEvent *);

private:
    Ui::MainWindow *ui;
    Controller c_;
};

#endif // MAINWINDOW_H
