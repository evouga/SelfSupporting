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

    Controller::EditMode getEditMode();

private slots:

    void on_actionLoad_Mesh_triggered();

    void on_jitterButton_clicked();

    void on_weightSumSlider_valueChanged(int value);

    void on_actionNew_Mesh_triggered();

    void on_computeHeightButton_clicked();

    void on_projectButton_clicked();

    void on_subdivideButton_clicked();

    void on_referenceCheckBox_clicked();

    void on_networkCheckBox_clicked();

    void on_resetButton_clicked();

    void on_iterateButton_clicked();

    void updateUI();

    void on_screenshotButton_clicked();

    void on_actionCenter_Cameras_triggered();

protected:
    void moveEvent(QMoveEvent *);
    void resizeEvent(QResizeEvent *);
    void keyPressEvent(QKeyEvent *);

private:
    Ui::MainWindow *ui;
    Controller c_;
};

#endif // MAINWINDOW_H
