#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "controller.h"

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
    void setStatusBar(QString &str);
    double getWeightSliderValue();

    void setAlphaSliderValue(double value);
    double getAlphaSliderValue();
    void setBetaSliderValue(double value);
    double getBetaSliderValue();
    bool showReferenceMesh();
    bool showNetworkMesh();

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

protected:
    void moveEvent(QMoveEvent *);
    void resizeEvent(QResizeEvent *);
    void keyPressEvent(QKeyEvent *);

private:
    //void loadMesh(const char *filename);
    //void jitterMesh();
    //void subdivideMesh();

    Ui::MainWindow *ui;
    Controller c_;
};

#endif // MAINWINDOW_H
