#ifndef MAINWINDOWCONTROLLER_H
#define MAINWINDOWCONTROLLER_H

#include "mainwindow.h"

class MainWindowController
{
public:
    MainWindowController(MainWindow &mw);

    void repaintGLPanels();
    void recomputeWeights();

private:
    MainWindow &mw_;
};

#endif // MAINWINDOWCONTROLLER_H
