#-------------------------------------------------
#
# Project created by QtCreator 2011-10-07T04:22:50
#
#-------------------------------------------------

QT       += core gui opengl

TARGET = SelfSupporting
TEMPLATE = app


SOURCES += main.cpp\
    camera.cpp \
    translator.cpp \
    rotator.cpp \
    mesh.cpp \
    zoomer.cpp \
    openglpanel3d.cpp \
    openglpanel2d.cpp \
    mainwindow.cpp \
    newmeshdialog.cpp \
    selector.cpp \
    controller.cpp \
    solvers.cpp \
    vectorpool.cpp \
    meshrenderer.cpp \
    referencemesh.cpp \
    networkmesh.cpp \
    networkmeshrenderer.cpp \
    referencemeshrenderer.cpp \
    networkthread.cpp \
    YImage.cpp \
    stressmesh.cpp \
    stressmeshrenderer.cpp

HEADERS  += \
    eiquadprog.hpp \
    camera.h \
    translator.h \
    rotator.h \
    mesh.h \
    zoomer.h \
    openglpanel3d.h \
    openglpanel2d.h \
    mainwindow.h \
    newmeshdialog.h \
    selector.h \
    controller.h \
    solvers.h \
    vectorpool.h \
    meshrenderer.h \
    referencemesh.h \
    networkmesh.h \
    networkmeshrenderer.h \
    referencemeshrenderer.h \
    networkthread.h \
    YImage.hpp \
    function.h \
    stressmesh.h \
    stressmeshrenderer.h

FORMS    += \
    mainwindow.ui \
    newmeshdialog.ui

unix:!macx:!symbian: LIBS += -L$$PWD/../../OpenMesh-2.0.1/build/Build/lib/OpenMesh/ -lOpenMeshCore

INCLUDEPATH += $$PWD/../../OpenMesh-2.0.1/src \
    $$PWD/../../eigen
DEPENDPATH += $$PWD/../../OpenMesh-2.0.1/src

DEFINES += EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET

RESOURCES +=

QMAKE_CXXFLAGS += -g







































unix:!macx:!symbian: LIBS += -lbcls

unix|win32: LIBS += -lcblas































