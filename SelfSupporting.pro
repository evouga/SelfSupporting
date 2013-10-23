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
    stressmesh.h \
    stressmeshrenderer.h

FORMS    += \
    mainwindow.ui \
    newmeshdialog.ui

unix:!macx:!symbian: LIBS += -lOpenMeshCore -L/usr/lib/coin

INCLUDEPATH += ../ext/openmesh ../ext/eigen
QMAKE_LIBDIR += ../ext/openmesh/build/Build/lib/OpenMesh

INCLUDEPATH += ../bcls-0.1/include

RESOURCES +=

QMAKE_CXXFLAGS += -g -msse2 -isystem /usr/include/coin

unix:!macx:!symbian: LIBS += -lbcls -lpng -lGL -lGLU -lcblas

