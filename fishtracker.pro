#-------------------------------------------------
#
# Project created by QtCreator 2014-04-27T05:27:58
#
#-------------------------------------------------

QT       += core gui opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = fishtracker
TEMPLATE = app

INCLUDEPATH += "C:/Program Files (x86)/Point Grey Research/FlyCapture2/include"
LIBS += -L"C:/Program Files (x86)/Point Grey Research/FlyCapture2/lib" \
    -lFlyCapture2

INCLUDEPATH += C:/OpenCV/opencv/build/include
LIBS += -LC:/OpenCV/opencv/build/x86/vc10/lib \
    -lopencv_core2410 \
    -lopencv_highgui2410 \
    -lopencv_imgproc2410 \
    -lopencv_calib3d2410 \
    -lopencv_features2d2410

SOURCES += main.cpp\
        mainwindow.cpp \
    pgrcamera.cpp \
    camera3d.cpp \
    stereoimage.cpp \
    iodata.cpp \
    stereoprocessing.cpp \
    stereoparametres.cpp \
    disparitymap.cpp \
    glwidget.cpp \
    triangulation.cpp

HEADERS  += mainwindow.h \
    pgrcamera.h \
    camera3d.h \
    stereoimage.h \
    iodata.h \
    asmopencv.h \
    stereoprocessing.h \
    stereoparametres.h \
    disparitymap.h \
    glwidget.h \
    triangulation.h

FORMS    += mainwindow.ui \
    glwidget.ui
