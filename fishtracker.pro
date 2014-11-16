#-------------------------------------------------
#
# Project created by QtCreator 2014-04-27T05:27:58
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = fishtracker
TEMPLATE = app

INCLUDEPATH += C:/NV/libs/opencv-2.4/install/w32/vc10/shared/debug/include
LIBS += -LC:/NV/libs/opencv-2.4/install/w32/vc10/shared/debug/lib \
    -lopencv_core245d \
    -lopencv_highgui245d \
    -lopencv_imgproc245d

SOURCES += main.cpp\
        mainwindow.cpp

HEADERS  += mainwindow.h

FORMS    += mainwindow.ui
