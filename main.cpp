#include "mainwindow.h"
#include "stereoscopy.h"
#include <QApplication>

#include <QDebug>

int main(int argc, char *argv[])
{    
    Stereoscopy *stereoscopy = new Stereoscopy();
    /*stereoscopy->startCapture();
    stereoscopy->loopCapture();
    stereoscopy->endCapture();*/
    //stereoscopy->checkProjectPoints("image1_1.jpg", "image2_1.jpg");
    //stereoscopy->checkDisparityMap("image1_a.jpg", "image2_a.jpg");
    //stereoscopy->checkDisparityMap2("image1_a.jpg", "image2_a.jpg");
    stereoscopy->checkDisparityMap2("image1_b.jpg", "image2_b.jpg");

    return 0;

    /*QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();*/
}
