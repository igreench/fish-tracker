#include "mainwindow.h"
#include <QApplication>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv/cv.h"

#include <QDebug>

char waitkey;

int main(int argc, char *argv[]) {
    Q_UNUSED(argc)
    Q_UNUSED(argv)

    /*QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();*/

    qDebug() << "CV";


    cvNamedWindow("Camera_Output", 1);
    CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY);
    while(1) {
        IplImage* frame = cvQueryFrame(capture);

        cvShowImage("Camera_Output", frame);

        waitkey = cvWaitKey(10);
        if (char(waitkey) == 27){
            break;
        }
    }
    cvReleaseCapture(&capture);
    cvDestroyWindow("Camera_Output");

    return 0;
}
