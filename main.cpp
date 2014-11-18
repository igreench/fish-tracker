#include "mainwindow.h"
#include <QApplication>

#include "FlyCapture2.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>

#include <QDebug>

using namespace FlyCapture2;

int main() {
    qDebug() << "Start program";
    Error error;
    Camera camera;
    CameraInfo camInfo;

    // Connect the camera
    error = camera.Connect(0);
    if (error != PGRERROR_OK) {
        qDebug() << "Failed to connect to camera" ;
        return -1;
    }

    // Get the camera info and print it out
    error = camera.GetCameraInfo(&camInfo);
    if (error != PGRERROR_OK) {
        qDebug() << "Failed to get camera info from camera";
        return -1;
    }
    qDebug() << camInfo.vendorName << " "
              << camInfo.modelName << " "
              << camInfo.serialNumber;

    error = camera.StartCapture();
    if (error == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED) {
        qDebug() << "Bandwidth exceeded";
        return -1;
    } else {
        if (error != PGRERROR_OK) {
            qDebug() << "Failed to start image capture";
            return -1;
        }
    }

    // Capture loop
    qDebug() << "Start capture";
    char key = 0;
    while (key != 'q') {
        // Get the image
        Image rawImage;
        Error error = camera.RetrieveBuffer(&rawImage);
        if (error != PGRERROR_OK) {
            qDebug() << "Capture error";
            continue;
        }

        // convert to rgb
        Image rgbImage;
        rawImage.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage);

        // convert to OpenCV Mat
        unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize() / (double)rgbImage.GetRows();
        cv::Mat image = cv::Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(),rowBytes);

        cv::imshow("image", image);
        key = cv::waitKey(30);
    }

    qDebug() << "Stop capture";
    error = camera.StopCapture();
    if (error != PGRERROR_OK) {
        // This may fail when the camera was removed, so don't show
        // an error message
    }

    camera.Disconnect();

    return 0;
}
