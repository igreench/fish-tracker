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

        //try to undistort
        cv::Mat rmap[2];
        cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << 1898.86,0,801.607,0,1684.22,533.037,0,0,1);
        //3,3;1898.86,0,801.607;0,1684.22,533.037;0,0,1;
        cv::Mat distCoeffs = (cv::Mat_<double>(1,5) << -0.138546,-0.290375,0.00790141,0.00869189,0);
        //1,5;-0.138546,-0.290375,0.00790141,0.00869189,0;
        cv::Mat R1, P1, rimage;
        cv::Size imageSize = image.size();
        //R1 = cameraMatrix.inv()*H1*cameraMatrix;//
        //P1 = cameraMatrix;//
        cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, R1, P1, imageSize, CV_16SC2, rmap[0], rmap[1]);
        //cv::initUndistortRectifyMap(intrinsic, distortion, R, P, CV_8UC3 mapx, mapy);
        cv::remap(image, rimage, rmap[0], rmap[1], CV_INTER_LINEAR);

        cv::imshow("image1", rimage);
        //

        key = cv::waitKey(30);
    }

    qDebug() << "Stop capture";
    error = camera.StopCapture();
    if (error != PGRERROR_OK) {
        // This may fail when the camera was removed, so don't show
        // an error message
    }

    //intristics 3,3;1977,0,934.867;0,1812.95,904.821;0,0,1;
    //distCoeffs 1,5;-0.0833973,0.151344,0.0536747,0.045857,0;


    camera.Disconnect();

    return 0;
}
