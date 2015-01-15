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
    Camera camera2;
    PGRGuid guid, guid2;
    BusManager busMgr;
    CameraInfo camInfo;

    // Connect the camera
    error = busMgr.GetCameraFromIndex(0, &guid);
    error = camera.Connect(&guid);
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

    // Connect the camera

    error = busMgr.GetCameraFromIndex(1, &guid2);
    error = camera2.Connect(&guid2);
    if (error != PGRERROR_OK) {
        qDebug() << "Failed to connect to camera" ;
        return -1;
    }

    // Get the camera info and print it out
    error = camera2.GetCameraInfo(&camInfo);
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

    error = camera2.StartCapture();
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

        // camera1: get the image
        Image rawImage1;
        Error error = camera.RetrieveBuffer(&rawImage1);
        if (error != PGRERROR_OK) {
            qDebug() << "Capture error";
            continue;
        }

        // camera1: convert to rgb
        Image rgbImage1;
        rawImage1.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage1);

        // camera1: convert to OpenCV Mat
        unsigned int rowBytes = (double)rgbImage1.GetReceivedDataSize() / (double)rgbImage1.GetRows();
        cv::Mat image1_1 = cv::Mat(rgbImage1.GetRows(), rgbImage1.GetCols(), CV_8UC3, rgbImage1.GetData(), rowBytes);
        cv::Mat image1_2 = cv::Mat(rgbImage1.GetRows() / 4, rgbImage1.GetCols() / 4, CV_8UC3);


        // camera1: undistortion
        cv::Mat rmap[2];
        cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << 1898.86,0,801.607,0,1684.22,533.037,0,0,1);
        cv::Mat distCoeffs = (cv::Mat_<double>(1,5) << -0.138546,-0.290375,0.00790141,0.00869189,0);
        cv::Mat R1, P1, rimage1;
        cv::Size imageSize1 = image1_1.size();
        cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, R1, P1, imageSize1, CV_16SC2, rmap[0], rmap[1]);
        cv::remap(image1_1, rimage1, rmap[0], rmap[1], CV_INTER_LINEAR);

        // camera1: resize
        cv::resize(rimage1, image1_1, image1_2.size(), 0, 0, CV_INTER_LINEAR);

        // camera1: show
        cv::imshow("image1", image1_1);

        // camera2: get the image
        Image rawImage2;
        error = camera2.RetrieveBuffer(&rawImage2);
        if (error != PGRERROR_OK) {
            qDebug() << "Capture error";
            continue;
        }

        // camera2: convert to rgb
        Image rgbImage2;
        rawImage2.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage2);

        // camera2: convert to OpenCV Mat
        rowBytes = (double)rgbImage2.GetReceivedDataSize() / (double)rgbImage2.GetRows();
        cv::Mat image2_1 = cv::Mat(rgbImage2.GetRows(), rgbImage2.GetCols(), CV_8UC3, rgbImage2.GetData(),rowBytes);
        cv::Mat image2_2 = cv::Mat(rgbImage2.GetRows() / 4, rgbImage1.GetCols() / 4, CV_8UC3);
        cv::resize(image2_1, image2_2, image2_2.size(), 0, 0, CV_INTER_LINEAR);

        // camera2: undistortion
        cv::Mat cameraMatrix2 = (cv::Mat_<double>(3,3) << 1898.86,0,801.607,0,1684.22,533.037,0,0,1);
        cv::Mat distCoeffs2 = (cv::Mat_<double>(1,5) << -0.138546,-0.290375,0.00790141,0.00869189,0);
        cv::Mat R2, P2, rimage2;
        cv::Size imageSize2 = image2_1.size();
        cv::initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, imageSize2, CV_16SC2, rmap[0], rmap[1]);
        cv::remap(image2_1, rimage2, rmap[0], rmap[1], CV_INTER_LINEAR);

        // camera2: resize
        cv::resize(rimage2, image2_1, image2_2.size(), 0, 0, CV_INTER_LINEAR);

        // camera2: show
        cv::imshow("image2", image2_1);

        // recify
        /*cv::Mat R, T, Q;
        cv::Rect validRoi[2];
        cv::stereoRectify(cameraMatrix, distCoeffs,
                          cameraMatrix2, distCoeffs2,
                          imageSize, R, T, R1, R2, P1, P2, Q,
                          CV_CALIB_ZERO_DISPARITY, 1,
                          imageSize,
                          &validRoi[0],
                            &validRoi[1]);

        //*/

        // wait key
        key = cv::waitKey(30);
    }

    qDebug() << "Stop capture";
    error = camera.StopCapture();
    if (error != PGRERROR_OK) {
        // This may fail when the camera was removed, so don't show
        // an error message
    }
    error = camera2.StopCapture();
    if (error != PGRERROR_OK) {
        // This may fail when the camera was removed, so don't show
        // an error message
    }

    //intristics 3,3;1977,0,934.867;0,1812.95,904.821;0,0,1;
    //distCoeffs 1,5;-0.0833973,0.151344,0.0536747,0.045857,0;


    camera.Disconnect();
    camera2.Disconnect();

    return 0;
}
