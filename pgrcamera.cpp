#include "pgrcamera.h"

#include <QDebug>

PGRCamera::PGRCamera(int id) {
    this->id = id;
    isConnect = false;
}

bool PGRCamera::isConnected() {
    return isConnect;
}

void PGRCamera::startCapture() {
    qDebug() << "Start program";

    PGRGuid guid;
    BusManager busMgr;
    CameraInfo camInfo;

    // Connect the camera
    error = busMgr.GetCameraFromIndex(id, &guid);
    error = camera.Connect(&guid);
    if (error != PGRERROR_OK) {
        qDebug() << "Failed to connect to camera" ;
        return ; //exception
    }

    // Get the camera info and print it out
    error = camera.GetCameraInfo(&camInfo);
    if (error != PGRERROR_OK) {
        qDebug() << "Failed to get camera info from camera";
        return ; //exception
    }
    qDebug() << camInfo.vendorName << " "
              << camInfo.modelName << " "
              << camInfo.serialNumber;

    error = camera.StartCapture();
    if (error == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED) {
        qDebug() << "Bandwidth exceeded";
        return ; //exception
    } else {
        if (error != PGRERROR_OK) {
            qDebug() << "Failed to start image capture";
            return ; //exception
        }
    }
    isConnect = true;
}

void PGRCamera::stopCapture() {
    isConnect = false;
    qDebug() << "Stop capture";
    error = camera.StopCapture();
    if (error != PGRERROR_OK) {
        // This may fail when the camera was removed, so don't show
        // an error message
    }

    camera.Disconnect();
}

Mat PGRCamera::getFrame() {
    Image rawImage;
    Error error = camera.RetrieveBuffer(&rawImage);
    if (error != PGRERROR_OK) {
        qDebug() << "Capture error";
        return Mat();
        //continue;
    }
    Image rgbImage;
    rawImage.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage);
    unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize() / (double)rgbImage.GetRows();
    return Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(), rowBytes).clone();
}
