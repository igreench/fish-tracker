#include "camera3d.h"
#include "imagedata.h"

#include <opencv2/imgproc/imgproc.hpp>

#include <QDebug>

Camera3D::Camera3D() {
    stereoImage = new StereoImage();
    isConnect = false;

    camera1 = new PGRCamera(1);
    camera2 = new PGRCamera(0);
    camera1->startCapture();
    camera2->startCapture();
    if (camera1->isConnected() && camera2->isConnected()) {
        isConnect = true;
    }

    SIZE_W = 400;
    SIZE_H = 300;
}

StereoImage *Camera3D::getStereoImage() {
    return stereoImage;
}

bool Camera3D::isConnected() {
    return isConnect;
}

void Camera3D::startCapture() {
    //need to handle errors and exceptions        
    //timer->start(3000); //30
}

void Camera3D::stopCapture() {
    //timer->stop();
    camera1->stopCapture();
    camera2->stopCapture();
}

void Camera3D::update() {
    ImageData *leftData = camera1->getFrame();
    ImageData *rightData = camera2->getFrame();
    Mat left, right;
    resize(Mat(leftData->getRows(), leftData->getCols(), leftData->getType(), leftData->getData(), leftData->getStep()),
           left, Size(SIZE_W, SIZE_H), 0, 0, CV_INTER_LINEAR);
    resize(Mat(rightData->getRows(), rightData->getCols(), rightData->getType(), rightData->getData(), rightData->getStep()),
           right, Size(SIZE_W, SIZE_H), 0, 0, CV_INTER_LINEAR);
    stereoImage->setLeft(left);
    stereoImage->setRight(right);
}
