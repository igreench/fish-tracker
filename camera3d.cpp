#include "camera3d.h"

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
}

StereoImage Camera3D::getStereoImage() {
    return *stereoImage;
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
    //qDebug() << "Camera3D::update()";
    stereoImage->setLeft(camera1->getFrame());
    stereoImage->setRight(camera2->getFrame());
}
