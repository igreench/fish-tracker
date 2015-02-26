#include "camera3d.h"

#include <QDebug>

Camera3D::Camera3D() {
    stereoImage = new StereoImage();
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(update()));
}

void Camera3D::startCapture() {
    //need to handle errors and exceptions
    camera1 = new PGRCamera(0);
    camera2 = new PGRCamera(1);
    camera1->startCapture();
    camera2->startCapture();
    timer->start(3000); //30
}

void Camera3D::stopCapture() {
    timer->stop();
    camera1->stopCapture();
    camera2->stopCapture();
}

void Camera3D::update() {
    qDebug() << "Camera3D::update()";
    stereoImage->setLeft(camera1->getFrame());
    stereoImage->setRight(camera2->getFrame());
}
