#include "triangulation.h"

Triangulation::Triangulation() {
    blurWidth = 20;
    blurHeight = 20;
    thresh = 50;
    threshMaxval = 80;

    background = new StereoImage();
    isBackgroundCalculated = false;

    mode = 2;
}

Triangulation::~Triangulation() {
}

void Triangulation::setObjects(std::vector < cv::Point3d > objects) {
    this->objects = objects;
}

std::vector < cv::Point3d > Triangulation::getObjects() {
    return objects;
}

void Triangulation::setBackground(StereoImage *si) {
    this->background->setImages(si->getLeft().clone(), si->getRight().clone()); //?
}

bool Triangulation::getBackground() {
    return background;
}

void Triangulation::setIsBackgroundCalculated(bool flag) {
    this->isBackgroundCalculated = flag;
}

bool Triangulation::getIsBackgroundCalculated() {
    return isBackgroundCalculated;
}
