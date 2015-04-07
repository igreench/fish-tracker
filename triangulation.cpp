#include "triangulation.h"

Triangulation::Triangulation() {
    blurWidth = 20;
    blurHeight = 20;
    thresh = 50;
    threshMaxval = 80;

    background = new StereoImage();
    isBackgroundCalculated = false;
    indexCurrentSavedStereoImage = 0;
    indexCurrentStereoImage = 0;
    indexMaxSavedStereoImage = 5;
    indexMaxStereoImage = 100;

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

StereoImage *Triangulation::getBackground() {
    return background;
}

void Triangulation::setIsBackgroundCalculated(bool flag) {
    this->isBackgroundCalculated = flag;
}

bool Triangulation::getIsBackgroundCalculated() {
    return isBackgroundCalculated;
}

void Triangulation::setIndexCurrentSavedStereoImage(int value) {
    this->indexCurrentSavedStereoImage = value;
}

int Triangulation::getIndexCurrentSavedStereoImage() {
    return indexCurrentSavedStereoImage;
}

void Triangulation::setIndexCurrentStereoImage(int value) {
    this->indexCurrentStereoImage = value;
}

int Triangulation::getIndexCurrentStereoImage() {
    return indexCurrentStereoImage;
}

int Triangulation::getIndexMaxSavedStereoImage() {
    return indexMaxSavedStereoImage;
}

int Triangulation::getIndexMaxStereoImage() {
    return indexMaxStereoImage;
}
