#include "stereoimage.h"

#include <QDebug>

using namespace stereo;

StereoImage::StereoImage() {
}

StereoImage::StereoImage(Mat left, Mat right) {
    this->left = left;
    this->right = right;
}

void StereoImage::setLeft(Mat image) {
    if (left.data) {
        left.release();
    } else {
        qDebug() << "!!!!!!!!!!!!!!!!!!!!!!!!!!!1";
    }
    this->left = image;
}

void StereoImage::setRight(Mat image) {
    if (right.data) {
        right.release();
    } else {
        qDebug() << "!!!!!!!!!!!!!!!!!!!!!!!!!!!2";
    }
    this->right = image;
}

void StereoImage::setImages(Mat left, Mat right) {
    this->setLeft(left);
    this->setRight(right);
}

void StereoImage::setImages(StereoImage *stereoImage) {
    this->setImages(stereoImage->getLeft(), stereoImage->getRight());
}

Mat StereoImage::getLeft() {
    return left;
}

Mat StereoImage::getRight() {
    return right;
}

bool StereoImage::isEmpty() {
    return (!left.data || !right.data);
}

void StereoImage::clear() {
    left = Mat();
    right = Mat();
}

void StereoImage::release() {
    if (!left.empty()) {
        left.release();
    }
    if (!right.empty()) {
        right.release();
    }
}
