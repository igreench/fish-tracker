#include "stereoimage.h"

StereoImage::StereoImage() {
}

void StereoImage::setLeft(Mat image) {
    this->left = image;
}

void StereoImage::setRight(Mat image) {
    this->right = image;
}

void StereoImage::setImages(Mat left, Mat right) {
    this->left = left;
    this->right = right;
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
