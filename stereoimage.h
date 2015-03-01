#ifndef STEREOIMAGE_H
#define STEREOIMAGE_H

#include <opencv2/core/core.hpp>

using namespace cv;

class StereoImage
{
public:
    StereoImage();

    void setLeft(Mat image);
    void setRight(Mat image);
    void setImages(Mat left, Mat right);

    bool isEmpty();
    void clear();

    Mat getLeft();
    Mat getRight();

private:
    Mat left;
    Mat right;
};

#endif // STEREOIMAGE_H
