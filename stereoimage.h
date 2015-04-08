#ifndef STEREOIMAGE_H
#define STEREOIMAGE_H

#include <opencv2/core/core.hpp>

using namespace cv;

namespace stereo {

class StereoImage
{
public:
    StereoImage();
    StereoImage(Mat left, Mat right);

    void setLeft(Mat image);
    void setRight(Mat image);
    void setImages(Mat left, Mat right);
    void setImages(StereoImage *stereImage);

    bool isEmpty();
    void clear();

    Mat getLeft();
    Mat getRight();

private:
    Mat left;
    Mat right;
};

}

#endif // STEREOIMAGE_H
