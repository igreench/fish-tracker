#ifndef STEREOIMAGE_H
#define STEREOIMAGE_H

#include <opencv2/core/core.hpp>

using namespace cv;

class StereoImage
{
public:
    StereoImage();

    void setLeftImage(Mat image);
    void setRightImage(Mat image);
    void setImages(Mat left, Mat right);

private:
    Mat left;
    Mat right;
};

#endif // STEREOIMAGE_H
