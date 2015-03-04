#ifndef STEREOPROCESSING_H
#define STEREOPROCESSING_H

#include <opencv2/core/core.hpp>

#include "stereoimage.h"
#include "stereoparametres.h"

using namespace cv;
using namespace std;

namespace stereo {

class StereoProcessing {
public:
    //Is it utility class?
    StereoProcessing();
    StereoImage* undistortStereoImage(StereoImage* stereoImage, StereoParametres* stereoParametres);
    Mat projectPoints(StereoImage* stereoImage, StereoParametres* stereoParametres);
    StereoImage* undistortRectify(StereoImage* stereoImage, StereoParametres* stereoParametres);
    Mat disparityMap(StereoImage* stereoImage, StereoParametres* stereoParametres);
    StereoImage* triangulate(StereoImage* stereoImage, StereoParametres* stereoParametres);
    Mat circlesPattern();

    bool addImage(const Mat im, vector<Point2f> *imageCorners, Mat &result);

private:
    static const int BOARD_WIDTH = 9; //8
    static const int BOARD_HEIGHT = 8; //9
};

}

#endif // STEREOPROCESSING_H
