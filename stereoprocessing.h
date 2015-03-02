#ifndef STEREOPROCESSING_H
#define STEREOPROCESSING_H

#include <opencv2/core/core.hpp>

#include "stereoimage.h"
#include "stereoparametres.h"

using namespace cv;

namespace stereo {

class StereoProcessing {
public:
    StereoProcessing();

    StereoImage* undistortStereoImage(StereoImage* stereoImage, StereoParametres* stereoParametres);
    void triangulate(StereoImage* stereoImage, StereoParametres* stereoParametres);
    void drawCirclesPattern();
};

}

#endif // STEREOPROCESSING_H
