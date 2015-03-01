#ifndef STEREOPROCESSING_H
#define STEREOPROCESSING_H

#include <opencv2/core/core.hpp>

#include "stereoimage.h"
#include "stereoparametres.h"

using namespace cv;

namespace stereo {

class StereoProcessing
{
public:
    StereoProcessing();

    void setStereoImage(StereoImage* stereoImage);
    void setStereoParametres(StereoParametres* stereoParametres);

    StereoImage* getStereoImage();
    StereoParametres* getStereoParametres();

    StereoImage* undistortStereoImage();
    void triangulate();
    void drawCirclesPattern();

private:
    StereoImage* stereoImage;
    StereoParametres* stereoParametres;
};

}

#endif // STEREOPROCESSING_H
