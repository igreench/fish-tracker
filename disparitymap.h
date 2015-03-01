#ifndef DISPARITYMAP_H
#define DISPARITYMAP_H

#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;

class DisparityMap
{
public:
    DisparityMap();

    StereoSGBM sgbm;
};

#endif // DISPARITYMAP_H
