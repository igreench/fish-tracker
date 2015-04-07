#ifndef TRIANGULATION_H
#define TRIANGULATION_H

#include <opencv2/core/core.hpp>
#include "stereoimage.h"

using namespace stereo;

class Triangulation
{
public:
    Triangulation();
    ~Triangulation();

    void setObjects(std::vector < cv::Point3d > objects);
    std::vector < cv::Point3d > getObjects();

    int blurWidth;
    int blurHeight;
    int thresh;
    int threshMaxval;

    int mode;

    void setBackground(StereoImage *si);
    bool getBackground();

    void setIsBackgroundCalculated(bool flag);
    bool getIsBackgroundCalculated();

private:
    std::vector < cv::Point3d > objects;
    bool isBackgroundCalculated;
    StereoImage *background;
};

#endif // TRIANGULATION_H
