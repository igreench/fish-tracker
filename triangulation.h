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
    StereoImage *getBackground();
    void setIsBackgroundCalculated(bool flag);
    bool getIsBackgroundCalculated();
    void setIndexCurrentSavedStereoImage(int value);
    int getIndexCurrentSavedStereoImage();
    void setIndexCurrentStereoImage(int value);
    int getIndexCurrentStereoImage();
    int getIndexMaxSavedStereoImage();
    int getIndexMaxStereoImage();

private:
    std::vector < cv::Point3d > objects;

    StereoImage *background;
    bool isBackgroundCalculated;
    int indexCurrentSavedStereoImage;
    int indexCurrentStereoImage;
    int indexMaxSavedStereoImage;
    int indexMaxStereoImage;
};

#endif // TRIANGULATION_H
