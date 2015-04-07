#ifndef TRIANGULATION_H
#define TRIANGULATION_H

#include <opencv2/core/core.hpp>
#include "stereoimage.h"

using namespace stereo;
using namespace std;
using namespace cv;

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

    //Does translate it to class StereoBackground? =)
    void setBackground(StereoImage *si);
    StereoImage *getBackground();
    void setIsBackgroundCalculated(bool flag);
    bool getIsBackgroundCalculated();
    void setIndexCurrentStereoImage(int value);
    int getIndexCurrentStereoImage();
    int getIndexMaxStereoImage();
    void addStereoImage(StereoImage *si);
    void calculateBackground();

private:
    std::vector < cv::Point3d > objects;

    StereoImage *background;
    bool isBackgroundCalculated;
    int indexCurrentStereoImage;
    int indexMaxStereoImage;
    vector <StereoImage *> stereoImages;
};

#endif // TRIANGULATION_H
