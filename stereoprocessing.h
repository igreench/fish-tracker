#ifndef STEREOPROCESSING_H
#define STEREOPROCESSING_H

#include <opencv2/core/core.hpp>

#include "stereoimage.h"
#include "stereoparametres.h"

using namespace cv;
using namespace std;

namespace stereo {

struct Description {
    std::string source;//!< Source name
    cv::Mat A;//!< Intristics
    cv::Mat d;//!< Distorsion coefficents
    cv::Mat R;//!< Cam rotation matrix
    cv::Mat t;//!< Tramslation vector
    std::vector<cv::Point3d> points;//!< Undistort point on image plane (u,v,1)
    int cols;//!< Number of cols in the source image
    int rows;//!< Number of rows int the source image
};

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

    void calculateRT(StereoImage *stereoImage, StereoParametres* stereoParametres);
    void calculateRT2(StereoImage* stereoImage, StereoParametres* stereoParametres);
    void calculateRP(StereoImage* stereoImage, StereoParametres* stereoParametres);
    void calculateRP2(StereoImage* stereoImage, StereoParametres* stereoParametres);
    void calculateRMap(StereoImage* stereoImage, StereoParametres* stereoParametres);

    std::vector<cv::Point3d> intersect(Description* a,Description* b);

private:
    static const int BOARD_WIDTH = 9; //8
    static const int BOARD_HEIGHT = 8; //9

    Description descriptionLeft;
    Description descriptionRight;
};

}

#endif // STEREOPROCESSING_H
