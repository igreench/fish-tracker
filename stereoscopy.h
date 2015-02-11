#ifndef STEREOSCOPY_H
#define STEREOSCOPY_H

#include "FlyCapture2.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "opencv2/contrib/contrib.hpp"

#include <QString>

using namespace FlyCapture2;
using namespace std;
using namespace cv;

template<class T>
QString templateMatToString(Mat mat);
QString matToString(Mat mat);

class Stereoscopy {
public:
    Stereoscopy();

    void startCapture();
    void endCapture();
    void loopCapture();

    void checkProjectPoints(string fn1, string fn2);

    void check();
    std::vector<cv::Point2d> Generate2DPoints();
    std::vector<cv::Point3d> Generate3DPoints();

private:
    Error error;
    Camera camera;
    Camera camera2;

    bool isStereoCalibrated;
    bool isShowing;
    //bool isStereoRectified = false;
    Mat scres;// stereo callibrate result with points
    Mat R, T, Q; //
    Mat rmap1, rmap2;

    StereoSGBM sgbm;

    bool addImage(const Mat im, vector<Point2f> *imageCorners, Mat &result);

    vector<vector<Point3f> > objectPoints;
    vector<vector<Point2f> > imagePoints1;
    vector<vector<Point2f> > imagePoints2;

    static const int BOARD_WIDTH = 8;
    static const int BOARD_HEIGHT = 9;

};

#endif // STEREOSCOPY_H
