#ifndef STEREOPARAMETRES_H
#define STEREOPARAMETRES_H

#include <opencv2/core/core.hpp>

#include <QString>

using namespace cv;

namespace stereo {

template<class T>
QString templateMatToString(Mat mat);
QString matToString(Mat mat);
Mat stringToMat(QString str);

class StereoParametres {
public:
    StereoParametres();

    void setCameraMatrix1(Mat cameraMatrix1);
    void setCameraMatrix2(Mat cameraMatrix2);
    void setDistCoeffs1(Mat distCoeffs1);
    void setDistCoeffs2(Mat distCoeffs2);
    void setR(Mat R);
    void setT(Mat T);

    Mat getCameraMatrix1();
    Mat getCameraMatrix2();
    Mat getDistCoeffs1();
    Mat getDistCoeffs2();
    Mat getR();
    Mat getT();

    bool isEmpty();
    void print();

private:
    Mat cameraMatrix1;
    Mat cameraMatrix2;
    Mat distCoeffs1;
    Mat distCoeffs2;
    Mat R;
    Mat T;
};

}

#endif // STEREOPARAMETRES_H
