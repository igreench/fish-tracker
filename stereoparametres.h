#ifndef STEREOPARAMETRES_H
#define STEREOPARAMETRES_H

#include <opencv2/core/core.hpp>

#include <QObject>
#include <QString>

using namespace cv;

namespace stereo {

template<class T>
QString templateMatToString(Mat mat);
QString matToString(Mat mat);
Mat stringToMat(QString str);

class StereoParametres : public QObject {
    Q_OBJECT
public:
    StereoParametres();

    void setCameraMatrix1(Mat cameraMatrix1);
    void setCameraMatrix2(Mat cameraMatrix2);
    void setDistCoeffs1(Mat distCoeffs1);
    void setDistCoeffs2(Mat distCoeffs2);
    void setR(Mat R);
    void setT(Mat T);

    void setR1(Mat R1);
    void setR2(Mat R2);
    void setP1(Mat P1);
    void setP2(Mat P2);

    void setRMap1x(Mat rmap1x);
    void setRMap1y(Mat rmap1y);
    void setRMap2x(Mat rmap2x);
    void setRMap2y(Mat rmap2y);

    Mat getCameraMatrix1();
    Mat getCameraMatrix2();
    Mat getDistCoeffs1();
    Mat getDistCoeffs2();
    Mat getR();
    Mat getT();

    Mat getR1();
    Mat getR2();
    Mat getP1();
    Mat getP2();

    Mat getRMap1x();
    Mat getRMap1y();
    Mat getRMap2x();
    Mat getRMap2y();

    bool isEmpty();
    bool isEmptyRT();
    bool isEmptyRP();
    bool isEmptyRMap();

public slots:
    void print();

private:
    Mat cameraMatrix1;
    Mat cameraMatrix2;
    Mat distCoeffs1;
    Mat distCoeffs2;
    Mat R;
    Mat T;
    Mat R1;
    Mat R2;
    Mat P1;
    Mat P2;
    Mat rmap1x;
    Mat rmap1y;
    Mat rmap2x;
    Mat rmap2y;
};

}

#endif // STEREOPARAMETRES_H
