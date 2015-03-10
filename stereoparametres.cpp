#include "stereoparametres.h"

#include <QDebug>

using namespace stereo;

StereoParametres::StereoParametres()
{
}

void StereoParametres::setCameraMatrix1(Mat cameraMatrix1) {
    this->cameraMatrix1 = cameraMatrix1;
}

void StereoParametres::setCameraMatrix2(Mat cameraMatrix2) {
    this->cameraMatrix2 = cameraMatrix2;
}

void StereoParametres::setDistCoeffs1(Mat distCoeffs1) {
    this->distCoeffs1 = distCoeffs1;
}

void StereoParametres::setDistCoeffs2(Mat distCoeffs2) {
    this->distCoeffs2 = distCoeffs2;
}

void StereoParametres::setR(Mat R) {
    this->R = R;
}

void StereoParametres::setT(Mat T) {
    this->T = T;
}

void StereoParametres::setR1(Mat R1) {
    this->R1 = R1;
}

void StereoParametres::setR2(Mat R2) {
    this->R2 = R2;
}

void StereoParametres::setP1(Mat P1) {
    this->P1 = P1;
}

void StereoParametres::setP2(Mat P2) {
    this->P2 = P2;
}

void StereoParametres::setRMap1x(Mat rmap1x) {
    this->rmap1x = rmap1x;
}

void StereoParametres::setRMap1y(Mat rmap1y) {
    this->rmap1y = rmap1y;
}

void StereoParametres::setRMap2x(Mat rmap2x) {
    this->rmap2x = rmap2x;
}

void StereoParametres::setRMap2y(Mat rmap2y) {
    this->rmap2y = rmap2y;
}

Mat StereoParametres::getCameraMatrix1() {
    return cameraMatrix1;
}

Mat StereoParametres::getCameraMatrix2() {
    return cameraMatrix2;
}
Mat StereoParametres::getDistCoeffs1() {
    return distCoeffs1;
}

Mat StereoParametres::getDistCoeffs2() {
    return distCoeffs2;
}

Mat StereoParametres::getR() {
    return R;
}

Mat StereoParametres::getT() {
    return T;
}

Mat StereoParametres::getR1() {
    return R1;
}

Mat StereoParametres::getR2() {
    return R2;
}

Mat StereoParametres::getP1() {
    return P1;
}

Mat StereoParametres::getP2() {
    return P2;
}

Mat StereoParametres::getRMap1x() {
    return rmap1x;
}

Mat StereoParametres::getRMap1y() {
    return rmap1y;
}

Mat StereoParametres::getRMap2x() {
    return rmap2x;
}

Mat StereoParametres::getRMap2y() {
    return rmap2y;
}

void StereoParametres::print() {
    qDebug() << "cameraMatrix1" << matToString(cameraMatrix1);
    qDebug() << "cameraMatrix2" << matToString(cameraMatrix2);
    qDebug() << "distCoeffs1" << matToString(distCoeffs1);
    qDebug() << "distCoeffs2" << matToString(distCoeffs2);
    qDebug() << "R" << matToString(R);
    qDebug() << "T" << matToString(T);
}

bool StereoParametres::isEmpty() { //isEmptyIntrinsic
    return (cameraMatrix1.empty() ||
            cameraMatrix2.empty() ||
            distCoeffs1.empty() ||
            distCoeffs2.empty());
}

bool StereoParametres::isEmptyRT() { //isEmptyExternal
    return (R.empty() ||
            T.empty());
}

bool StereoParametres::isEmptyRP() {
    return (R1.empty() ||
            R2.empty() ||
            P1.empty() ||
            P2.empty());
}

bool StereoParametres::isEmptyRMap() {
    return (rmap1x.empty() ||
            rmap1y.empty() ||
            rmap2x.empty() ||
            rmap2y.empty());
}

template<class T>
QString stereo::templateMatToString(Mat mat) {
    QString str;
    str += QString("%1,%2;").arg(mat.rows).arg(mat.cols);
    for(int i = 0; i < mat.rows; i++) {
        for(int j = 0; j < mat.cols; j++) {
            str += QString("%1").arg(mat.at<T>(i, j));
            if (j + 1 < mat.cols) {
               str += QString(",");
            }
        }
        str += QString(";");
    }
    return str;
}

QString stereo::matToString(Mat mat) {
    if (mat.empty()) {
        return "";
    }
    switch (mat.type()) {
    case CV_32F :
        return stereo::templateMatToString<float>(mat);
        break;
    case CV_64F :
        return stereo::templateMatToString<double>(mat);
        break;
    case CV_8U :
        return stereo::templateMatToString<unsigned char>(mat);
        break;
    default:
        qDebug() << "Error: unknowing type\n";
        return "";
    }
}

cv::Mat stereo::stringToMat(QString str)
{
    if(str.isEmpty())
    {
        return cv::Mat();
    }
    str.remove(" ");
    QStringList data = str.split(";",QString::SkipEmptyParts);
    QString sizes = data.first();
    bool isok;
    QStringList sizes_list = sizes.split(",",QString::SkipEmptyParts);
    if(2!=sizes_list.size())
    {
        return cv::Mat();
    }


    cv::Mat A(sizes_list[0].toInt(&isok),sizes_list[1].toInt(&isok),CV_64FC1,cv::Scalar::all(0));

    data.removeFirst();
    for(int i=0;i<data.size();i++)
    {
        QStringList row_list = data[i].split(",",QString::SkipEmptyParts);
        for(int j=0;j<row_list.size();j++)
        {

           double x = row_list[j].toDouble(&isok);
           if(!isok)
           {
               return cv::Mat();
           }
           A.at<double>(i,j) = x;
        }
    }
    return A;
}
