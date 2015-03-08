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

void StereoParametres::print() {
    qDebug() << "cameraMatrix1" << matToString(cameraMatrix1);
    qDebug() << "cameraMatrix2" << matToString(cameraMatrix2);
    qDebug() << "distCoeffs1" << matToString(distCoeffs1);
    qDebug() << "distCoeffs2" << matToString(distCoeffs2);
    qDebug() << "R" << matToString(R);
    qDebug() << "T" << matToString(T);
}

bool StereoParametres::isEmpty() {
    return (cameraMatrix1.empty() ||
            cameraMatrix2.empty() ||
            distCoeffs1.empty() ||
            distCoeffs2.empty());
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
