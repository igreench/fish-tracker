#include "triangulation.h"

#include <opencv2/imgproc/imgproc.hpp>

#include <QDebug>

Triangulation::Triangulation() {
    blurWidth = 20;
    blurHeight = 20;
    thresh = 50;
    threshMaxval = 80;

    background = new StereoImage();
    isBackgroundCalculated = false;
    indexCurrentStereoImage = 0;
    indexMaxStereoImage = 5;

    mode = 2;
}

Triangulation::~Triangulation() {
}

void Triangulation::setObjects(std::vector < cv::Point3d > objects) {
    this->objects = objects;
}

std::vector < cv::Point3d > Triangulation::getObjects() {
    return objects;
}

void Triangulation::setBackground(StereoImage *si) {
    this->background->setImages(si->getLeft().clone(), si->getRight().clone()); //?
}

StereoImage *Triangulation::getBackground() {
    return background;
}

void Triangulation::setIsBackgroundCalculated(bool flag) {
    this->isBackgroundCalculated = flag;
}

bool Triangulation::getIsBackgroundCalculated() {
    return isBackgroundCalculated;
}

void Triangulation::setIndexCurrentStereoImage(int value) {
    this->indexCurrentStereoImage = value;
}

int Triangulation::getIndexCurrentStereoImage() {
    return indexCurrentStereoImage;
}

int Triangulation::getIndexMaxStereoImage() {
    return indexMaxStereoImage;
}

void Triangulation::addStereoImage(StereoImage *si) {
    stereoImages.push_back(si);
}

void Triangulation::calculateBackground() {

    //Calculation and testing background

    qDebug() << "start calc bg";

    vector < Mat > pics1;
    vector < Mat > pics2;

    for (int i = 0; i < stereoImages.size(); i++) {
        Mat grey1, grey2;
        cvtColor(stereoImages[i]->getLeft(), grey1, CV_BGR2GRAY);
        cvtColor(stereoImages[i]->getRight(), grey2, CV_BGR2GRAY);
        pics1.push_back(grey1);
        pics2.push_back(grey2);
    }

    qDebug() << "added imgs";

    Q_ASSERT(pics1.size() == pics2.size());

    int w = 1600;
    int h = 1200;

    Mat bg1(h, w, CV_8UC1);
    Mat bg2(h, w, CV_8UC1);

    qDebug() << "bg1.cols: " << bg1.cols;
    qDebug() << "bg1.rows: " << bg1.rows;
    qDebug() << "pics1[0].cols: " << pics1[0].cols;
    qDebug() << "pics1[0].rows: " << pics1[0].rows;

    for (int i = 0; i < h; i++) { //rows
        for (int j = 0; j < w; j++) { //cols
            long long int s1 = 1; //0
            long long int s2 = 1; //0

            QVector < int > m1;
            QVector < int > m2;
            for (int k = 0; k < pics1.size(); k++) {
                s1 *= pics1[k].at<uchar>(i, j);
                s2 *= pics2[k].at<uchar>(i, j);

                m1.push_back(pics1[k].at<uchar>(i, j));
                m2.push_back(pics2[k].at<uchar>(i, j));
                //qDebug() << "pics1[k].at<uchar>(i, j): " << pics1[k].at<uchar>(i, j);
            }

            qSort(m1);
            qSort(m2);

            bg1.at<uchar>(i, j) = m1[m1.size() / 2];
            bg2.at<uchar>(i, j) = m2[m2.size() / 2];
        }
    }

    qDebug() << "calced";

    background->setImages(bg1, bg2);
}
