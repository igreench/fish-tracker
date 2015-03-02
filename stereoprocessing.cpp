#include "stereoprocessing.h"

#include <opencv/cv.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "opencv2/contrib/contrib.hpp"

#include <QDebug>

using namespace stereo;
using namespace std;
using namespace cv;

StereoProcessing::StereoProcessing()
{
}
/*
void StereoProcessing::setStereoImage(StereoImage* stereoImage) {
    this->stereoImage = stereoImage;
}

void StereoProcessing::setStereoParametres(StereoParametres* stereoParametres) {
    this->stereoParametres = stereoParametres;
}

StereoImage* StereoProcessing::getStereoImage() {
    return stereoImage;
}

StereoParametres* StereoProcessing::getStereoParametres() {
    return stereoParametres;
}
*/

StereoImage* StereoProcessing::undistortStereoImage(StereoImage* stereoImage, StereoParametres* stereoParametres) {
    StereoImage* undistortStereoImage = new StereoImage();
    Mat left, right;
    //qDebug() << "start undistort";
    //stereoParametres->print();
    undistort(stereoImage->getLeft(), left, stereoParametres->getCameraMatrix1(), stereoParametres->getDistCoeffs1());
    undistort(stereoImage->getRight(), right, stereoParametres->getCameraMatrix2(), stereoParametres->getDistCoeffs2());
    undistortStereoImage->setImages(left, right);
    //qDebug() << "end undistort";
    //undistortStereoImage->setImages(stereoImage->getLeft(), stereoImage->getRight());
    return undistortStereoImage;
}

void StereoProcessing::triangulate(StereoImage* stereoImage, StereoParametres* stereoParametres) {
    Mat image1 = stereoImage->getLeft();
    Mat image2 = stereoImage->getRight();

    Mat imageSmall = Mat(image1.rows / 4, image1.cols / 4, CV_8UC3);
    Size imageSizeSmall = imageSmall.size(); //400x300

    /*Mat cameraMatrix1 = (Mat_<double>(3,3) << 1806.53,0,815.786,0,1595.14,590.314,0,0,1);
    Mat distCoeffs1 = (Mat_<double>(1,5) << -0.267514,0.213748,0.00136627,0.000194796,0);
    Mat cameraMatrix2 = (Mat_<double>(3,3) << 1739.3,0,808.929,0,1542.11,581.767,0,0,1);
    Mat distCoeffs2 = (Mat_<double>(1,5) << -0.247249,0.161344,-0.00280154,0.000444185,0);*/

    Mat cameraMatrix1 = stereoParametres->getCameraMatrix1();
    Mat distCoeffs1 = stereoParametres->getDistCoeffs1();
    Mat cameraMatrix2 = stereoParametres->getCameraMatrix2();
    Mat distCoeffs2 = stereoParametres->getDistCoeffs2();

    //Start

    bool isShowImages = true;
    bool isShowUImages = true;
    bool isShowTImages = true;
    bool isShowDImages = true;
    bool isShowCImages = true;

    Mat rimage1, rimage2;

    if (isShowImages) {
        cv::resize(image1, rimage1, imageSizeSmall, 0, 0, CV_INTER_LINEAR);
        cv::resize(image2, rimage2, imageSizeSmall, 0, 0, CV_INTER_LINEAR);
        imshow("image1", rimage1);
        imshow("image2", rimage2);
    }

    //Calibration

    Mat uimage1, uimage2;

    undistort(image1, uimage1, cameraMatrix1, distCoeffs1);
    undistort(image2, uimage2, cameraMatrix2, distCoeffs2);

    if (isShowUImages) {
        resize(uimage1, rimage1, imageSizeSmall, 0, 0, CV_INTER_LINEAR);
        resize(uimage2, rimage2, imageSizeSmall, 0, 0, CV_INTER_LINEAR);
        imshow("uimage1", rimage1);
        imshow("uimage2", rimage2);
    }

    //Binarization
    Mat g1, g2;

    cvtColor(uimage1, g1, CV_BGR2GRAY);
    cvtColor(uimage2, g2, CV_BGR2GRAY);

    Mat timage1, timage2;

    blur(g1, g1, Size(20, 20));
    blur(g2, g2, Size(20, 20));

    //It would be cool combinate threshold and adaptiveThreshold

    threshold(g1, timage1, 50, 80, CV_THRESH_BINARY_INV); //50 250
    threshold(g2, timage2, 50, 80, CV_THRESH_BINARY_INV);

    if (isShowTImages) {
        resize(timage1, rimage1, imageSizeSmall, 0, 0, CV_INTER_LINEAR);
        resize(timage2, rimage2, imageSizeSmall, 0, 0, CV_INTER_LINEAR);
        imshow("timage1", rimage1);
        imshow("timage2", rimage2);
    }

    //Finding contours

    vector<vector<Point> > contours1, contours2;
    vector<Vec4i> hierarchy1, hierarchy2;
    RNG rng(12345);

    //findContours( timage1, contours1, hierarchy1, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    findContours( timage1, contours1, hierarchy1, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE, Point(0, 0) );
    Mat drawing1 = Mat::zeros( timage1.size(), CV_8UC3 );
    for( int i = 0; i < contours1.size(); i++ ) {
        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        drawContours( drawing1, contours1, i, color, 2, 8, hierarchy1, 0, Point() );
    }

    findContours( timage2, contours2, hierarchy2, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    Mat drawing2 = Mat::zeros( timage2.size(), CV_8UC3 );
    for( int i = 0; i < contours2.size(); i++ ) {
        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        drawContours( drawing2, contours2, i, color, 2, 8, hierarchy2, 0, Point() );
    }

    if (isShowDImages) {
        resize(drawing1, rimage1, imageSizeSmall, 0, 0, CV_INTER_LINEAR);
        imshow("drawing1", rimage1);
        resize(drawing2, rimage2, imageSizeSmall, 0, 0, CV_INTER_LINEAR);
        imshow("drawing2", rimage2);
    }

    //Getting moments and mass centers
    vector<Moments> mu1(contours1.size());
    vector<Point2f> mc1(contours1.size());
    vector<Moments> mu2(contours2.size());
    vector<Point2f> mc2(contours2.size());
    for(int i = 0; i < contours1.size(); i++) {
        mu1[i] = moments(contours1[i], false);
        mc1[i] = Point2f(mu1[i].m10/mu1[i].m00 , mu1[i].m01/mu1[i].m00);
        Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
        circle(drawing1, mc1[i], 4, color, -1, 8, 0);
    }
    for(int i = 0; i < contours2.size(); i++) {
        mu2[i] = moments(contours2[i], false);
        mc2[i] = Point2f(mu2[i].m10/mu2[i].m00 , mu2[i].m01/mu2[i].m00);
        Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
        circle(drawing2, mc2[i], 4, color, -1, 8, 0);
    }

    if (isShowCImages) {
        resize(drawing1, rimage1, imageSizeSmall, 0, 0, CV_INTER_LINEAR);
        imshow("cdrawing1", rimage1);
        resize(drawing2, rimage2, imageSizeSmall, 0, 0, CV_INTER_LINEAR);
        imshow("cdrawing2", rimage2);
    }

    undistortPoints(mc1, mc1, cameraMatrix1, distCoeffs1);
    undistortPoints(mc2, mc2, cameraMatrix2, distCoeffs2);

    //Triangulation
    /*Mat R = (Mat_<double>(3,3) << 0.991532,-0.0276931,-0.126874,0.0602062,0.963683,0.260173,0.115061,-0.265608,0.95719);
    Mat T = (Mat_<double>(3,1) << 8.33789,-17.5109,83.1614);*/
    Mat R = stereoParametres->getR();
    Mat T = stereoParametres->getT();
    Mat points4D;
    Mat RT;
    hconcat(R, T, RT);
    Mat cam1 = cameraMatrix1 * RT;
    Mat cam2 = cameraMatrix2 * RT;
    triangulatePoints(cam1, cam2, mc1, mc2, points4D);
    qDebug() << "points4D" << matToString(points4D);

    double w = points4D.at<double>(3,0);
    double x = points4D.at<double>(0,0)/w;
    double y = points4D.at<double>(1,0)/w;
    double z = points4D.at<double>(2,0)/w;
    qDebug() << x << ", " << y << ", " << z;
}

void StereoProcessing::drawCirclesPattern() {
    Mat image(1792, 1600, CV_8UC3);
    //Mat image(400, 400, CV_8UC3);
    image.setTo(Scalar(255, 255, 255));
    int n = 9;
    int m = 8;
    int r = 64;
    //int r = 10;
    for (int i = 0; i < (n + 1) * m; i++) {
        circle(image, Point((i % m) * (3 * r) + 2 * r, (i / n) * (3 * r) + 2 * r), r, Scalar(0, 0, 0), -1);
    }
    imwrite( "pattern.jpg", image );
    qDebug() << "cols: " << image.cols << " rows: " << image.rows;
    resize(image, image, Size(image.cols / 4, image.rows / 4), 0, 0, CV_INTER_LINEAR);
    qDebug() << "cols: " << image.cols << " rows: " << image.rows;
    imshow("image", image);
}

bool StereoProcessing::addImage(const Mat im, vector<Point2f> *imageCorners, Mat &result) {
    Size pattern_size = Size(BOARD_WIDTH, BOARD_HEIGHT);//!< Размер шахматной доски
    Size imsize; //!< Размер изображение вычисляется в addImage

    qDebug() << "addImage";
    if(im.empty()) {
        qDebug() << "Empty image exit";
        return false;
    }

    Mat grey;
    imsize = Size(im.rows, im.cols);

    if (1 != im.channels()) {
        cvtColor(im, grey, CV_BGR2GRAY);
        result = im.clone();
    } else {
        grey = im.clone();
        cvtColor(grey, result, CV_GRAY2BGR);
    }

    imsize = Size(grey.cols,grey.rows);
    try {
        qDebug() << "findChessboardCorners";
        Mat small_gray;
        small_gray = grey;
        bool found = false;
        result = grey;
        found = findCirclesGrid(small_gray, pattern_size, *imageCorners, CALIB_CB_SYMMETRIC_GRID);
        if (!found) {
            qDebug() << "Not found";
            return false;
        }

        qDebug() << "found";
        drawChessboardCorners(result, pattern_size, *imageCorners, true);
        return true;
    } catch(...) {
        qDebug() << "Exception";
    }


    return false;
}
