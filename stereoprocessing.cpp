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
    Mat image1 = stereoImage->getLeft().clone();
    Mat image2 = stereoImage->getRight().clone();
    StereoImage* undistortStereoImage = new StereoImage();
    Mat left, right;
    //qDebug() << "start undistort";
    //stereoParametres->print();
    undistort(image1, left, stereoParametres->getCameraMatrix1(), stereoParametres->getDistCoeffs1());
    undistort(image2, right, stereoParametres->getCameraMatrix2(), stereoParametres->getDistCoeffs2());
    undistortStereoImage->setImages(left, right);
    //qDebug() << "end undistort";
    //undistortStereoImage->setImages(stereoImage->getLeft(), stereoImage->getRight());
    return undistortStereoImage;
}

Mat StereoProcessing::projectPoints(StereoImage* stereoImage, StereoParametres* stereoParametres) {
    Mat image1 = stereoImage->getLeft().clone();
    Mat image2 = stereoImage->getRight().clone();

    Mat cameraMatrix1 = stereoParametres->getCameraMatrix1();
    Mat distCoeffs1 = stereoParametres->getDistCoeffs1();
    Mat cameraMatrix2 = stereoParametres->getCameraMatrix2();
    Mat distCoeffs2 = stereoParametres->getDistCoeffs2();

    vector<Point2f> corners1, corners2;

    Mat scres;

    if (addImage(image1, &corners1, scres) && addImage(image2, &corners2, scres)) {

        vector<vector<Point3f> > objectPoints;
        vector<vector<Point2f> > imagePoints1;
        vector<vector<Point2f> > imagePoints2;

        imagePoints1.push_back(corners1);
        imagePoints2.push_back(corners2);

        int n = BOARD_WIDTH * BOARD_HEIGHT;
        vector<Point3f> obj;
        for (int j = 0; j < n; j++) {
            obj.push_back(Point3f(j % BOARD_WIDTH, j / BOARD_WIDTH, 0.0f));
        }
        objectPoints.push_back(obj);

        /*qDebug() << "imagePoints1 size:" << imagePoints1.size();
        qDebug() << "imagePoints2 size:" << imagePoints2.size();
        qDebug() << "corners1 size:" << corners1.size();
        qDebug() << "corners2 size:" << corners2.size();
        qDebug() << "objectPoints size:" << objectPoints.size();
        qDebug() << "obj size:" << obj.size();*/

        qDebug() << "Done creation objectPoints";

        vector<Point2f> imagePoints;

        vector<Mat> rvecs1, tvecs1;
        vector<Mat> rvecs2, tvecs2;
        calibrateCamera(objectPoints, imagePoints1, image1.size(), cameraMatrix1, distCoeffs1, rvecs1, tvecs1);
        calibrateCamera(objectPoints, imagePoints2, image2.size(), cameraMatrix2, distCoeffs2, rvecs2, tvecs2);

        qDebug() << "cameraMatrix1: " << matToString(cameraMatrix1);
        qDebug() << "distCoeffs1: " << matToString(distCoeffs1);

        cv::projectPoints(Mat(obj), rvecs1[0], tvecs1[0], cameraMatrix1, distCoeffs1, imagePoints);

        /*Size pattern_size = Size(BOARD_WIDTH, BOARD_HEIGHT);
        drawChessboardCorners(image1_1, pattern_size, imagePoints1, true);*/

        for(unsigned int i = 0; i < imagePoints.size(); ++i) {
            circle(image1, Point2d(imagePoints[i].x, imagePoints[i].y), 20, Scalar( 255, 0, 0 ), CV_FILLED, 8, 0);
            //qDebug() << "x:" << imagePoints[i].x << "y:" << imagePoints[i].y;
        }
        return image1;
    }
    qDebug() << "StereoProcessing::projectPoints: didn't find calibration desks";
    return stereoImage->getLeft();
}

StereoImage* StereoProcessing::undistortRectify(StereoImage* stereoImage, StereoParametres* stereoParametres) {
    Mat image1 = stereoImage->getLeft().clone();
    Mat image2 = stereoImage->getRight().clone();
    Mat cameraMatrix1 = stereoParametres->getCameraMatrix1();
    Mat distCoeffs1 = stereoParametres->getDistCoeffs1();
    Mat cameraMatrix2 = stereoParametres->getCameraMatrix2();
    Mat distCoeffs2 = stereoParametres->getDistCoeffs2();
    Mat R = stereoParametres->getR();
    Mat T = stereoParametres->getT();
    Mat R1 = stereoParametres->getR1();
    Mat R2 = stereoParametres->getR2();
    Mat P1 = stereoParametres->getP1();
    Mat P2 = stereoParametres->getP2();
    Mat rmap1x = stereoParametres->getRMap1x();
    Mat rmap1y = stereoParametres->getRMap1y();
    Mat rmap2x = stereoParametres->getRMap2x();
    Mat rmap2y = stereoParametres->getRMap2y();
    StereoImage* undistortRectifyStereoImage = new StereoImage();
    undistortRectifyStereoImage->setImages(image1, image2);

    //BUG! If stereoParametres have RP but haven't RT then programm will throw error. It corrects with bool.
    if (stereoParametres->isEmptyRT()) {
        qDebug() << "Error: isEmptyRT";
        return undistortRectifyStereoImage;
    }

    if (stereoParametres->isEmptyRP()) {
        qDebug() << "Exception: isEmptyRP";
        Mat Q;
        stereoRectify(cameraMatrix1,
                      distCoeffs1,
                      cameraMatrix2,
                      distCoeffs2,
                      image1.size(),
                      R, T, R1, R2, P1, P2, Q);
    }

    if (stereoParametres->isEmptyRMap()) {
        qDebug() << "Exception: isEmptyRMap";
        initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, image1.size(), CV_32FC1, rmap1x, rmap1y);
        initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, image2.size(), CV_32FC1, rmap2x, rmap2y);
    }

    remap(image1, image1, rmap1x, rmap1y, CV_INTER_LINEAR);
    remap(image2, image2, rmap2x, rmap2y, CV_INTER_LINEAR);

    undistortRectifyStereoImage->setImages(image1, image2);
    return undistortRectifyStereoImage;
}

void StereoProcessing::calculateRT(StereoImage* stereoImage, StereoParametres* stereoParametres) {
    Mat image1 = stereoImage->getLeft().clone();
    Mat image2 = stereoImage->getRight().clone();
    Mat R, T, E, F;

    Mat cameraMatrix1 = stereoParametres->getCameraMatrix1();
    Mat distCoeffs1 = stereoParametres->getDistCoeffs1();
    Mat cameraMatrix2 = stereoParametres->getCameraMatrix2();
    Mat distCoeffs2 = stereoParametres->getDistCoeffs2();

    Mat objectPoints, imagePoints1, imagePoints2;

    Mat R1, R2, T1, T2, r1, r2, tr1, tr2;
    stereoCalibrate(objectPoints, imagePoints1, imagePoints2,
                    cameraMatrix1,
                    distCoeffs1,
                    cameraMatrix2,
                    distCoeffs2,
                    image1.size(), R, T, E, F/*,
                                                    TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 1e-6),
                                                    CV_CALIB_FIX_ASPECT_RATIO*/);

    qDebug() << "R: " << matToString(R);
    qDebug() << "T: " << matToString(T);
}

void StereoProcessing::calculateRT2(StereoImage* stereoImage, StereoParametres* stereoParametres) {
    Mat image1 = stereoImage->getLeft().clone();
    Mat image2 = stereoImage->getRight().clone();

    Mat cameraMatrix1 = stereoParametres->getCameraMatrix1();
    Mat distCoeffs1 = stereoParametres->getDistCoeffs1();
    Mat cameraMatrix2 = stereoParametres->getCameraMatrix2();
    Mat distCoeffs2 = stereoParametres->getDistCoeffs2();

    vector<Mat> rvecs1, tvecs1;
    vector<Mat> rvecs2, tvecs2;

    Mat objectPoints, imagePoints1, imagePoints2;
    calibrateCamera(objectPoints, imagePoints1, image1.size(), cameraMatrix1, distCoeffs1, rvecs1, tvecs1);
    calibrateCamera(objectPoints, imagePoints2, image2.size(), cameraMatrix2, distCoeffs2, rvecs2, tvecs2);

    qDebug() << "start calculating R and T";

    Mat R1, R2, T1, T2, r1, r2, tr1, tr2;

    qDebug() << "rvecs1: " << matToString(Mat(rvecs1[0]));
    qDebug() << "tvecs1: " << matToString(Mat(tvecs1[0]));
    qDebug() << "rvecs2: " << matToString(Mat(rvecs2[0]));
    qDebug() << "tvecs2: " << matToString(Mat(tvecs2[0]));

    Rodrigues(Mat(rvecs1[0]), r1);
    Rodrigues(Mat(rvecs2[0]), r2);
    qDebug() << "r1: " << matToString(r1);
    qDebug() << "r2: " << matToString(r2);

    transpose(r1, tr1);
    transpose(r2, tr2);
    qDebug() << "tr1: " << matToString(tr1);
    qDebug() << "tr2: " << matToString(tr2);

    R1 = r1 * tr2;
    T1 = -R1 * Mat(tvecs2[0]) + Mat(tvecs1[0]);
    qDebug() << "R1: " << matToString(R1);
    qDebug() << "T1: " << matToString(T1);

    R2 = r2 * tr1;
    T2 = -R2 * Mat(tvecs1[0]) + Mat(tvecs2[0]);
    qDebug() << "R2: " << matToString(R2);
    qDebug() << "T2: " << matToString(T2);

    qDebug() << "end calculating R and T";

    //end of calculating
}

void StereoProcessing::calculateRP(StereoImage* stereoImage, StereoParametres* stereoParametres) {
    if (stereoParametres->isEmptyRT()) {
        qDebug() << "calculateRP: Error: isEmptyRT";
        return;
    }

    Mat image1 = stereoImage->getLeft().clone();
    Mat image2 = stereoImage->getRight().clone();

    Mat R, T, R1, R2, P1, P2, Q;

    Mat cameraMatrix1 = stereoParametres->getCameraMatrix1();
    Mat distCoeffs1 = stereoParametres->getDistCoeffs1();
    Mat cameraMatrix2 = stereoParametres->getCameraMatrix2();
    Mat distCoeffs2 = stereoParametres->getDistCoeffs2();

    stereoRectify(cameraMatrix1,
                  distCoeffs1,
                  cameraMatrix2,
                  distCoeffs2,
                  image1.size(),
                  R, T, R1, R2, P1, P2, Q);
}

void StereoProcessing::calculateRP2(StereoImage* stereoImage, StereoParametres* stereoParametres) {
    if (stereoParametres->isEmptyRT()) {
        qDebug() << "calculateRP2: Error: isEmptyRT";
        return;
    }

    Mat image1 = stereoImage->getLeft().clone();
    Mat image2 = stereoImage->getRight().clone();

    Mat cameraMatrix1 = stereoParametres->getCameraMatrix1();
    Mat distCoeffs1 = stereoParametres->getDistCoeffs1();
    Mat cameraMatrix2 = stereoParametres->getCameraMatrix2();
    Mat distCoeffs2 = stereoParametres->getDistCoeffs2();

    vector<vector<Point3f> > objectPoints;
    vector<vector<Point2f> > imagePoints1;
    vector<vector<Point2f> > imagePoints2;

    // use intrinsic parameters of each camera, but
    // compute the rectification transformation directly
    // from the fundamental matrix

    Mat F, R1, R2, P1, P2;

    vector<Point2f> allimgpt[2];
    std::copy(imagePoints1[0].begin(), imagePoints1[0].end(), back_inserter(allimgpt[0]));
    std::copy(imagePoints2[0].begin(), imagePoints2[0].end(), back_inserter(allimgpt[1]));
    F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
    Mat H1, H2;
    stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F, image1.size(), H1, H2, 3);
    R1 = cameraMatrix1.inv() * H1 * cameraMatrix1;
    R2 = cameraMatrix2.inv() * H2 * cameraMatrix2;
    P1 = cameraMatrix1;
    P2 = cameraMatrix2;

}

void StereoProcessing::calculateRMap(StereoImage* stereoImage, StereoParametres* stereoParametres) {
    if (stereoParametres->isEmptyRP()) {
        qDebug() << "calculateRMap: Error: isEmptyRP";
        return;
    }

    Mat image1 = stereoImage->getLeft().clone();
    Mat image2 = stereoImage->getRight().clone();

    Mat cameraMatrix1 = stereoParametres->getCameraMatrix1();
    Mat distCoeffs1 = stereoParametres->getDistCoeffs1();
    Mat cameraMatrix2 = stereoParametres->getCameraMatrix2();
    Mat distCoeffs2 = stereoParametres->getDistCoeffs2();

    Mat R1, R2, P1, P2;

    Mat rmap1x, rmap1y, rmap2x, rmap2y;

    initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, image1.size(), CV_32FC1, rmap1x, rmap1y);
    initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, image2.size(), CV_32FC1, rmap2x, rmap2y);
}

Mat StereoProcessing::disparityMap(StereoImage* stereoImage, StereoParametres* stereoParametres) {
    //TODO
    //use undistortRectify
    return stereoImage->getLeft();
}

StereoImage* StereoProcessing::triangulate(StereoImage* stereoImage, StereoParametres* stereoParametres) {
    Mat image1 = stereoImage->getLeft().clone();
    Mat image2 = stereoImage->getRight().clone();

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

    bool isShowImages = false;
    bool isShowUImages = false;
    bool isShowTImages = false;
    bool isShowDImages = false;
    bool isShowCImages = false;

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

    return new StereoImage(drawing1, drawing2); //?

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

Mat StereoProcessing::circlesPattern() {
    Mat image(1792, 1600, CV_8UC3);
    image.setTo(Scalar(255, 255, 255));
    int n = BOARD_WIDTH; //9
    int m = BOARD_HEIGHT; //8
    int r = 64;
    for (int i = 0; i < (n + 1) * m; i++) {
        circle(image, Point((i % m) * (3 * r) + 2 * r, (i / n) * (3 * r) + 2 * r), r, Scalar(0, 0, 0), -1);
    }
    return image;
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
