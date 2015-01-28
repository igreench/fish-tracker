#include "mainwindow.h"
#include <QApplication>

#include "FlyCapture2.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv/cv.h>

#include "opencv2/contrib/contrib.hpp"

#include <QDebug>

using namespace FlyCapture2;

//std::vector<cv::Point3f> objectPoints;
std::vector<std::vector<cv::Point3f> > objectPoints;


//std::vector<cv::Point2f> imageCorners1;
//std::vector<cv::Point2f> imageCorners2;
//std::vector<std::vector<cv::Point2f> > imagePoints1;
//std::vector<std::vector<cv::Point2f> > imagePoints2;
std::vector<std::vector<cv::Point2f> > imagePoints1;
std::vector<std::vector<cv::Point2f> > imagePoints2;

const int BOARD_WIDTH = 8;
const int BOARD_HEIGHT = 9;

template<class T>
QString templateMatToString(cv::Mat mat) {
    QString str;
    str+= QString("%1,%2;").arg(mat.rows).arg(mat.cols);
    for(int i=0;i<mat.rows;i++)
    {
        for(int j=0;j<mat.cols;j++)
        {
            str+= QString("%1").arg(mat.at<T>(i,j));
            if(j+1<mat.cols)
            {
               str+= QString(",");
            }
        }
        str+= QString(";");
    }
    return str;
}

QString matToString(cv::Mat mat)
{
    if(mat.empty())
    {
        return "";
    }
    switch (mat.type()) {
    case CV_32F :
        return templateMatToString<float>(mat);
        break;
    case CV_64F :
        return templateMatToString<double>(mat);
        break;
    case CV_8U :
        return templateMatToString<unsigned char>(mat);
        break;
    default:
        qDebug() << "Error: unknowing type\n";
        return "";
    }
}

bool addImage(const cv::Mat im, std::vector<cv::Point2f> *imageCorners, cv::Mat &result)
{
    std::vector<std::vector<cv::Point2f> > detected_corners;//!< Список обнаруженных углов
    cv::Size pattern_size = cv::Size(BOARD_WIDTH, BOARD_HEIGHT);//!< Размер шахматной доски
    cv::Size imsize; //!< Размер изображение вычисляется в addImage

    //std::vector<cv::Point2f> imageCorners;

    qDebug() << "addImage";
    if(im.empty())
    {
        qDebug() << "Empty image exit";
        return false;
    }

    cv::Mat grey;
    //imsize = cv::Size(im.rows, im.cols);
    imsize = cv::Size(im.rows, im.cols);


   // grey.create(imsize,CV_8UC1);

    if(1!=im.channels())
    {

        cv::cvtColor(im, grey, CV_BGR2GRAY);
      //  im.convertTo(grey,CV_8UC1);
        result = im.clone();
    }else{
        //Р Р°Р·РјР°Р·С‹РІР°РµРј, С‡С‚РѕР±С‹ РґРµС‚РµРєС‚РѕСЂ РЅРµ РѕС‚РІР»РµРєР°Р»СЃСЏ РЅР° РјРµР»РєРёРµ С‚РѕС‡РєРё
       // cv::blur(im,grey,Size(6,6));
        grey = im.clone();

   //     result.create(cv::Size(grey.cols,grey.rows),CV_8UC3);
   //     result.setTo(0);
        cv::cvtColor(grey, result, CV_GRAY2BGR);
    }

    imsize = cv::Size(grey.cols,grey.rows);

    std::vector<cv::Point2f> detected;
    double fx = 1;
    double fy = 1;
    try{

        qDebug() << "findChessboardCorners";
        cv::Mat small_gray;
        small_gray = grey;

        bool found = false;
        //,CV_CALIB_CB_NORMALIZE_IMAGE+CALIB_CB_FAST_CHECK CV_CALIB_CB_ADAPTIVE_THRESH

        result = grey;
        //found = cv::findCirclesGrid(small_gray, pattern_size, detected, cv::CALIB_CB_SYMMETRIC_GRID);
        //found = cv::findCirclesGrid(small_gray, pattern_size, detected_corners, cv::CALIB_CB_SYMMETRIC_GRID);
        found = cv::findCirclesGrid(small_gray, pattern_size, *imageCorners, cv::CALIB_CB_SYMMETRIC_GRID);
        if(!found)
        {
            qDebug() << "Not found";
            return false;
        }

        /*for(unsigned int i=0;i<detected.size();i++)
        {
            cv::Point2f pt = detected[i];
            detected[i] = cv::Point2f(pt.x/fx,pt.y/fy);
        //    qDebug() << pt.x << pt.x/fx << pt.y << pt.y/fy;
        }*/

        //detected_corners.push_back(detected);

        qDebug() << "found";
        //cv::drawChessboardCorners(result, pattern_size, detected_corners, true);
        cv::drawChessboardCorners(result, pattern_size, *imageCorners, true);
        return true;


    }catch(...)
    {
        qDebug() << "Exception";
    }


    return false;
}

int main() {
    qDebug() << "Start program";
    Error error;
    Camera camera;
    Camera camera2;
    PGRGuid guid, guid2;
    BusManager busMgr;
    CameraInfo camInfo;

    bool isStereoCalibrated = false;
    bool isStereoRectified = false;
    cv::Mat scres;// stereo callibrate result with points
    cv::Mat R, T, Q; //
    cv::Mat rmap[2];

    // Connect the camera
    error = busMgr.GetCameraFromIndex(0, &guid);
    error = camera.Connect(&guid);
    if (error != PGRERROR_OK) {
        qDebug() << "Failed to connect to camera" ;
        return -1;
    }

    // Get the camera info and print it out
    error = camera.GetCameraInfo(&camInfo);
    if (error != PGRERROR_OK) {
        qDebug() << "Failed to get camera info from camera";
        return -1;
    }
    qDebug() << camInfo.vendorName << " "
              << camInfo.modelName << " "
              << camInfo.serialNumber;

    // Connect the camera

    error = busMgr.GetCameraFromIndex(1, &guid2);
    error = camera2.Connect(&guid2);
    if (error != PGRERROR_OK) {
        qDebug() << "Failed to connect to camera" ;
        return -1;
    }

    // Get the camera info and print it out
    error = camera2.GetCameraInfo(&camInfo);
    if (error != PGRERROR_OK) {
        qDebug() << "Failed to get camera info from camera";
        return -1;
    }
    qDebug() << camInfo.vendorName << " "
              << camInfo.modelName << " "
              << camInfo.serialNumber;

    error = camera.StartCapture();
    if (error == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED) {
        qDebug() << "Bandwidth exceeded";
        return -1;
    } else {
        if (error != PGRERROR_OK) {
            qDebug() << "Failed to start image capture";
            return -1;
        }
    }

    error = camera2.StartCapture();
    if (error == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED) {
        qDebug() << "Bandwidth exceeded";
        return -1;
    } else {
        if (error != PGRERROR_OK) {
            qDebug() << "Failed to start image capture";
            return -1;
        }
    }

    // Capture loop
    qDebug() << "Start capture";
    char key = 0;
    while (key != 'q') {

        // camera1: get the image
        Image rawImage1;
        Error error = camera.RetrieveBuffer(&rawImage1);
        if (error != PGRERROR_OK) {
            qDebug() << "Capture error";
            continue;
        }

        // camera1: convert to rgb
        Image rgbImage1;
        rawImage1.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage1);

        // camera1: convert to OpenCV Mat
        unsigned int rowBytes = (double)rgbImage1.GetReceivedDataSize() / (double)rgbImage1.GetRows();
        cv::Mat image1_1 = cv::Mat(rgbImage1.GetRows(), rgbImage1.GetCols(), CV_8UC3, rgbImage1.GetData(), rowBytes);
        cv::Mat image1_2 = cv::Mat(rgbImage1.GetRows() / 4, rgbImage1.GetCols() / 4, CV_8UC3);


        // camera1: undistortion

        //3,3;1947.44,0,1011.07;0,1707.99,414.401;0,0,1;
        //1,5;-0.160223,-0.123745,0.00997641,-0.00934415,0;
//        cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << 1898.86,0,801.607,0,1684.22,533.037,0,0,1);
//        cv::Mat distCoeffs = (cv::Mat_<double>(1,5) << -0.138546,-0.290375,0.00790141,0.00869189,0);
        cv::Mat cameraMatrix1 = (cv::Mat_<double>(3,3) << 1947.44,0,1011.07,0,1707.99,414.401,0,0,1);
        cv::Mat distCoeffs1 = (cv::Mat_<double>(1,5) << -0.160223,-0.123745,0.00997641,-0.00934415,0);
        cv::Size imageSize1 = image1_1.size();

        // camera2: get the image
        Image rawImage2;
        error = camera2.RetrieveBuffer(&rawImage2);
        if (error != PGRERROR_OK) {
            qDebug() << "Capture error";
            continue;
        }

        // camera2: convert to rgb
        Image rgbImage2;
        rawImage2.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage2);

        // camera2: convert to OpenCV Mat
        rowBytes = (double)rgbImage2.GetReceivedDataSize() / (double)rgbImage2.GetRows();
        cv::Mat image2_1 = cv::Mat(rgbImage2.GetRows(), rgbImage2.GetCols(), CV_8UC3, rgbImage2.GetData(),rowBytes);
        cv::Mat image2_2 = cv::Mat(rgbImage2.GetRows() / 4, rgbImage1.GetCols() / 4, CV_8UC3);
        cv::resize(image2_1, image2_2, image2_2.size(), 0, 0, CV_INTER_LINEAR);

        // camera2: undistortion
        //3,3;1849.16,0,900.797;0,1642.24,507.873;0,0,1;
        //1,5;-0.26513,0.15058,0.00745374,-0.00767843,0;
//        cv::Mat cameraMatrix2 = (cv::Mat_<double>(3,3) << 1898.86,0,801.607,0,1684.22,533.037,0,0,1);
//        cv::Mat distCoeffs2 = (cv::Mat_<double>(1,5) << -0.138546,-0.290375,0.00790141,0.00869189,0);
        cv::Mat cameraMatrix2 = (cv::Mat_<double>(3,3) << 1849.16,0,900.797,0,1642.24,507.873,0,0,1);
        cv::Mat distCoeffs2 = (cv::Mat_<double>(1,5) << -0.26513,0.15058,0.00745374,-0.00767843,0);
        cv::Size imageSize2 = image2_1.size();

        //stereoCalibrate

        cv::Mat E, F;

        if (!isStereoCalibrated) {
            std::vector<cv::Point2f> corners1, corners2;
            isStereoCalibrated = addImage(image1_1, &corners1, scres) && addImage(image2_1, &corners2, scres);            
            imagePoints1.clear();
            imagePoints2.clear();
            objectPoints.clear();
            imagePoints1.push_back(corners1);            
            imagePoints2.push_back(corners2);
            if (isStereoCalibrated) {
                int n = BOARD_WIDTH * BOARD_HEIGHT;
                std::vector<cv::Point3f> obj;
                for (int j = 0; j < n; j++) {
                    obj.push_back(cv::Point3f(j % BOARD_WIDTH, j / BOARD_WIDTH, 0.0f));
                }
                objectPoints.push_back(obj);

                qDebug() << "imagePoints1 size:" << imagePoints1.size();
                qDebug() << "imagePoints2 size:" << imagePoints2.size();
                qDebug() << "corners1 size:" << corners1.size();
                qDebug() << "corners2 size:" << corners2.size();
                qDebug() << "objectPoints size:" << objectPoints.size();
                qDebug() << "obj size:" << obj.size();

                cv::stereoCalibrate(objectPoints, imagePoints1, imagePoints2,
                                    cameraMatrix1,
                                    distCoeffs1,
                                    cameraMatrix2,
                                    distCoeffs2,
                                    imageSize1, R, T, E, F,
                                    cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-8),
                                    CV_CALIB_ZERO_TANGENT_DIST +
                                    CV_CALIB_FIX_INTRINSIC+
                                    CV_CALIB_FIX_K3);

                qDebug() << "R" << matToString(R);
                qDebug() << "T" << matToString(T);
            }
            //cv::imshow("scres", scres);//
        }

        cv::Mat R1, R2, P1, P2; //Q
        // stereoRectify
        if (isStereoCalibrated && !isStereoRectified) {

            //cv::resize(scres, scres, image2_2.size(), 0, 0, CV_INTER_LINEAR);
            //cv::imshow("scres", scres);

            //cv::Mat Q;
            cv::Rect validRoi[2];
            //cv::Mat R = (cv::Mat_<double>(3,3) <<  0.87, -0.003, -0.46, 0.001, 0.999, -0.003, 0.46, 0.002, 0.89);
            //cv::Mat T = (cv::Mat_<double> (3,1) << 228, 0, 0);
            //cv::Rect validRoi[2];

            qDebug() << "cameraMatrix1" << matToString(cameraMatrix1);
            qDebug() << "distCoeffs1" << matToString(distCoeffs1);
            qDebug() << "cameraMatrix2" << matToString(cameraMatrix2);
            qDebug() << "distCoeffs2" << matToString(distCoeffs2);

            //Mat R1, R2, P1, P2, Q;

            qDebug() << "Starting Rectification";
            cv::stereoRectify(cameraMatrix1,
                              distCoeffs1,
                              cameraMatrix2,
                              distCoeffs2,
                              imageSize1,
                              R, T, R1, R2, P1, P2, Q//,//
                              //CV_CALIB_ZERO_DISPARITY, 1,
                              //imageSize1,
                              //&validRoi[0],
                              //&validRoi[1]
                              );
            qDebug() << "Done Rectification";
            qDebug() << "Q: " << matToString(Q);

            isStereoRectified = true;
        }

        if (isStereoCalibrated && isStereoRectified) {
/*
            cv::initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, imageSize1, CV_16SC2, rmap[0], rmap[1]);
            cv::initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, imageSize2, CV_16SC2, rmap[0], rmap[1]);



            cv::Mat rimage1, rimage2;
            //cv::Mat R1, P1, rimage1;
            cv::remap(image1_1, rimage1, rmap[0], rmap[1], CV_INTER_LINEAR);

            //cv::Mat R2, P2, rimage2;
            //cv::getDefaultNewCameraMatrix(
            cv::remap(image2_1, rimage2, rmap[0], rmap[1], CV_INTER_LINEAR);

            //
            cv::StereoSGBM sgbm;
            sgbm.SADWindowSize = 5;
            sgbm.numberOfDisparities = 192;
            sgbm.preFilterCap = 4;
            sgbm.minDisparity = -64;
            sgbm.uniquenessRatio = 1;
            sgbm.speckleWindowSize = 150;
            sgbm.speckleRange = 2;
            sgbm.disp12MaxDiff = 10;
            sgbm.fullDP = false;
            sgbm.P1 = 600;
            sgbm.P2 = 2400;

            cv::Mat disp, disp8;
            sgbm(rimage1, rimage2, disp);
            cv::normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);

            cv::resize(disp8, disp8, image2_2.size(), 0, 0, CV_INTER_LINEAR);
            cv::imshow("disp", disp8);

            cv::resize(rimage1, rimage1, image1_2.size(), 0, 0, CV_INTER_LINEAR);
            cv::imshow("rimage1", rimage1);
            cv::resize(rimage2, rimage2, image2_2.size(), 0, 0, CV_INTER_LINEAR);
            cv::imshow("rimage2", rimage2);*/
        } else {
            cv::resize(image1_1, image1_1, image1_2.size(), 0, 0, CV_INTER_LINEAR);
            cv::imshow("image1", image1_1);
            cv::resize(image2_1, image2_1, image2_2.size(), 0, 0, CV_INTER_LINEAR);
            cv::imshow("image2", image2_1);
        }

        if (key == 'k') {
            if (isStereoCalibrated && isStereoRectified) {

                qDebug() << "Let's check disparity map";
                std::vector<cv::Point2f> corners1;
                if (addImage(image1_1, &corners1, scres)) {

                    qDebug() << "Pattern found";

                    cv::initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, imageSize1, CV_16SC2, rmap[0], rmap[1]);
                    cv::initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, imageSize2, CV_16SC2, rmap[0], rmap[1]);

                    qDebug() << "Done initUndistortRectifyMap";

                    cv::Mat rimage1, rimage2;
                    cv::remap(image1_1, rimage1, rmap[0], rmap[1], CV_INTER_LINEAR);
                    cv::remap(image2_1, rimage2, rmap[0], rmap[1], CV_INTER_LINEAR);

                    qDebug() << "Done remap";

                    cv::Mat r;
                    cv::Rodrigues(R, r);

                    qDebug() << "Done Rodrigues";

                    //std::vector<std::vector<cv::Point2f> > imagePoints1;
                    //std::vector<cv::Point2f> projectedPoints1;

                    //cv::projectPoints(objectPoints, r, T, cameraMatrix1, distCoeffs1, projectedPoints1);

                    /*for(unsigned int i = 0; i < projectedPoints1.size(); ++i) {
                        //std::cout << "Image point: " << imagePoints1[i] << " Projected to " << projectedPoints1[i] << std::endl;
                    }*/

                    //

                    cv::StereoSGBM sgbm;
                    sgbm.SADWindowSize = 5;
                    sgbm.numberOfDisparities = 192;
                    sgbm.preFilterCap = 4;
                    sgbm.minDisparity = -64;
                    sgbm.uniquenessRatio = 1;
                    sgbm.speckleWindowSize = 150;
                    sgbm.speckleRange = 2;
                    sgbm.disp12MaxDiff = 10;
                    sgbm.fullDP = false;
                    sgbm.P1 = 600;
                    sgbm.P2 = 2400;

                    qDebug() << "Done StereoSGBM";

                    cv::Mat disp, disp8;
                    sgbm(rimage1, rimage2, disp);
                    cv::normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);

                    qDebug() << "Done normalize";

                    //cv::resize(disp8, disp8, image2_2.size(), 0, 0, CV_INTER_LINEAR);
                    //cv::imshow("disp", disp8);//

                    //
                    cv::Mat image3d;

                    cv::reprojectImageTo3D(disp8, image3d, Q);

                    qDebug() << "Done reprojectImageTo3D";

                    cv::resize(scres, scres, image1_2.size(), 0, 0, CV_INTER_LINEAR);
                    cv::imshow("scres", scres);

                    //objectPoints.clear();
                    std::vector<std::vector<cv::Point3d> > objectPointsP;
                    std::vector<cv::Point3d> objpoints;
                    for(unsigned int i = 0; i < corners1.size(); ++i) {
                        //objectPoints.push_back( image3d(corners1[i].x, corners1[i].y) );
                        /*objpoints.push_back(cv::Point3f(
                                                image3d.at<cv::Point3f>(image3d(corners1[i].x, corners1[i].y)[0]),
                                                image3d.at<cv::Point3f>(image3d(corners1[i].x, corners1[i].y)[1]),
                                                image3d.at<cv::Point3f>(image3d(corners1[i].x, corners1[i].y)[2])));*/
                        if ((image3d.at<cv::Vec3f>(corners1[i].x, corners1[i].y)[0] > -1000) &&
                            (image3d.at<cv::Vec3f>(corners1[i].x, corners1[i].y)[0] < 1000) &&
                            (image3d.at<cv::Vec3f>(corners1[i].x, corners1[i].y)[1] > -1000) &&
                            (image3d.at<cv::Vec3f>(corners1[i].x, corners1[i].y)[1] < 1000) &&
                            (image3d.at<cv::Vec3f>(corners1[i].x, corners1[i].y)[2] > -1000) &&
                            (image3d.at<cv::Vec3f>(corners1[i].x, corners1[i].y)[2] < 1000)) {
                            objpoints.push_back(cv::Point3f(
                                                    image3d.at<cv::Vec3f>(corners1[i].x, corners1[i].y)[0],
                                                    image3d.at<cv::Vec3f>(corners1[i].x, corners1[i].y)[1],
                                                    image3d.at<cv::Vec3f>(corners1[i].x, corners1[i].y)[2]));
                            //qDebug() << "Corner " << i << " : x=" << corners1[i].x << " y=" << corners1[i].y;
                            //qDebug() << "objpoints " << i << " : x=" << objpoints[i].x << " y=" << objpoints[i].y << " z=" << objpoints[i].z;
                        } else {
                            //objpoints.push_back(cv::Point3f(0.0f, 0.0f, 0.0f));
                            //objpoints.push_back(cv::Point3d(0, 0, 0));
                            //objpoints.push_back(objpoints[i-1]);
                            //qDebug() << "out of range";
                            //qDebug() << "Corner " << i << " : x=" << corners1[i].x << " y=" << corners1[i].y;
                            //qDebug() << "objpoints " << i << " : x=" << objpoints[i].x << " y=" << objpoints[i].y << " z=" << objpoints[i].z;
                        }
                    }
                    objectPointsP.push_back(objpoints);

                    qDebug() << "Done adding object points";


                    //imagePoints1.clear();
                    //std::vector<std::vector<cv::Point2f> > imagePointsP;
                    std::vector<cv::Point2d> imagePoints;
                    qDebug() << "Start projectPoints";

                    qDebug() << "r: " << matToString(r);
                    qDebug() << "T: " << matToString(T);
                    qDebug() << "cameraMatrix1: " << matToString(cameraMatrix1);
                    qDebug() << "distCoeffs1: " << matToString(distCoeffs1);
                    for(unsigned int i = 0; i < objpoints.size(); ++i) {
                        qDebug() << objpoints[i].x << objpoints[i].y << objpoints[i].z;
                    }
                    qDebug() << "Start projectPoints";
                    cv::projectPoints(objpoints, r, T, cameraMatrix1, distCoeffs1, imagePoints);
                    //cv::projectPoints(objpoints, r, T, cameraMatrix1, distCoeffs1, imagePointsP);

                    qDebug() << "Done projectPoints";

                    cv::Size pattern_size = cv::Size(BOARD_WIDTH, BOARD_HEIGHT);

                    //cv::drawChessboardCorners(scres, pattern_size, imagePoints1, true);
                    //qDebug() << "Done drawChessboardCorners";

                    //imagePoints.push_back(cv::Point2d(50, 50));

                    for(unsigned int i = 0; i < imagePoints.size(); ++i) {
                        cv::circle(rimage1, cv::Point2d(imagePoints[i].x, imagePoints[i].y), 30, cv::Scalar( 255, 0, 0 ));
                        qDebug() << "x:" << imagePoints[i].x << "y:" << imagePoints[i].y;
                    }

                    //cv::resize(scres, scres, image1_2.size(), 0, 0, CV_INTER_LINEAR);
                    //cv::imshow("scres1", scres);

                    cv::resize(rimage1, rimage1, image1_2.size(), 0, 0, CV_INTER_LINEAR);
                    cv::imshow("scres1", rimage1);
                }
            }
        }

        // wait key
        key = cv::waitKey(30);
    }

    qDebug() << "Stop capture";
    error = camera.StopCapture();
    if (error != PGRERROR_OK) {
        // This may fail when the camera was removed, so don't show
        // an error message
    }
    error = camera2.StopCapture();
    if (error != PGRERROR_OK) {
        // This may fail when the camera was removed, so don't show
        // an error message
    }

    //intristics 3,3;1977,0,934.867;0,1812.95,904.821;0,0,1;
    //distCoeffs 1,5;-0.0833973,0.151344,0.0536747,0.045857,0;


    camera.Disconnect();
    camera2.Disconnect();

    return 0;
}
