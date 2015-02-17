#include "stereoscopy.h"

#include <opencv/cv.h>

#include <windows.h>

#include <QDebug>

Stereoscopy::Stereoscopy() {
    isStereoCalibrated = false;
    isShowing = true;

    /*sgbm.SADWindowSize = 5;
    sgbm.numberOfDisparities = 192;
    sgbm.preFilterCap = 4;
    sgbm.minDisparity = -64;
    sgbm.uniquenessRatio = 1;
    sgbm.speckleWindowSize = 150;
    sgbm.speckleRange = 2;
    sgbm.disp12MaxDiff = 10;
    sgbm.fullDP = false;
    sgbm.P1 = 600;
    sgbm.P2 = 2400;*/

    sgbm.SADWindowSize = 5; //5
    sgbm.numberOfDisparities = 128; //192
    sgbm.preFilterCap = 1; //4
    sgbm.minDisparity = 0; //-64
    sgbm.uniquenessRatio = 1; //1
    sgbm.speckleWindowSize = 150; //150
    sgbm.speckleRange = 2; //2
    sgbm.disp12MaxDiff = 2; //10
    sgbm.fullDP = false; //false
    sgbm.P1 = 400; //600
    sgbm.P2 = 1600; //2400
}

void Stereoscopy::startCapture() {
    qDebug() << "Start program";

    PGRGuid guid, guid2;
    BusManager busMgr;
    CameraInfo camInfo;

    // Connect the camera
    error = busMgr.GetCameraFromIndex(0, &guid);
    error = camera.Connect(&guid);
    if (error != PGRERROR_OK) {
        qDebug() << "Failed to connect to camera" ;
        return ; //exception
    }

    // Get the camera info and print it out
    error = camera.GetCameraInfo(&camInfo);
    if (error != PGRERROR_OK) {
        qDebug() << "Failed to get camera info from camera";
        return ; //exception
    }
    qDebug() << camInfo.vendorName << " "
              << camInfo.modelName << " "
              << camInfo.serialNumber;

    // Connect the camera

    error = busMgr.GetCameraFromIndex(1, &guid2);
    error = camera2.Connect(&guid2);
    if (error != PGRERROR_OK) {
        qDebug() << "Failed to connect to camera" ;
        return ; //exception
    }

    // Get the camera info and print it out
    error = camera2.GetCameraInfo(&camInfo);
    if (error != PGRERROR_OK) {
        qDebug() << "Failed to get camera info from camera";
        return ; //exception
    }
    qDebug() << camInfo.vendorName << " "
              << camInfo.modelName << " "
              << camInfo.serialNumber;

    error = camera.StartCapture();
    if (error == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED) {
        qDebug() << "Bandwidth exceeded";
        return ; //exception
    } else {
        if (error != PGRERROR_OK) {
            qDebug() << "Failed to start image capture";
            return ; //exception
        }
    }

    error = camera2.StartCapture();
    if (error == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED) {
        qDebug() << "Bandwidth exceeded";
        return ; //exception
    } else {
        if (error != PGRERROR_OK) {
            qDebug() << "Failed to start image capture";
            return ; //exception
        }
    }
}

void Stereoscopy::endCapture() {
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
}

void Stereoscopy::showImagesFromCameras() {
    char key = 0;
    while (key != 'q') {
        Image rawImage1;
        Error error = camera.RetrieveBuffer(&rawImage1);
        if (error != PGRERROR_OK) {
            qDebug() << "Capture error";
            continue;
        }
        Image rgbImage1;
        rawImage1.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage1);
        unsigned int rowBytes = (double)rgbImage1.GetReceivedDataSize() / (double)rgbImage1.GetRows();
        Mat image1_1 = Mat(rgbImage1.GetRows(), rgbImage1.GetCols(), CV_8UC3, rgbImage1.GetData(), rowBytes);
        Mat image1_2 = Mat(rgbImage1.GetRows() / 4, rgbImage1.GetCols() / 4, CV_8UC3);
        Image rawImage2;
        error = camera2.RetrieveBuffer(&rawImage2);
        if (error != PGRERROR_OK) {
            qDebug() << "Capture error";
            continue;
        }
        Image rgbImage2;
        rawImage2.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage2);
        rowBytes = (double)rgbImage2.GetReceivedDataSize() / (double)rgbImage2.GetRows();
        Mat image2_1 = Mat(rgbImage2.GetRows(), rgbImage2.GetCols(), CV_8UC3, rgbImage2.GetData(),rowBytes);
        Mat image2_2 = Mat(rgbImage2.GetRows() / 4, rgbImage1.GetCols() / 4, CV_8UC3);
        if (key == 'b') {
            imwrite( "image1.jpg", image1_1 );
            imwrite( "image2.jpg", image2_1 );
        }
        resize(image1_1, image1_1, image2_2.size(), 0, 0, CV_INTER_LINEAR);
        resize(image2_1, image2_1, image1_2.size(), 0, 0, CV_INTER_LINEAR);
        imshow("image1", image1_1);
        imshow("image2", image2_1);
        key = waitKey(30);
    }
}

void Stereoscopy::loopCapture() {
    qDebug() << "Start capture";

    // camera1: undistortion
    //Mat cameraMatrix1 = (Mat_<double>(3,3) << 1947.44,0,1011.07,0,1707.99,414.401,0,0,1);
//    Mat cameraMatrix1 = (Mat_<double>(3,3) << 1927.04,0,917.475,0,1706.88,579.804,0,0,1);
    //Mat distCoeffs1 = (Mat_<double>(1,5) << -0.160223,-0.123745,0.00997641,-0.00934415,0);
//    Mat distCoeffs1 = (Mat_<double>(1,5) << -0.215192,0.171866,0.0118907,0.00368162,0);


    // camera2: undistortion
    //Mat cameraMatrix2 = (Mat_<double>(3,3) << 1849.16,0,900.797,0,1642.24,507.873,0,0,1);
//    Mat cameraMatrix2 = (Mat_<double>(3,3) << 1766.16,0,817.485,0,1564.63,562.498,0,0,1);
    //Mat distCoeffs2 = (Mat_<double>(1,5) << -0.26513,0.15058,0.00745374,-0.00767843,0);
//    Mat distCoeffs2 = (Mat_<double>(1,5) << -0.19803,0.0611625,-0.00181152,-0.000889885,0);

    /*Mat cameraMatrix1 = (Mat_<double>(3,3) << 6413.91,0,810.516,0,5364.43,717.19,0,0,1);
    Mat distCoeffs1 = (Mat_<double>(1,5) << -3.62446,83.4373,-0.0860216,0.0135343,-2475.21);
    Mat cameraMatrix2 = (Mat_<double>(3,3) << 13175.3,0,787.325,0,12363.5,646.243,0,0,1);
    Mat distCoeffs2 = (Mat_<double>(1,5) << -14.8809,391.48,-0.103346,-0.0954315,3172.4);*/

    Mat cameraMatrix1 = (Mat_<double>(3,3) << 1806.53,0,815.786,0,1595.14,590.314,0,0,1);
    Mat distCoeffs1 = (Mat_<double>(1,5) << -0.267514,0.213748,0.00136627,0.000194796,0);
    Mat cameraMatrix2 = (Mat_<double>(3,3) << 1739.3,0,808.929,0,1542.11,581.767,0,0,1);
    Mat distCoeffs2 = (Mat_<double>(1,5) << -0.247249,0.161344,-0.00280154,0.000444185,0);

    /*Mat rvec1 = (Mat_<double>(3,1) << -0.121595,0.378872,-0.181438);
    Mat tvec1 = (Mat_<double>(3,1) << -2.64742,-5.55454,64.7538);
    Mat rvec2 = (Mat_<double>(3,1) << -0.335479,0.106753,-0.188999);
    Mat tvec2 = (Mat_<double>(3,1) << -6.88747,-4.15174,138.473);*/

    Mat R1, R2, P1, P2; //Q
    Mat E, F;
    cv::Rect validRoi[2];

    //for debug
    /*Mat CM1 = Mat(3, 3, CV_64FC1);
    Mat CM2 = Mat(3, 3, CV_64FC1);
    Mat D1, D2;*/

    imagePoints1.clear();
    imagePoints2.clear();
    objectPoints.clear();

    bool isAddedPattern = false;

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
        Mat image1_1 = Mat(rgbImage1.GetRows(), rgbImage1.GetCols(), CV_8UC3, rgbImage1.GetData(), rowBytes);
        Mat image1_2 = Mat(rgbImage1.GetRows() / 4, rgbImage1.GetCols() / 4, CV_8UC3);
        //resize(image1_1, image1_1, image1_2.size(), 0, 0, CV_INTER_LINEAR);//
        Size imageSize1 = image1_1.size(); //1600x1200



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
        Mat image2_1 = Mat(rgbImage2.GetRows(), rgbImage2.GetCols(), CV_8UC3, rgbImage2.GetData(),rowBytes);
        Mat image2_2 = Mat(rgbImage2.GetRows() / 4, rgbImage1.GetCols() / 4, CV_8UC3);
        //resize(image2_1, image2_1, image2_2.size(), 0, 0, CV_INTER_LINEAR);//
        Size imageSize2 = image2_1.size();

        //stereoCalibrate



        if (key == 'c') {

            vector<Point2f> corners1, corners2;

            if (addImage(image1_1, &corners1, scres) && addImage(image2_1, &corners2, scres)) {

                imwrite( "image1.jpg", image1_1 );
                imwrite( "image2.jpg", image2_1 );

                imagePoints1.clear();
                imagePoints2.clear();
                objectPoints.clear();
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
                calibrateCamera(objectPoints, imagePoints1, imageSize1, cameraMatrix1, distCoeffs1, rvecs1, tvecs1);
                calibrateCamera(objectPoints, imagePoints2, imageSize2, cameraMatrix2, distCoeffs2, rvecs2, tvecs2);
//                calibrateCamera(objectPoints, imagePoints1, imageSize1, cameraMatrix1, distCoeffs1, rvec1, tvec1);
//                calibrateCamera(objectPoints, imagePoints2, imageSize2, cameraMatrix2, distCoeffs2, rvec2, tvec2);

                qDebug() << "cameraMatrix1: " << matToString(cameraMatrix1);
                qDebug() << "distCoeffs1: " << matToString(distCoeffs1);

                projectPoints(Mat(obj), rvecs1[0], tvecs1[0], cameraMatrix1, distCoeffs1, imagePoints);

                /*Size pattern_size = Size(BOARD_WIDTH, BOARD_HEIGHT);
                drawChessboardCorners(image1_1, pattern_size, imagePoints1, true);*/

                for(unsigned int i = 0; i < imagePoints.size(); ++i) {
                    circle(image1_1, Point2d(imagePoints[i].x, imagePoints[i].y), 20, Scalar( 255, 0, 0 ), CV_FILLED, 8, 0);
                    qDebug() << "x:" << imagePoints[i].x << "y:" << imagePoints[i].y;
                }

                //resize(scres, scres, image1_2.size(), 0, 0, CV_INTER_LINEAR);
                //imshow("scres1", scres);

                resize(image1_1, image1_1, image1_2.size(), 0, 0, CV_INTER_LINEAR);
                imshow("image1_1", image1_1);

            }
        }

        if (key == 'b') {
            imwrite( "image1.jpg", image1_1 );
            imwrite( "image2.jpg", image2_1 );
        }

        if (key == 'r') {
            vector<Point2f> corners1, corners2;
            if (addImage(image1_1, &corners1, scres) && addImage(image2_1, &corners2, scres)) {
                imagePoints1.push_back(corners1);
                imagePoints2.push_back(corners2);
                int n = BOARD_WIDTH * BOARD_HEIGHT;
                vector<Point3f> obj;
                for (int j = 0; j < n; j++) {
                    obj.push_back(Point3f(j % BOARD_WIDTH, j / BOARD_WIDTH, 0.0f));
                }
                objectPoints.push_back(obj);
                isAddedPattern = true;
            }

            //
            if (isAddedPattern) {
                stereoCalibrate(objectPoints, imagePoints1, imagePoints2,
                                cameraMatrix1,
                                distCoeffs1,
                                cameraMatrix2,
                                distCoeffs2,
                                imageSize1, R, T, E, F,
                                TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 1e-6),
                                CV_CALIB_FIX_ASPECT_RATIO);
                stereoRectify(cameraMatrix1,
                              distCoeffs1,
                              cameraMatrix2,
                              distCoeffs2,
                              imageSize1,
                              R, T, R1, R2, P1, P2, Q);
                initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, imageSize1, CV_32FC1, rmap1x, rmap1y);
                initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, imageSize2, CV_32FC1, rmap2x, rmap2y);
                qDebug() << "SEREOCALIBRATED";
                qDebug() << "cameraMatrix1" << matToString(cameraMatrix1);
                qDebug() << "distCoeffs1" << matToString(distCoeffs1);
                qDebug() << "cameraMatrix2" << matToString(cameraMatrix2);
                qDebug() << "distCoeffs2" << matToString(distCoeffs2);
                qDebug() << "R1" << matToString(R1);
                qDebug() << "P1" << matToString(P1);
                qDebug() << "R2" << matToString(R2);
                qDebug() << "P2" << matToString(P2);
                Mat rimage1, rimage2;
                remap(image1_1, rimage1, rmap1x, rmap1y, CV_INTER_LINEAR);
                remap(image2_1, rimage2, rmap2x, rmap2y, CV_INTER_LINEAR);
                rectangle(rimage1, validRoi[0], Scalar(0, 255, 0), 3);
                rectangle(rimage2, validRoi[1], Scalar(0, 255, 0), 3);
                resize(rimage1, rimage1, image1_2.size(), 0, 0, CV_INTER_LINEAR);
                resize(rimage2, rimage2, image2_2.size(), 0, 0, CV_INTER_LINEAR);
                imshow("rimage1", rimage1);
                imshow("rimage2", rimage2);
            }
        }

        if (key == 's') {
            imwrite( "image1.jpg", image1_1 );
            imwrite( "image2.jpg", image2_1 );

            if (!isAddedPattern) {

            vector<Point2f> corners1, corners2;

            if (addImage(image1_1, &corners1, scres) && addImage(image2_1, &corners2, scres)) {

                imwrite( "image1.jpg", image1_1 );
                imwrite( "image2.jpg", image2_1 );

                imagePoints1.clear();
                imagePoints2.clear();
                objectPoints.clear();
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

                //stereocalibrate

               stereoCalibrate(objectPoints, imagePoints1, imagePoints2,
                                    cameraMatrix1,
                                    distCoeffs1,
                                    cameraMatrix2,
                                    distCoeffs2,
                                    imageSize1, R, T, E, F
                                    //,
                                    /*TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-8),
                                    CV_CALIB_ZERO_TANGENT_DIST +
                                    CV_CALIB_FIX_INTRINSIC+
                                    CV_CALIB_FIX_K3*/
                                    /*TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
                                    CV_CALIB_SAME_FOCAL_LENGTH | CV_CALIB_ZERO_TANGENT_DIST*/
                                    /*TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
                                    CV_CALIB_FIX_ASPECT_RATIO +
                                    CV_CALIB_ZERO_TANGENT_DIST +
                                    CV_CALIB_SAME_FOCAL_LENGTH +
                                    CV_CALIB_RATIONAL_MODEL +
                                    CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5*/
                                    );

                /*stereoCalibrate(objectPoints, imagePoints1, imagePoints2,
                    CM1, D1, CM2, D2, image1_1.size(), R, T, E, F,
                    cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
                    CV_CALIB_SAME_FOCAL_LENGTH | CV_CALIB_ZERO_TANGENT_DIST);*/

                qDebug() << "R" << matToString(R);
                qDebug() << "T" << matToString(T);



                qDebug() << "cameraMatrix1" << matToString(cameraMatrix1);
                qDebug() << "distCoeffs1" << matToString(distCoeffs1);
                qDebug() << "cameraMatrix2" << matToString(cameraMatrix2);
                qDebug() << "distCoeffs2" << matToString(distCoeffs2);

                qDebug() << "Starting Rectification";
                /*cv::stereoRectify(cameraMatrix1,
                                  distCoeffs1,
                                  cameraMatrix2,
                                  distCoeffs2,
                                  imageSize1,
                                  R, T, R1, R2, P1, P2, Q,
                                  0, //CV_CALIB_ZERO_DISPARITY or 0
                                  1,
                                  imageSize1,
                                  &validRoi[0],
                                  &validRoi[1]
                                  );*/

                // use intrinsic parameters of each camera, but
                // compute the rectification transformation directly
                // from the fundamental matrix

                vector<Point2f> allimgpt[2];
                std::copy(imagePoints1[0].begin(), imagePoints1[0].end(), back_inserter(allimgpt[0]));
                std::copy(imagePoints2[0].begin(), imagePoints2[0].end(), back_inserter(allimgpt[1]));
                F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
                Mat H1, H2;
                stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F, imageSize1, H1, H2, 3);
                R1 = cameraMatrix1.inv()*H1*cameraMatrix1;
                R2 = cameraMatrix2.inv()*H2*cameraMatrix2;
                P1 = cameraMatrix1;
                P2 = cameraMatrix2;

                //stereoRectify(CM1, D1, CM2, D2, image1_1.size(), R, T, R1, R2, P1, P2, Q);

                qDebug() << "Done Rectification";
                qDebug() << "Q: " << matToString(Q);

                qDebug() << "R1" << matToString(R1);
                qDebug() << "P1" << matToString(P1);
                qDebug() << "R2" << matToString(R2);
                qDebug() << "P2" << matToString(P2);

                //Sleep(uint(1000));

                //CV_16SC2 CV_32FC1
                initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, imageSize1, CV_32FC1, rmap1x, rmap1y);
                initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, imageSize2, CV_32FC1, rmap2x, rmap2y);
                //initUndistortRectifyMap(CM1, D1, R1, P1, image1_1.size(), CV_32FC1, rmap1x, rmap1y);
                //initUndistortRectifyMap(CM2, D2, R2, P2, image2_1.size(), CV_32FC1, rmap2x, rmap2y);

                //qDebug() << "rmap1" << matToString(rmap1);
                //qDebug() << "rmap2" << matToString(rmap2);
                qDebug() << "Done initUndistortRectifyMap";

                qDebug() << "cameraMatrix1" << matToString(cameraMatrix1);
                qDebug() << "distCoeffs1" << matToString(distCoeffs1);
                qDebug() << "cameraMatrix2" << matToString(cameraMatrix2);
                qDebug() << "distCoeffs2" << matToString(distCoeffs2);

                //cv::destroyAllWindows();//

                isStereoCalibrated = true;
            }
            } else {
                stereoCalibrate(objectPoints, imagePoints1, imagePoints2,
                                cameraMatrix1,
                                distCoeffs1,
                                cameraMatrix2,
                                distCoeffs2,
                                imageSize1, R, T, E, F);
                stereoRectify(cameraMatrix1,
                              distCoeffs1,
                              cameraMatrix2,
                              distCoeffs2,
                              imageSize1,
                              R, T, R1, R2, P1, P2, Q);
                initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, imageSize1, CV_32FC1, rmap1x, rmap1y);
                initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, imageSize2, CV_32FC1, rmap2x, rmap2y);
                qDebug() << "SEREOCALIBRATED";
                qDebug() << "cameraMatrix1" << matToString(cameraMatrix1);
                qDebug() << "distCoeffs1" << matToString(distCoeffs1);
                qDebug() << "cameraMatrix2" << matToString(cameraMatrix2);
                qDebug() << "distCoeffs2" << matToString(distCoeffs2);
                qDebug() << "R1" << matToString(R1);
                qDebug() << "P1" << matToString(P1);
                qDebug() << "R2" << matToString(R2);
                qDebug() << "P2" << matToString(P2);
                isStereoCalibrated = true;
            }
        }

        if (key == 'd') {
            qDebug() << "cameraMatrix1" << matToString(cameraMatrix1);
            qDebug() << "distCoeffs1" << matToString(distCoeffs1);
            qDebug() << "cameraMatrix2" << matToString(cameraMatrix2);
            qDebug() << "distCoeffs2" << matToString(distCoeffs2);
            qDebug() << "R1" << matToString(R1);
            qDebug() << "P1" << matToString(P1);
            qDebug() << "R2" << matToString(R2);
            qDebug() << "P2" << matToString(P2);
        }

        //showing
        if (isStereoCalibrated/* && key == 'g'*/) {
            qDebug() << "showing";

            Mat rimage1, rimage2;
            Mat disp, disp8;

            //initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, imageSize1, CV_32FC1, rmap1x, rmap1y);
            //initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, imageSize2, CV_32FC1, rmap2x, rmap2y);

            /*qDebug() << "cameraMatrix1" << matToString(cameraMatrix1);
            qDebug() << "distCoeffs1" << matToString(distCoeffs1);
            qDebug() << "cameraMatrix2" << matToString(cameraMatrix2);
            qDebug() << "distCoeffs2" << matToString(distCoeffs2);
            qDebug() << "R1" << matToString(R1);
            qDebug() << "P1" << matToString(P1);
            qDebug() << "R2" << matToString(R2);
            qDebug() << "P2" << matToString(P2);*/

            remap(image1_1, rimage1, rmap1x, rmap1y, CV_INTER_LINEAR);
            remap(image2_1, rimage2, rmap2x, rmap2y, CV_INTER_LINEAR);


            //resize(rimage1, rimage1, image1_2.size(), 0, 0, CV_INTER_LINEAR);//
            //resize(rimage2, rimage2, image2_2.size(), 0, 0, CV_INTER_LINEAR);//

            //sgbm(rimage1, rimage2, disp);
            //normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);

            if (isShowing) {
                //resize(disp8, disp8, image1_2.size(), 0, 0, CV_INTER_LINEAR);//!
                //imshow("disp", disp8);

                rectangle(rimage1, validRoi[0], Scalar(0, 255, 0), 3);
                rectangle(rimage2, validRoi[1], Scalar(0, 255, 0), 3);
                resize(rimage1, rimage1, image1_2.size(), 0, 0, CV_INTER_LINEAR);
                resize(rimage2, rimage2, image2_2.size(), 0, 0, CV_INTER_LINEAR);
                imshow("rimage1", rimage1);
                imshow("rimage2", rimage2);

                //d
                resize(image1_1, image1_1, image1_2.size(), 0, 0, CV_INTER_LINEAR);
                resize(image2_1, image2_1, image2_2.size(), 0, 0, CV_INTER_LINEAR);
                imshow("image1", image1_1);
                imshow("image2", image2_1);
            }
        } else {
            if (isShowing) {
                resize(image1_1, image1_1, image1_2.size(), 0, 0, CV_INTER_LINEAR);
                resize(image2_1, image2_1, image2_2.size(), 0, 0, CV_INTER_LINEAR);
                imshow("image1", image1_1);
                imshow("image2", image2_1);
            }
        }

        // wait key
        key = waitKey(30);
    }
}

template<class T>
QString templateMatToString(Mat mat) {
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

QString matToString(Mat mat) {
    if (mat.empty()) {
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

bool Stereoscopy::addImage(const Mat im, vector<Point2f> *imageCorners, Mat &result) {
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

void Stereoscopy::checkProjectPoints(string fn1, string fn2) {
    Mat image1 = imread(fn1);
    if (!image1.data) {
        qDebug() <<  "Could not open or find the image1";
        return ;
    }
    Mat image2 = imread(fn2);
    if (!image2.data) {
        qDebug() <<  "Could not open or find the image2";
        return ;
    }

    Mat cameraMatrix1 = (Mat_<double>(3,3) << 1927.04,0,917.475,0,1706.88,579.804,0,0,1);
    Mat distCoeffs1 = (Mat_<double>(1,5) << -0.215192,0.171866,0.0118907,0.00368162,0);
    Size imageSize1 = image1.size();
    Mat cameraMatrix2 = (Mat_<double>(3,3) << 1766.16,0,817.485,0,1564.63,562.498,0,0,1);
    Mat distCoeffs2 = (Mat_<double>(1,5) << -0.19803,0.0611625,-0.00181152,-0.000889885,0);
    //Size imageSize2 = image2.size();

    qDebug() << "cameraMatrix1" << matToString(cameraMatrix1);
    qDebug() << "distCoeffs1" << matToString(distCoeffs1);
    qDebug() << "cameraMatrix2" << matToString(cameraMatrix2);
    qDebug() << "distCoeffs2" << matToString(distCoeffs2);

    vector<Point2f> corners1, corners2;

    if (addImage(image1, &corners1, scres) && addImage(image2, &corners2, scres)) {
        imagePoints1.clear();
        imagePoints2.clear();
        objectPoints.clear();
        imagePoints1.push_back(corners1);
        imagePoints2.push_back(corners2);

        int n = BOARD_WIDTH * BOARD_HEIGHT;
        vector<Point3f> obj;
        for (int j = 0; j < n; j++) {
            obj.push_back(Point3f(j % BOARD_WIDTH, j / BOARD_WIDTH, 0.0f));
        }
        objectPoints.push_back(obj);

        Mat cameraMatrix = Mat(3, 3, CV_32FC1);
        Mat distCoeffs;
        vector<Mat> rvecs, tvecs;

        cameraMatrix.at<float>(0, 0) = 1;
        cameraMatrix.at<float>(1, 1) = 1;

        calibrateCamera(objectPoints, imagePoints1, imageSize1, cameraMatrix, distCoeffs, rvecs, tvecs);

        vector<Point3f> objpoints;
        for (int j = 0; j < n; j++) {
            objpoints.push_back(Point3f(j % BOARD_WIDTH, j / BOARD_WIDTH, 0.0f));
        }

        qDebug() << "Done creation objectPoints";

        vector<Point2f> imagePoints;
        qDebug() << "Start projectPoints";

        qDebug() << "cameraMatrix1: " << matToString(cameraMatrix1);
        qDebug() << "distCoeffs1: " << matToString(distCoeffs1);
        qDebug() << "cameraMatrix: " << matToString(cameraMatrix);
        qDebug() << "distCoeffs: " << matToString(distCoeffs);
        qDebug() << "rvecs[0]: " << matToString(rvecs[0]);
        qDebug() << "tvecs[0]: " << matToString(tvecs[0]);

        projectPoints(Mat(objpoints), rvecs[0], tvecs[0], cameraMatrix, distCoeffs, imagePoints); //rT

        Size pattern_size = Size(BOARD_WIDTH, BOARD_HEIGHT);
        drawChessboardCorners(image1, pattern_size, imagePoints, true);

        //resize(image1, image1, image1.size(), 0, 0, CV_INTER_LINEAR);
        imshow("image1", image1);

    }

    char key = 0;
    while (key != 'q') {
        key = waitKey(30);
    }
}

void Stereoscopy::checkDisparityMap(string fn1, string fn2) {
    Mat image1 = imread(fn1);
    if (!image1.data) {
        qDebug() <<  "Could not open or find the image1";
        return ;
    }
    Mat image2 = imread(fn2);
    if (!image2.data) {
        qDebug() <<  "Could not open or find the image2";
        return ;
    }

    Size imageSize1 = image1.size();
    Size imageSize2 = image2.size();
    Mat imageSmall = Mat(image1.rows / 4, image1.cols / 4, CV_8UC3);
    Size imageSizeSmall = imageSmall.size(); //400x300

    /*
    Mat cameraMatrix1 = (Mat_<double>(3,3) << 1927.04,0,917.475,0,1706.88,579.804,0,0,1);
    Mat distCoeffs1 = (Mat_<double>(1,5) << -0.215192,0.171866,0.0118907,0.00368162,0);
    Mat cameraMatrix2 = (Mat_<double>(3,3) << 1766.16,0,817.485,0,1564.63,562.498,0,0,1);
    Mat distCoeffs2 = (Mat_<double>(1,5) << -0.19803,0.0611625,-0.00181152,-0.000889885,0);*/

    /*Mat cameraMatrix1 = (Mat_<double>(3,3) << 6413.91,0,810.516,0,5364.43,717.19,0,0,1);
    Mat distCoeffs1 = (Mat_<double>(1,5) << -3.62446,83.4373,-0.0860216,0.0135343,-2475.21);
    Mat cameraMatrix2 = (Mat_<double>(3,3) << 13175.3,0,787.325,0,12363.5,646.243,0,0,1);
    Mat distCoeffs2 = (Mat_<double>(1,5) << -14.8809,391.48,-0.103346,-0.0954315,3172.4);*/

    /*Mat cameraMatrix1 = Mat::eye(3, 3, CV_64F);
    Mat distCoeffs1;
    Mat cameraMatrix2 = Mat::eye(3, 3, CV_64F);
    Mat distCoeffs2;*/

    //cal1
    Mat cameraMatrix1 = (Mat_<double>(3,3) << 1806.53,0,815.786,0,1595.14,590.314,0,0,1);
    Mat distCoeffs1 = (Mat_<double>(1,5) << -0.267514,0.213748,0.00136627,0.000194796,0);
    Mat cameraMatrix2 = (Mat_<double>(3,3) << 1739.3,0,808.929,0,1542.11,581.767,0,0,1);
    Mat distCoeffs2 = (Mat_<double>(1,5) << -0.247249,0.161344,-0.00280154,0.000444185,0);

    qDebug() << "cameraMatrix1" << matToString(cameraMatrix1);
    qDebug() << "distCoeffs1" << matToString(distCoeffs1);
    qDebug() << "cameraMatrix2" << matToString(cameraMatrix2);
    qDebug() << "distCoeffs2" << matToString(distCoeffs2);

    vector<Point2f> corners1, corners2;

    Mat E, F;
    Mat R1, R2, P1, P2; //Q

    if (addImage(image1, &corners1, scres) && addImage(image2, &corners2, scres)) {
        imagePoints1.clear();
        imagePoints2.clear();
        objectPoints.clear();
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

        //stereocalibrate

        stereoCalibrate(objectPoints, imagePoints1, imagePoints2,
                        cameraMatrix1,
                        distCoeffs1,
                        cameraMatrix2,
                        distCoeffs2,
                        imageSize1, R, T, E, F,
                        TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 50, 1e-6),
                        CV_CALIB_FIX_INTRINSIC +
                        CV_CALIB_USE_INTRINSIC_GUESS//*/
                                                /*TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-8),
                                                CV_CALIB_ZERO_TANGENT_DIST +
                                                CV_CALIB_FIX_INTRINSIC+
                                                CV_CALIB_FIX_K3*/
                        /*TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
                                                CV_CALIB_SAME_FOCAL_LENGTH | CV_CALIB_ZERO_TANGENT_DIST*/
                                            /*, //good
                                            TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
                                            CV_CALIB_FIX_ASPECT_RATIO +
                                            CV_CALIB_ZERO_TANGENT_DIST +
                                            CV_CALIB_SAME_FOCAL_LENGTH +
                                            CV_CALIB_RATIONAL_MODEL +
                                            CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5*/
                        );

        /*stereoCalibrate(object_points, imagePoints1, imagePoints2,
                CM1, D1, CM2, D2, img1.size(), R, T, E, F,
                cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
                CV_CALIB_SAME_FOCAL_LENGTH | CV_CALIB_ZERO_TANGENT_DIST);*/

        qDebug() << "R" << matToString(R);
        qDebug() << "T" << matToString(T);

        cv::Rect validRoi[2];

        qDebug() << "cameraMatrix1" << matToString(cameraMatrix1);
        qDebug() << "distCoeffs1" << matToString(distCoeffs1);
        qDebug() << "cameraMatrix2" << matToString(cameraMatrix2);
        qDebug() << "distCoeffs2" << matToString(distCoeffs2);

        qDebug() << "Starting Rectification";
        cv::stereoRectify(cameraMatrix1,
                          distCoeffs1,
                          cameraMatrix2,
                          distCoeffs2,
                          imageSize1,
                          R, T, R1, R2, P1, P2, Q/*,
                          CV_CALIB_ZERO_DISPARITY, 1,
                          imageSize1,
                          &validRoi[0],
                          &validRoi[1]*/
                          );
        qDebug() << "Done Rectification";
        qDebug() << "Q: " << matToString(Q);

        qDebug() << "R1" << matToString(R1);
        qDebug() << "P1" << matToString(P1);
        qDebug() << "R2" << matToString(R2);
        qDebug() << "P2" << matToString(P2);

        //Sleep(uint(1000));

        //CV_16SC2 CV_32FC1
        //cv::initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, imageSize1, CV_32FC1, rmap1, rmap2);
        //cv::initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, imageSize2, CV_32FC1, rmap1, rmap2);
        //qDebug() << "rmap1" << matToString(rmap1);
        //qDebug() << "rmap2" << matToString(rmap2);
        //qDebug() << "Done initUndistortRectifyMap";

        //cv::destroyAllWindows();//

    } else {
        qDebug() << "Didn't stereocalibration";
        return;
    }

    Mat rimage1, rimage2;
    Mat disp, disp8;

    initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, imageSize1, CV_32FC1, rmap1x, rmap1y);
    initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, imageSize2, CV_32FC1, rmap2x, rmap2y);

    remap(image1, rimage1, rmap1x, rmap1y, CV_INTER_LINEAR);
    remap(image2, rimage2, rmap2x, rmap2y, CV_INTER_LINEAR);

    StereoBM sbm;
    sbm.state->SADWindowSize = 9; //5 or 9
    sbm.state->numberOfDisparities = 16; // 64 //112  //must divisible 16
    sbm.state->preFilterSize = 5; //5
    sbm.state->preFilterCap = 32; //61
    sbm.state->minDisparity = 20; //-39
    sbm.state->textureThreshold = 200; //507
    sbm.state->uniquenessRatio = 0; //0
    sbm.state->speckleWindowSize = 0; //0
    sbm.state->speckleRange = 0; //8
    sbm.state->disp12MaxDiff = 1; //1

    sgbm.SADWindowSize = 5; //5
    sgbm.numberOfDisparities = 192; //192
    sgbm.preFilterCap = 4; //4
    sgbm.minDisparity = -64; //-64
    sgbm.uniquenessRatio = 1; //1
    sgbm.speckleWindowSize = 150; //150
    sgbm.speckleRange = 2; //
    sgbm.disp12MaxDiff = 10; //10
    sgbm.fullDP = false; //false
    sgbm.P1 = 600; //600
    sgbm.P2 = 2400; //2400

    //Mat g1, g2;

    cvtColor(rimage1, g1, CV_BGR2GRAY);
    cvtColor(rimage2, g2, CV_BGR2GRAY);

    //sbm(g1, g2, disp);
    sgbm(g1, g2, disp);

    //sgbm(rimage1, rimage2, disp);
    //sgbm(image1, image2, disp);
    normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);

    resize(disp8, disp8, imageSizeSmall, 0, 0, CV_INTER_LINEAR);
    imshow("disp", disp8);

    resize(rimage1, rimage1, imageSizeSmall, 0, 0, CV_INTER_LINEAR);
    resize(rimage2, rimage2, imageSizeSmall, 0, 0, CV_INTER_LINEAR);
    imshow("rimage1", rimage1);
    imshow("rimage2", rimage2);

    /*char key = 0;
    while (key != 'q') {
        key = waitKey(30);
    }*/
}

void Stereoscopy::checkDisparityMapFromCapture() {
    Image rawImage1;
    Error error = camera.RetrieveBuffer(&rawImage1);
    if (error != PGRERROR_OK) {
        qDebug() << "Capture error";
        return;
    }
    Image rgbImage1;
    rawImage1.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage1);
    // camera1: convert to OpenCV Mat
    unsigned int rowBytes = (double)rgbImage1.GetReceivedDataSize() / (double)rgbImage1.GetRows();
    Mat image1 = Mat(rgbImage1.GetRows(), rgbImage1.GetCols(), CV_8UC3, rgbImage1.GetData(), rowBytes);

    Image rawImage2;
    error = camera2.RetrieveBuffer(&rawImage2);
    if (error != PGRERROR_OK) {
        qDebug() << "Capture error";
        return;
    }
    Image rgbImage2;
    rawImage2.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage2);
    rowBytes = (double)rgbImage2.GetReceivedDataSize() / (double)rgbImage2.GetRows();
    Mat image2 = Mat(rgbImage2.GetRows(), rgbImage2.GetCols(), CV_8UC3, rgbImage2.GetData(),rowBytes);

    Size imageSize1 = image1.size();
    Size imageSize2 = image2.size();
    Mat imageSmall = Mat(image1.rows / 4, image1.cols / 4, CV_8UC3);
    Size imageSizeSmall = imageSmall.size(); //400x300

    /*
    Mat cameraMatrix1 = (Mat_<double>(3,3) << 1927.04,0,917.475,0,1706.88,579.804,0,0,1);
    Mat distCoeffs1 = (Mat_<double>(1,5) << -0.215192,0.171866,0.0118907,0.00368162,0);
    Mat cameraMatrix2 = (Mat_<double>(3,3) << 1766.16,0,817.485,0,1564.63,562.498,0,0,1);
    Mat distCoeffs2 = (Mat_<double>(1,5) << -0.19803,0.0611625,-0.00181152,-0.000889885,0);*/

    /*Mat cameraMatrix1 = (Mat_<double>(3,3) << 6413.91,0,810.516,0,5364.43,717.19,0,0,1);
    Mat distCoeffs1 = (Mat_<double>(1,5) << -3.62446,83.4373,-0.0860216,0.0135343,-2475.21);
    Mat cameraMatrix2 = (Mat_<double>(3,3) << 13175.3,0,787.325,0,12363.5,646.243,0,0,1);
    Mat distCoeffs2 = (Mat_<double>(1,5) << -14.8809,391.48,-0.103346,-0.0954315,3172.4);*/

    /*Mat cameraMatrix1 = Mat::eye(3, 3, CV_64F);
    Mat distCoeffs1;
    Mat cameraMatrix2 = Mat::eye(3, 3, CV_64F);
    Mat distCoeffs2;*/

    //cal1 ??
    /*Mat cameraMatrix1 = (Mat_<double>(3,3) << 6413.91,0,810.516,0,5364.43,717.19,0,0,1);
    Mat distCoeffs1 = (Mat_<double>(1,5) << -3.62446,83.4373,-0.0860216,0.0135343,-2475.21);
    Mat cameraMatrix2 = (Mat_<double>(3,3) << 13175.3,0,787.325,0,12363.5,646.243,0,0,1);
    Mat distCoeffs2 = (Mat_<double>(1,5) << -14.8809,391.48,-0.103346,-0.0954315,3172.4);*/

    //cal1
    Mat cameraMatrix1 = (Mat_<double>(3,3) << 1806.53,0,815.786,0,1595.14,590.314,0,0,1);
    Mat distCoeffs1 = (Mat_<double>(1,5) << -0.267514,0.213748,0.00136627,0.000194796,0);
    Mat cameraMatrix2 = (Mat_<double>(3,3) << 1739.3,0,808.929,0,1542.11,581.767,0,0,1);
    Mat distCoeffs2 = (Mat_<double>(1,5) << -0.247249,0.161344,-0.00280154,0.000444185,0);

    qDebug() << "cameraMatrix1" << matToString(cameraMatrix1);
    qDebug() << "distCoeffs1" << matToString(distCoeffs1);
    qDebug() << "cameraMatrix2" << matToString(cameraMatrix2);
    qDebug() << "distCoeffs2" << matToString(distCoeffs2);

    vector<Point2f> corners1, corners2;

    Mat E, F;
    Mat R1, R2, P1, P2; //Q

    if (addImage(image1, &corners1, scres) && addImage(image2, &corners2, scres)) {
        imagePoints1.clear();
        imagePoints2.clear();
        objectPoints.clear();
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

        //stereocalibrate

        stereoCalibrate(objectPoints, imagePoints1, imagePoints2,
                        cameraMatrix1,
                        distCoeffs1,
                        cameraMatrix2,
                        distCoeffs2,
                        imageSize1, R, T, E, F/*,
                                                TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-8),
                                                CV_CALIB_ZERO_TANGENT_DIST +
                                                CV_CALIB_FIX_INTRINSIC+
                                                CV_CALIB_FIX_K3*/
                        /*TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
                                                CV_CALIB_SAME_FOCAL_LENGTH | CV_CALIB_ZERO_TANGENT_DIST*/
                                            /*,
                                            TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
                                            CV_CALIB_FIX_ASPECT_RATIO +
                                            CV_CALIB_ZERO_TANGENT_DIST +
                                            CV_CALIB_SAME_FOCAL_LENGTH +
                                            CV_CALIB_RATIONAL_MODEL +
                                            CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5*/);

        /*stereoCalibrate(object_points, imagePoints1, imagePoints2,
                CM1, D1, CM2, D2, img1.size(), R, T, E, F,
                cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
                CV_CALIB_SAME_FOCAL_LENGTH | CV_CALIB_ZERO_TANGENT_DIST);*/

        qDebug() << "R" << matToString(R);
        qDebug() << "T" << matToString(T);

        //cv::Rect validRoi[2];

        qDebug() << "cameraMatrix1" << matToString(cameraMatrix1);
        qDebug() << "distCoeffs1" << matToString(distCoeffs1);
        qDebug() << "cameraMatrix2" << matToString(cameraMatrix2);
        qDebug() << "distCoeffs2" << matToString(distCoeffs2);

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

        qDebug() << "R1" << matToString(R1);
        qDebug() << "P1" << matToString(P1);
        qDebug() << "R2" << matToString(R2);
        qDebug() << "P2" << matToString(P2);

        //CV_16SC2 CV_32FC1
        //cv::initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, imageSize1, CV_32FC1, rmap1, rmap2);
        //cv::initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, imageSize2, CV_32FC1, rmap1, rmap2);
        //qDebug() << "rmap1" << matToString(rmap1);
        //qDebug() << "rmap2" << matToString(rmap2);
        //qDebug() << "Done initUndistortRectifyMap";

        //cv::destroyAllWindows();//

    } else {
        qDebug() << "Didn't stereocalibration";
        return;
    }

    Mat rimage1, rimage2;
    Mat disp, disp8;

    initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, imageSize1, CV_32FC1, rmap1x, rmap1y);
    initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, imageSize2, CV_32FC1, rmap2x, rmap2y);

    remap(image1, rimage1, rmap1x, rmap1y, CV_INTER_LINEAR);
    remap(image2, rimage2, rmap2x, rmap2y, CV_INTER_LINEAR);

    StereoBM sbm;
    sbm.state->SADWindowSize = 9; //5 or 9
    sbm.state->numberOfDisparities = 16; // 64 //112  //must divisible 16
    sbm.state->preFilterSize = 5; //5
    sbm.state->preFilterCap = 32; //61
    sbm.state->minDisparity = 20; //-39
    sbm.state->textureThreshold = 200; //507
    sbm.state->uniquenessRatio = 0; //0
    sbm.state->speckleWindowSize = 0; //0
    sbm.state->speckleRange = 0; //8
    sbm.state->disp12MaxDiff = 1; //1

    sgbm.SADWindowSize = 5; //5
    sgbm.numberOfDisparities = 192; //192
    sgbm.preFilterCap = 4; //4
    sgbm.minDisparity = -64; //-64
    sgbm.uniquenessRatio = 1; //1
    sgbm.speckleWindowSize = 150; //150
    sgbm.speckleRange = 2; //
    sgbm.disp12MaxDiff = 10; //10
    sgbm.fullDP = false; //false
    sgbm.P1 = 600; //600
    sgbm.P2 = 2400; //2400

    //Mat g1, g2;

    cvtColor(rimage1, g1, CV_BGR2GRAY);
    cvtColor(rimage2, g2, CV_BGR2GRAY);

/*    //sbm(g1, g2, disp);
    sgbm(g1, g2, disp);

    //sgbm(rimage1, rimage2, disp);
    //sgbm(image1, image2, disp);
    normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);

    resize(disp8, disp8, imageSizeSmall, 0, 0, CV_INTER_LINEAR);
    imshow("disp", disp8);*/

    resize(rimage1, rimage1, imageSizeSmall, 0, 0, CV_INTER_LINEAR);
    resize(rimage2, rimage2, imageSizeSmall, 0, 0, CV_INTER_LINEAR);
    imshow("rimage1", rimage1);
    imshow("rimage2", rimage2);

    /*
    char key = 0;
    while (key != 'q') {
        key = waitKey(30);
    }*/
}

void Stereoscopy::checkDisparityMap2(string fn1, string fn2) {
    Mat image1 = imread(fn1);
    if (!image1.data) {
        qDebug() <<  "Could not open or find the image1";
        return ;
    }
    Mat image2 = imread(fn2);
    if (!image2.data) {
        qDebug() <<  "Could not open or find the image2";
        return ;
    }

    qDebug() << "Start checkDisparityMap2";

    Size imageSize1 = image1.size();
    Size imageSize2 = image2.size();
    Mat imageSmall = Mat(image1.rows / 4, image1.cols / 4, CV_8UC3);
    Size imageSizeSmall = imageSmall.size(); //400x300

    /*Mat cameraMatrix1 = (Mat_<double>(3,3) << 1927.04,0,917.475,0,1706.88,579.804,0,0,1);
    Mat distCoeffs1 = (Mat_<double>(1,5) << -0.215192,0.171866,0.0118907,0.00368162,0);
    Mat cameraMatrix2 = (Mat_<double>(3,3) << 1766.16,0,817.485,0,1564.63,562.498,0,0,1);
    Mat distCoeffs2 = (Mat_<double>(1,5) << -0.19803,0.0611625,-0.00181152,-0.000889885,0);*/

    /*Mat cameraMatrix1 = (Mat_<double>(3,3) << 6413.91,0,810.516,0,5364.43,717.19,0,0,1);
    Mat distCoeffs1 = (Mat_<double>(1,5) << -3.62446,83.4373,-0.0860216,0.0135343,-2475.21);
    Mat cameraMatrix2 = (Mat_<double>(3,3) << 13175.3,0,787.325,0,12363.5,646.243,0,0,1);
    Mat distCoeffs2 = (Mat_<double>(1,5) << -14.8809,391.48,-0.103346,-0.0954315,3172.4);*/

    //a, b
   /* Mat cameraMatrix1 = (Mat_<double>(3,3) << 4013.73,0,900.341,0,4013.73,533.891,0,0,1);
    Mat distCoeffs1 = (Mat_<double>(1,8) << -0.977429,-65.4483,0,0,0,0,0,-2387.05);
    Mat cameraMatrix2 = (Mat_<double>(3,3) << 4013.73,0,900.316,0,4013.73,534.174,0,0,1);
    Mat distCoeffs2 = (Mat_<double>(1,8) << -1.83777,-5.48685,0,0,0,0,0,-585.799);
    Mat R1 = (Mat_<double>(3,3) << 0.905447,-0.0484319,0.421686,0.0476124,0.998788,0.0124802,-0.42178,0.00877729,0.906656);
    Mat P1 = (Mat_<double>(3,4) << 3555.86,0,-997.901,0,0,3555.86,590.519,0,0,0,1,0);
    Mat R2 = (Mat_<double>(3,3) << 0.911022,-0.0516611,0.409109,0.0524549,0.99858,0.00928897,-0.409008,0.0129973,0.912438);
    Mat P2 = (Mat_<double>(3,4) << 3555.86,0,-997.901,-6491.28,0,3555.86,590.519,0,0,0,1,0);*/

    //cal1 ??
   /* Mat cameraMatrix1 = (Mat_<double>(3,3) << 6413.91,0,810.516,0,5364.43,717.19,0,0,1);
    Mat distCoeffs1 = (Mat_<double>(1,5) << -3.62446,83.4373,-0.0860216,0.0135343,-2475.21);
    Mat cameraMatrix2 = (Mat_<double>(3,3) << 13175.3,0,787.325,0,12363.5,646.243,0,0,1);
    Mat distCoeffs2 = (Mat_<double>(1,5) << -14.8809,391.48,-0.103346,-0.0954315,3172.4);
    Mat R1 = (Mat_<double>(3,3) << 0.909211,0.302429,0.286133,-0.399878,0.443026,0.802388,0.115901,-0.843958,0.523739);
    Mat P1 = (Mat_<double>(3,4) << 5848.82,0,-1744.66,0,0,5848.82,-15679.5,0,0,0,1,0);
    Mat R2 = (Mat_<double>(3,3) << 0.945285,0.304903,0.116064,-0.181635,0.196331,0.963568,0.271007,-0.931927,0.24097);
    Mat P2 = (Mat_<double>(3,4) << 5848.82,0,-1744.66,0,0,5848.82,-15679.5,533465,0,0,1,0);
    */
    //cal1
    Mat cameraMatrix1 = (Mat_<double>(3,3) << 1806.53,0,815.786,0,1595.14,590.314,0,0,1);
    Mat distCoeffs1 = (Mat_<double>(1,5) << -0.267514,0.213748,0.00136627,0.000194796,0);
    Mat cameraMatrix2 = (Mat_<double>(3,3) << 1739.3,0,808.929,0,1542.11,581.767,0,0,1);
    Mat distCoeffs2 = (Mat_<double>(1,5) << -0.247249,0.161344,-0.00280154,0.000444185,0);

    /*Mat r = (Mat_<double>(3,3) << 0.999836,-0.0123966,-0.0132168,0.0127787,0.999491,0.0292321,0.0128477,-0.0293962,0.999485);
    Mat t = (Mat_<double>(3,1) << 3.11602,0.325481,-0.507391);

    Mat R1, R2, P1, P2;

    cv::stereoRectify(cameraMatrix1,
                      distCoeffs1,
                      cameraMatrix2,
                      distCoeffs2,
                      imageSize1,
                      r, t, R1, R2, P1, P2, Q
                      );*/

    /*Mat R1 = (Mat_<double>(3,3) << 0.980892,0.0950288,-0.169765,-0.0923844,0.995448,0.02347,0.171218,-0.00729576,0.985206);
    Mat P1 = (Mat_<double>(3,4) << 1381.78,0,1097.9,0,0,1381.78,568.653,0,0,0,1,0);
    Mat R2 = (Mat_<double>(3,3) << 0.981797,0.102552,-0.159869,-0.105019,0.994445,-0.0070343,0.158259,0.0236955,0.987113);
    Mat P2 = (Mat_<double>(3,4) << 1381.78,0,1097.9,4385.48,0,1381.78,568.653,0,0,0,1,0);*/

    Mat R1 = (Mat_<double>(3,3) << 0.974573,0.0471726,-0.219049,-0.0468551,0.99888,0.00664693,0.219117,0.00378565,0.975691);
    Mat P1 = (Mat_<double>(3,4) << 1183.47,0,1184.05,0,0,1183.47,574.579,0,0,0,1,0);
    Mat R2 = (Mat_<double>(3,3) << 0.978186,0.0475122,-0.202225,-0.047805,0.998851,0.00343925,0.202156,0.00630314,0.979333);
    Mat P2 = (Mat_<double>(3,4) << 1183.47,0,1184.05,3670.38,0,1183.47,574.579,0,0,0,1,0);

    qDebug() << "cameraMatrix1" << matToString(cameraMatrix1);
    qDebug() << "distCoeffs1" << matToString(distCoeffs1);
    qDebug() << "cameraMatrix2" << matToString(cameraMatrix2);
    qDebug() << "distCoeffs2" << matToString(distCoeffs2);

    /*
    Mat R1 = (Mat_<double>(3,3) << 0.978727,0.14493,-0.145217,-0.196522,0.458986,-0.866436,-0.0589198,0.876543,0.477704);
    Mat P1 = (Mat_<double>(3,4) << 5848.82,0,-10588.8,0,0,5848.82,4099.75,0,0,0,1,0);
    Mat R2 = (Mat_<double>(3,3) << 0.98485,0.16081,-0.0648815,-0.0976413,0.205062,-0.973866,-0.143303,0.965448,0.217658);
    Mat P2 = (Mat_<double>(3,4) << 5848.82,0,-10588.8,0,0,5848.82,4099.75,-499448,0,0,1,0);
    */


    Mat rimage1, rimage2;
    Mat disp, disp8;

    initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, imageSize1, CV_32FC1, rmap1x, rmap1y);
    initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, imageSize2, CV_32FC1, rmap2x, rmap2y);

    /*qDebug() << "rmap1x" << matToString(rmap1x);
    qDebug() << "rmap1y" << matToString(rmap1y);
    qDebug() << "rmap2x" << matToString(rmap2x);
    qDebug() << "rmap2y" << matToString(rmap2y);*/

    qDebug() << "Start remap";

    //remap problems

    remap(image1, rimage1, rmap1x, rmap1y, CV_INTER_LINEAR);
    remap(image2, rimage2, rmap2x, rmap2y, CV_INTER_LINEAR);

    resize(image1, image1, imageSizeSmall, 0, 0, CV_INTER_LINEAR);
    resize(image2, image2, imageSizeSmall, 0, 0, CV_INTER_LINEAR);
    imshow("image1", image1);
    imshow("image2", image2);

    resize(rimage1, rimage1, imageSizeSmall, 0, 0, CV_INTER_LINEAR);
    resize(rimage2, rimage2, imageSizeSmall, 0, 0, CV_INTER_LINEAR);
    imshow("rimage1", rimage1);
    imshow("rimage2", rimage2);

    qDebug() << "cameraMatrix1" << matToString(cameraMatrix1);
    qDebug() << "distCoeffs1" << matToString(distCoeffs1);
    qDebug() << "cameraMatrix2" << matToString(cameraMatrix2);
    qDebug() << "distCoeffs2" << matToString(distCoeffs2);
    qDebug() << "R1" << matToString(R1);
    qDebug() << "P1" << matToString(P1);
    qDebug() << "R2" << matToString(R2);
    qDebug() << "P2" << matToString(P2);

    //Mat g1, g2;

    cvtColor(rimage1, g2, CV_BGR2GRAY);
    cvtColor(rimage2, g1, CV_BGR2GRAY);

    qDebug() << "Start StereoBM";

    StereoBM sbm;
    /*sbm.state->SADWindowSize = 9;
    sbm.state->numberOfDisparities = 112; //112
    sbm.state->preFilterSize = 5;
    sbm.state->preFilterCap = 61;
    sbm.state->minDisparity = -39;
    sbm.state->textureThreshold = 507;
    sbm.state->uniquenessRatio = 0;
    sbm.state->speckleWindowSize = 0;
    sbm.state->speckleRange = 8;
    sbm.state->disp12MaxDiff = 1;*/

    sbm.state->SADWindowSize = 9;
    sbm.state->numberOfDisparities = 128; //112
    sbm.state->preFilterSize = 5;
    sbm.state->preFilterCap = 61;
    sbm.state->minDisparity = -39;
    sbm.state->textureThreshold = 507;
    sbm.state->uniquenessRatio = 0;
    sbm.state->speckleWindowSize = 0;
    sbm.state->speckleRange = 8;
    sbm.state->disp12MaxDiff = 10;

    sgbm.SADWindowSize = 5; //5
    sgbm.numberOfDisparities = 128; //192
    sgbm.preFilterCap = 1; //4
    sgbm.minDisparity = 0; //-64
    sgbm.uniquenessRatio = 1; //1
    sgbm.speckleWindowSize = 150; //150
    sgbm.speckleRange = 2; //2
    sgbm.disp12MaxDiff = 2; //10
    sgbm.fullDP = false; //false
    sgbm.P1 = 400; //600
    sgbm.P2 = 1600; //2400

    qDebug() << "Start sbm";

    sbm(g1, g2, disp);
    //sgbm(g1, g2, disp);

    qDebug() << "Start normalize";
    normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);

    qDebug() << "Start 123";

    resize(disp8, disp8, imageSizeSmall, 0, 0, CV_INTER_LINEAR);
    imshow("disp", disp8);

    /*char key = 0;
    while (key != 'q') {
        key = waitKey(30);
    }*/
}

void Stereoscopy::checkDisparityMapFromCapture2() {
    Image rawImage1;
    Error error = camera.RetrieveBuffer(&rawImage1);
    if (error != PGRERROR_OK) {
        qDebug() << "Capture error";
        return;
    }
    Image rgbImage1;
    rawImage1.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage1);
    // camera1: convert to OpenCV Mat
    unsigned int rowBytes = (double)rgbImage1.GetReceivedDataSize() / (double)rgbImage1.GetRows();
    Mat image1 = Mat(rgbImage1.GetRows(), rgbImage1.GetCols(), CV_8UC3, rgbImage1.GetData(), rowBytes);

    Image rawImage2;
    error = camera2.RetrieveBuffer(&rawImage2);
    if (error != PGRERROR_OK) {
        qDebug() << "Capture error";
        return;
    }
    Image rgbImage2;
    rawImage2.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage2);
    rowBytes = (double)rgbImage2.GetReceivedDataSize() / (double)rgbImage2.GetRows();
    Mat image2 = Mat(rgbImage2.GetRows(), rgbImage2.GetCols(), CV_8UC3, rgbImage2.GetData(),rowBytes);

    Size imageSize1 = image1.size();
    Size imageSize2 = image2.size();
    Mat imageSmall = Mat(image1.rows / 4, image1.cols / 4, CV_8UC3);
    Size imageSizeSmall = imageSmall.size(); //400x300

    /*Mat cameraMatrix1 = (Mat_<double>(3,3) << 1927.04,0,917.475,0,1706.88,579.804,0,0,1);
    Mat distCoeffs1 = (Mat_<double>(1,5) << -0.215192,0.171866,0.0118907,0.00368162,0);
    Mat cameraMatrix2 = (Mat_<double>(3,3) << 1766.16,0,817.485,0,1564.63,562.498,0,0,1);
    Mat distCoeffs2 = (Mat_<double>(1,5) << -0.19803,0.0611625,-0.00181152,-0.000889885,0);*/

    /*Mat cameraMatrix1 = (Mat_<double>(3,3) << 6413.91,0,810.516,0,5364.43,717.19,0,0,1);
    Mat distCoeffs1 = (Mat_<double>(1,5) << -3.62446,83.4373,-0.0860216,0.0135343,-2475.21);
    Mat cameraMatrix2 = (Mat_<double>(3,3) << 13175.3,0,787.325,0,12363.5,646.243,0,0,1);
    Mat distCoeffs2 = (Mat_<double>(1,5) << -14.8809,391.48,-0.103346,-0.0954315,3172.4);*/

    //a, b
   /* Mat cameraMatrix1 = (Mat_<double>(3,3) << 4013.73,0,900.341,0,4013.73,533.891,0,0,1);
    Mat distCoeffs1 = (Mat_<double>(1,8) << -0.977429,-65.4483,0,0,0,0,0,-2387.05);
    Mat cameraMatrix2 = (Mat_<double>(3,3) << 4013.73,0,900.316,0,4013.73,534.174,0,0,1);
    Mat distCoeffs2 = (Mat_<double>(1,8) << -1.83777,-5.48685,0,0,0,0,0,-585.799);
    Mat R1 = (Mat_<double>(3,3) << 0.905447,-0.0484319,0.421686,0.0476124,0.998788,0.0124802,-0.42178,0.00877729,0.906656);
    Mat P1 = (Mat_<double>(3,4) << 3555.86,0,-997.901,0,0,3555.86,590.519,0,0,0,1,0);
    Mat R2 = (Mat_<double>(3,3) << 0.911022,-0.0516611,0.409109,0.0524549,0.99858,0.00928897,-0.409008,0.0129973,0.912438);
    Mat P2 = (Mat_<double>(3,4) << 3555.86,0,-997.901,-6491.28,0,3555.86,590.519,0,0,0,1,0);*/

    //cal1 ??
    Mat cameraMatrix1 = (Mat_<double>(3,3) << 1806.53,0,815.786,0,1595.14,590.314,0,0,1);
    Mat distCoeffs1 = (Mat_<double>(1,5) << -0.267514,0.213748,0.00136627,0.000194796,0);
    Mat cameraMatrix2 = (Mat_<double>(3,3) << 1739.3,0,808.929,0,1542.11,581.767,0,0,1);
    Mat distCoeffs2 = (Mat_<double>(1,5) << -0.247249,0.161344,-0.00280154,0.000444185,0);

    /*Mat cameraMatrix1 = (Mat_<double>(3,3) << 6413.91,0,810.516,0,5364.43,717.19,0,0,1);
    Mat distCoeffs1 = (Mat_<double>(1,5) << -3.62446,83.4373,-0.0860216,0.0135343,-2475.21);
    Mat cameraMatrix2 = (Mat_<double>(3,3) << 13175.3,0,787.325,0,12363.5,646.243,0,0,1);
    Mat distCoeffs2 = (Mat_<double>(1,5) << -14.8809,391.48,-0.103346,-0.0954315,3172.4);*/

    Mat R1 = (Mat_<double>(3,3) << 0.980892,0.0950288,-0.169765,-0.0923844,0.995448,0.023427,0.171218,-0.00729576,0.985206);
    Mat P1 = (Mat_<double>(3,4) << 1381.78,0,1097.9,0,0,1381.78,568.653,0,0,0,1,0);
    Mat R2 = (Mat_<double>(3,3) << 0.981797,0.102552,-0.159869,-0.105019,0.994445,-0.0070343,0.158259,0.0236955,0.987113);
    Mat P2 = (Mat_<double>(3,4) << 1381.78,0,1097.9,4385.48,0,1381.78,568.653,0,0,0,1,0);

    /*Mat R1 = (Mat_<double>(3,3) << 0.909211,0.302429,0.286133,-0.399878,0.443026,0.802388,0.115901,-0.843958,0.523739);
    Mat P1 = (Mat_<double>(3,4) << 5848.82,0,-1744.66,0,0,5848.82,-15679.5,0,0,0,1,0);
    Mat R2 = (Mat_<double>(3,3) << 0.945285,0.304903,0.116064,-0.181635,0.196331,0.963568,0.271007,-0.931927,0.24097);
    Mat P2 = (Mat_<double>(3,4) << 5848.82,0,-1744.66,0,0,5848.82,-15679.5,533465,0,0,1,0);*/

    qDebug() << "cameraMatrix1" << matToString(cameraMatrix1);
    qDebug() << "distCoeffs1" << matToString(distCoeffs1);
    qDebug() << "cameraMatrix2" << matToString(cameraMatrix2);
    qDebug() << "distCoeffs2" << matToString(distCoeffs2);

    /*
    Mat R1 = (Mat_<double>(3,3) << 0.978727,0.14493,-0.145217,-0.196522,0.458986,-0.866436,-0.0589198,0.876543,0.477704);
    Mat P1 = (Mat_<double>(3,4) << 5848.82,0,-10588.8,0,0,5848.82,4099.75,0,0,0,1,0);
    Mat R2 = (Mat_<double>(3,3) << 0.98485,0.16081,-0.0648815,-0.0976413,0.205062,-0.973866,-0.143303,0.965448,0.217658);
    Mat P2 = (Mat_<double>(3,4) << 5848.82,0,-10588.8,0,0,5848.82,4099.75,-499448,0,0,1,0);
    */


    Mat rimage1, rimage2;
    Mat disp, disp8;

    initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, imageSize1, CV_32FC1, rmap1x, rmap1y);
    initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, imageSize2, CV_32FC1, rmap2x, rmap2y);

    /*qDebug() << "rmap1x" << matToString(rmap1x);
    qDebug() << "rmap1y" << matToString(rmap1y);
    qDebug() << "rmap2x" << matToString(rmap2x);
    qDebug() << "rmap2y" << matToString(rmap2y);*/

    qDebug() << "Start remap";

    //remap problems

    remap(image1, rimage1, rmap1x, rmap1y, CV_INTER_LINEAR);
    remap(image2, rimage2, rmap2x, rmap2y, CV_INTER_LINEAR);

    /*resize(image1, image1, imageSizeSmall, 0, 0, CV_INTER_LINEAR);
    resize(image2, image2, imageSizeSmall, 0, 0, CV_INTER_LINEAR);
    imshow("image1", image1);
    imshow("image2", image2);*/

    resize(rimage1, rimage1, imageSizeSmall, 0, 0, CV_INTER_LINEAR);
    resize(rimage2, rimage2, imageSizeSmall, 0, 0, CV_INTER_LINEAR);
    imshow("rimage1", rimage1);
    imshow("rimage2", rimage2);

    qDebug() << "cameraMatrix1" << matToString(cameraMatrix1);
    qDebug() << "distCoeffs1" << matToString(distCoeffs1);
    qDebug() << "cameraMatrix2" << matToString(cameraMatrix2);
    qDebug() << "distCoeffs2" << matToString(distCoeffs2);
    qDebug() << "R1" << matToString(R1);
    qDebug() << "P1" << matToString(P1);
    qDebug() << "R2" << matToString(R2);
    qDebug() << "P2" << matToString(P2);

    //Mat g1, g2;

    cvtColor(rimage1, g2, CV_BGR2GRAY);
    cvtColor(rimage2, g1, CV_BGR2GRAY);

    qDebug() << "Start StereoBM";

    /*StereoBM sbm;
    sbm.state->SADWindowSize = 9;
    sbm.state->numberOfDisparities = 112; //112
    sbm.state->preFilterSize = 5;
    sbm.state->preFilterCap = 61;
    sbm.state->minDisparity = -39;
    sbm.state->textureThreshold = 507;
    sbm.state->uniquenessRatio = 0;
    sbm.state->speckleWindowSize = 0;
    sbm.state->speckleRange = 8;
    sbm.state->disp12MaxDiff = 1;*/

    sgbm.SADWindowSize = 5; //5
    sgbm.numberOfDisparities = 128; //192
    sgbm.preFilterCap = 1; //4
    sgbm.minDisparity = 0; //-64
    sgbm.uniquenessRatio = 1; //1
    sgbm.speckleWindowSize = 150; //150
    sgbm.speckleRange = 2; //2
    sgbm.disp12MaxDiff = 2; //10
    sgbm.fullDP = false; //false
    sgbm.P1 = 400; //600
    sgbm.P2 = 1600; //2400

    qDebug() << "Start sbm";

    //sbm(g1, g2, disp);
    sgbm(g1, g2, disp);

    qDebug() << "Start normalize";
    normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);

    qDebug() << "Start 123";

    resize(disp8, disp8, imageSizeSmall, 0, 0, CV_INTER_LINEAR);
    imshow("disp", disp8);

    /*char key = 0;
    while (key != 'q') {
        key = waitKey(30);
    }*/
}

void Stereoscopy::showDisparityMap() {
    Mat disp, disp8;

    sgbm(g1, g2, disp);

    qDebug() << "Start normalize";
    normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);

    qDebug() << "Start 123";

    resize(disp8, disp8, Size(400, 300), 0, 0, CV_INTER_LINEAR);
    imshow("disp", disp8);
}

void Stereoscopy::wait() {
    char key = 0;
    while (key != 'q') {
        key = waitKey(30);
    }
}

void Stereoscopy::checkUndistort(string fn1, string fn2) {
    Mat image1 = imread(fn1);
    if (!image1.data) {
        qDebug() <<  "Could not open or find the image1";
        return ;
    }
    Mat image2 = imread(fn2);
    if (!image2.data) {
        qDebug() <<  "Could not open or find the image2";
        return ;
    }

    Mat imageSmall = Mat(image1.rows / 4, image1.cols / 4, CV_8UC3);
    Size imageSizeSmall = imageSmall.size(); //400x300

    Mat cameraMatrix1 = (Mat_<double>(3,3) << 1806.53,0,815.786,0,1595.14,590.314,0,0,1);
    Mat distCoeffs1 = (Mat_<double>(1,5) << -0.267514,0.213748,0.00136627,0.000194796,0);
    Mat cameraMatrix2 = (Mat_<double>(3,3) << 1739.3,0,808.929,0,1542.11,581.767,0,0,1);
    Mat distCoeffs2 = (Mat_<double>(1,5) << -0.247249,0.161344,-0.00280154,0.000444185,0);

    Mat rimage1, rimage2;

    undistort(image1, rimage1, cameraMatrix1, distCoeffs1);
    undistort(image2, rimage2, cameraMatrix2, distCoeffs2);

    resize(image1, image1, imageSizeSmall, 0, 0, CV_INTER_LINEAR);
    resize(image2, image2, imageSizeSmall, 0, 0, CV_INTER_LINEAR);
    imshow("image1", image1);
    imshow("image2", image2);

    resize(rimage1, rimage1, imageSizeSmall, 0, 0, CV_INTER_LINEAR);
    resize(rimage2, rimage2, imageSizeSmall, 0, 0, CV_INTER_LINEAR);
    imshow("rimage1", rimage1);
    imshow("rimage2", rimage2);
}

void Stereoscopy::checkUndistortFromCapture() {
    Image rawImage1;
    Error error = camera.RetrieveBuffer(&rawImage1);
    if (error != PGRERROR_OK) {
        qDebug() << "Capture error";
        return;
    }
    Image rgbImage1;
    rawImage1.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage1);
    // camera1: convert to OpenCV Mat
    unsigned int rowBytes = (double)rgbImage1.GetReceivedDataSize() / (double)rgbImage1.GetRows();
    Mat image1 = Mat(rgbImage1.GetRows(), rgbImage1.GetCols(), CV_8UC3, rgbImage1.GetData(), rowBytes);

    Image rawImage2;
    error = camera2.RetrieveBuffer(&rawImage2);
    if (error != PGRERROR_OK) {
        qDebug() << "Capture error";
        return;
    }
    Image rgbImage2;
    rawImage2.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage2);
    rowBytes = (double)rgbImage2.GetReceivedDataSize() / (double)rgbImage2.GetRows();
    Mat image2 = Mat(rgbImage2.GetRows(), rgbImage2.GetCols(), CV_8UC3, rgbImage2.GetData(),rowBytes);

    Mat imageSmall = Mat(image1.rows / 4, image1.cols / 4, CV_8UC3);
    Size imageSizeSmall = imageSmall.size(); //400x300

    Mat cameraMatrix1 = (Mat_<double>(3,3) << 1806.53,0,815.786,0,1595.14,590.314,0,0,1);
    Mat distCoeffs1 = (Mat_<double>(1,5) << -0.267514,0.213748,0.00136627,0.000194796,0);
    Mat cameraMatrix2 = (Mat_<double>(3,3) << 1739.3,0,808.929,0,1542.11,581.767,0,0,1);
    Mat distCoeffs2 = (Mat_<double>(1,5) << -0.247249,0.161344,-0.00280154,0.000444185,0);

    Mat rimage1, rimage2;

    undistort(image1, rimage1, cameraMatrix1, distCoeffs1);
    undistort(image2, rimage2, cameraMatrix2, distCoeffs2);

    resize(image1, image1, imageSizeSmall, 0, 0, CV_INTER_LINEAR);
    resize(image2, image2, imageSizeSmall, 0, 0, CV_INTER_LINEAR);
    imshow("image1", image1);
    imshow("image2", image2);

    resize(rimage1, rimage1, imageSizeSmall, 0, 0, CV_INTER_LINEAR);
    resize(rimage2, rimage2, imageSizeSmall, 0, 0, CV_INTER_LINEAR);
    imshow("rimage1", rimage1);
    imshow("rimage2", rimage2);
}

void Stereoscopy::triangulate(string fn1, string fn2) {
    Mat image1 = imread(fn1);
    if (!image1.data) {
        qDebug() <<  "Could not open or find the image1";
        return ;
    }
    Mat image2 = imread(fn2);
    if (!image2.data) {
        qDebug() <<  "Could not open or find the image2";
        return ;
    }

    Mat imageSmall = Mat(image1.rows / 4, image1.cols / 4, CV_8UC3);
    Size imageSizeSmall = imageSmall.size(); //400x300

    Mat cameraMatrix1 = (Mat_<double>(3,3) << 1806.53,0,815.786,0,1595.14,590.314,0,0,1);
    Mat distCoeffs1 = (Mat_<double>(1,5) << -0.267514,0.213748,0.00136627,0.000194796,0);
    Mat cameraMatrix2 = (Mat_<double>(3,3) << 1739.3,0,808.929,0,1542.11,581.767,0,0,1);
    Mat distCoeffs2 = (Mat_<double>(1,5) << -0.247249,0.161344,-0.00280154,0.000444185,0);

    //Start

    bool isShowImages = false;
    bool isShowUImages = false;
    bool isShowTImages = false;
    bool isShowDImages = false;
    bool isShowCImages = true;

    Mat rimage1, rimage2;

    if (isShowImages) {
        resize(image1, rimage1, imageSizeSmall, 0, 0, CV_INTER_LINEAR);
        resize(image2, rimage2, imageSizeSmall, 0, 0, CV_INTER_LINEAR);
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
    Mat R = (Mat_<double>(3,3) << 0.991532,-0.0276931,-0.126874,0.0602062,0.963683,0.260173,0.115061,-0.265608,0.95719);
    Mat T = (Mat_<double>(3,1) << 8.33789,-17.5109,83.1614);
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
