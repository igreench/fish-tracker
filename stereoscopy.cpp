#include "stereoscopy.h"

#include <opencv/cv.h>

#include <QDebug>

Stereoscopy::Stereoscopy() {
    isStereoCalibrated = false;
    isShowing = true;

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

void Stereoscopy::loopCapture() {
    qDebug() << "Start capture";

    // camera1: undistortion
    //Mat cameraMatrix1 = (Mat_<double>(3,3) << 1947.44,0,1011.07,0,1707.99,414.401,0,0,1);
//    Mat cameraMatrix1 = (Mat_<double>(3,3) << 1927.04,0,917.475,0,1706.88,579.804,0,0,1);
    //Mat distCoeffs1 = (Mat_<double>(1,5) << -0.160223,-0.123745,0.00997641,-0.00934415,0);
//    Mat distCoeffs1 = (Mat_<double>(1,5) << -0.215192,0.171866,0.0118907,0.00368162,0);

    Mat cameraMatrix1 = (Mat_<double>(3,3) << 6413.91,0,810.516,0,5364.43,717.19,0,0,1);
    Mat distCoeffs1 = (Mat_<double>(1,5) << -3.62446,83.4373,-0.0860216,0.0135343,-2475.21);

    // camera2: undistortion
    //Mat cameraMatrix2 = (Mat_<double>(3,3) << 1849.16,0,900.797,0,1642.24,507.873,0,0,1);
//    Mat cameraMatrix2 = (Mat_<double>(3,3) << 1766.16,0,817.485,0,1564.63,562.498,0,0,1);
    //Mat distCoeffs2 = (Mat_<double>(1,5) << -0.26513,0.15058,0.00745374,-0.00767843,0);
//    Mat distCoeffs2 = (Mat_<double>(1,5) << -0.19803,0.0611625,-0.00181152,-0.000889885,0);

    Mat cameraMatrix2 = (Mat_<double>(3,3) << 13175.3,0,787.325,0,12363.5,646.243,0,0,1);
    Mat distCoeffs2 = (Mat_<double>(1,5) << -14.8809,391.48,-0.103346,-0.0954315,3172.4);

    //rvec1:  "3,1;-0.121595;0.378872;-0.181438;"
    //tvec1:  "3,1;-2.64742;-5.55454;64.7538;"
    //rvec2:  "3,1;-0.335479;0.106753;-0.188999;"
    //tvec2:  "3,1;-6.88747;-4.15174;138.473;"
    Mat rvec1 = (Mat_<double>(3,1) << -0.121595,0.378872,-0.181438);
    Mat tvec1 = (Mat_<double>(3,1) << -2.64742,-5.55454,64.7538);
    Mat rvec2 = (Mat_<double>(3,1) << -0.335479,0.106753,-0.188999);
    Mat tvec2 = (Mat_<double>(3,1) << -6.88747,-4.15174,138.473);

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

        Mat E, F;
        Mat R1, R2, P1, P2; //Q

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

        if (key == 's') {
            //imwrite( "image1.jpg", image1_1 );
            //imwrite( "image2.jpg", image2_1 );

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
                                    imageSize1, R, T, E, F/*,
                                    TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-8),
                                    CV_CALIB_ZERO_TANGENT_DIST +
                                    CV_CALIB_FIX_INTRINSIC+
                                    CV_CALIB_FIX_K3*/
                                    /*TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
                                    CV_CALIB_SAME_FOCAL_LENGTH | CV_CALIB_ZERO_TANGENT_DIST*/);

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
                qDebug() << "Done initUndistortRectifyMap";

                //cv::destroyAllWindows();//

                isStereoCalibrated = true;
            }
        }

        //showing
        if (isStereoCalibrated) {
            qDebug() << "showing";

            Mat rimage1, rimage2;
            Mat disp, disp8;

            initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, imageSize1, CV_32FC1, rmap1, rmap2);
            initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, imageSize2, CV_32FC1, rmap1, rmap2);

            remap(image1_1, rimage1, rmap1, rmap2, CV_INTER_LINEAR);
            remap(image2_1, rimage2, rmap1, rmap2, CV_INTER_LINEAR);
            //resize(rimage1, rimage1, image1_2.size(), 0, 0, CV_INTER_LINEAR);//
            //resize(rimage2, rimage2, image2_2.size(), 0, 0, CV_INTER_LINEAR);//

            sgbm(rimage1, rimage2, disp);
            normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);

            if (isShowing) {
                resize(disp8, disp8, image1_2.size(), 0, 0, CV_INTER_LINEAR);//!
                imshow("disp", disp8);

                /*resize(rimage1, rimage1, image1_2.size(), 0, 0, CV_INTER_LINEAR);
                resize(rimage2, rimage2, image2_2.size(), 0, 0, CV_INTER_LINEAR);
                imshow("rimage1", rimage1);
                imshow("rimage2", rimage2);*/
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
