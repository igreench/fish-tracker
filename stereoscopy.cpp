#include "stereoscopy.h"

#include <opencv/cv.h>

#include <QDebug>

Stereoscopy::Stereoscopy() {
    isStereoCalibrated = false;
    isShowing = true;

    StereoSGBM sgbm;
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

        //showing
        Mat rimage1, rimage2;
        Mat disp, disp8;
        if (isStereoCalibrated) {

            qDebug() << "showing";

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

                resize(rimage1, rimage1, image1_2.size(), 0, 0, CV_INTER_LINEAR);
                resize(rimage2, rimage2, image2_2.size(), 0, 0, CV_INTER_LINEAR);
                imshow("rimage1", rimage1);
                imshow("rimage2", rimage2);
            }
        } else {
            if (isShowing) {
                resize(image1_1, image1_1, image1_2.size(), 0, 0, CV_INTER_LINEAR);
                resize(image2_1, image2_1, image2_2.size(), 0, 0, CV_INTER_LINEAR);
                imshow("image1", image1_1);
                imshow("image2", image2_1);
            }
        }

        if (key == 's') {
            imwrite( "image1.jpg", image1_1 );
            imwrite( "image2.jpg", image2_1 );
        }

        if (key == 'p') {
            if (isStereoCalibrated) {
                qDebug() << "Checking disparity map";
                vector<Point2f> corners1;
                if (addImage(image1_1, &corners1, scres)) {

                    qDebug() << "Pattern found";

                    /*initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, imageSize1, CV_16SC2, rmap[0], rmap[1]);
                    initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, imageSize2, CV_16SC2, rmap[0], rmap[1]);

                    qDebug() << "Done initUndistortRectifyMap";

                    Mat rimage1, rimage2;
                    remap(image1_1, rimage1, rmap[0], rmap[1], CV_INTER_LINEAR);
                    remap(image2_1, rimage2, rmap[0], rmap[1], CV_INTER_LINEAR);

                    qDebug() << "Done remap";*/

                    Mat r;
                    Rodrigues(R, r);

                    qDebug() << "Done Rodrigues";

                    //vector<vector<Point2f> > imagePoints1;
                    //vector<Point2f> projectedPoints1;

                    //projectPoints(objectPoints, r, T, cameraMatrix1, distCoeffs1, projectedPoints1);

                    /*for(unsigned int i = 0; i < projectedPoints1.size(); ++i) {
                        //cout << "Image point: " << imagePoints1[i] << " Projected to " << projectedPoints1[i] << endl;
                    }*/

                    //

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
                    sgbm.P2 = 2400;

                    qDebug() << "Done StereoSGBM";*/

                    /*Mat disp, disp8;
                    sgbm(rimage1, rimage2, disp);
                    normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);

                    qDebug() << "Done normalize";*/

                    //resize(disp8, disp8, image2_2.size(), 0, 0, CV_INTER_LINEAR);
                    //imshow("disp", disp8);//

                    //
                    Mat image3d;

                    reprojectImageTo3D(disp8, image3d, Q);

                    qDebug() << "Done reprojectImageTo3D";

                    resize(scres, scres, image1_2.size(), 0, 0, CV_INTER_LINEAR);
                    imshow("scres", scres);

                    //objectPoints.clear();
                    vector<vector<Point3d> > objectPointsP;
                    vector<Point3d> objpoints;
                    for(unsigned int i = 0; i < corners1.size(); ++i) {
                        //objectPoints.push_back( image3d(corners1[i].x, corners1[i].y) );
                        /*objpoints.push_back(Point3f(
                                                image3d.at<Point3f>(image3d(corners1[i].x, corners1[i].y)[0]),
                                                image3d.at<Point3f>(image3d(corners1[i].x, corners1[i].y)[1]),
                                                image3d.at<Point3f>(image3d(corners1[i].x, corners1[i].y)[2])));*/
                        if ((image3d.at<Vec3f>(corners1[i].x, corners1[i].y)[0] > -1000) &&
                            (image3d.at<Vec3f>(corners1[i].x, corners1[i].y)[0] < 1000) &&
                            (image3d.at<Vec3f>(corners1[i].x, corners1[i].y)[1] > -1000) &&
                            (image3d.at<Vec3f>(corners1[i].x, corners1[i].y)[1] < 1000) &&
                            (image3d.at<Vec3f>(corners1[i].x, corners1[i].y)[2] > -1000) &&
                            (image3d.at<Vec3f>(corners1[i].x, corners1[i].y)[2] < 1000)) {
                            objpoints.push_back(Point3f(
                                                    image3d.at<Vec3f>(corners1[i].x, corners1[i].y)[0],
                                                    image3d.at<Vec3f>(corners1[i].x, corners1[i].y)[1],
                                                    image3d.at<Vec3f>(corners1[i].x, corners1[i].y)[2]));
                            //qDebug() << "Corner " << i << " : x=" << corners1[i].x << " y=" << corners1[i].y;
                            //qDebug() << "objpoints " << i << " : x=" << objpoints[i].x << " y=" << objpoints[i].y << " z=" << objpoints[i].z;
                        } else {
                            //objpoints.push_back(Point3f(0.0f, 0.0f, 0.0f));
                            //objpoints.push_back(Point3d(0, 0, 0));
                            //objpoints.push_back(objpoints[i-1]);
                            //qDebug() << "out of range";
                            //qDebug() << "Corner " << i << " : x=" << corners1[i].x << " y=" << corners1[i].y;
                            //qDebug() << "objpoints " << i << " : x=" << objpoints[i].x << " y=" << objpoints[i].y << " z=" << objpoints[i].z;
                        }
                    }
                    objectPointsP.push_back(objpoints);

                    qDebug() << "Done adding object points";


                    //imagePoints1.clear();
                    //vector<vector<Point2f> > imagePointsP;
                    vector<Point2d> imagePoints;
                    qDebug() << "Start projectPoints";

                    qDebug() << "r: " << matToString(r);
                    qDebug() << "T: " << matToString(T);
                    qDebug() << "cameraMatrix1: " << matToString(cameraMatrix1);
                    qDebug() << "distCoeffs1: " << matToString(distCoeffs1);
                    for(unsigned int i = 0; i < objpoints.size(); ++i) {
                        qDebug() << objpoints[i].x << objpoints[i].y << objpoints[i].z;
                    }
                    qDebug() << "Start projectPoints";
                    projectPoints(objpoints, r, T, cameraMatrix1, distCoeffs1, imagePoints);
                    //projectPoints(objpoints, r, T, cameraMatrix1, distCoeffs1, imagePointsP);

                    qDebug() << "Done projectPoints";

                    Size pattern_size = Size(BOARD_WIDTH, BOARD_HEIGHT);

                    //drawChessboardCorners(scres, pattern_size, imagePoints1, true);
                    //qDebug() << "Done drawChessboardCorners";

                    //imagePoints.push_back(Point2d(50, 50));

                    for(unsigned int i = 0; i < imagePoints.size(); ++i) {
                        circle(rimage1, Point2d(imagePoints[i].x, imagePoints[i].y), 20, Scalar( 255, 0, 0 ), CV_FILLED, 8, 0);
                        qDebug() << "x:" << imagePoints[i].x << "y:" << imagePoints[i].y;
                    }

                    //resize(scres, scres, image1_2.size(), 0, 0, CV_INTER_LINEAR);
                    //imshow("scres1", scres);

                    resize(rimage1, rimage1, image1_2.size(), 0, 0, CV_INTER_LINEAR);
                    imshow("scres1", rimage1);
                }
            }
        }

        if (key == 'o') {
            if (isStereoCalibrated) {
                qDebug() << "Checking disparity map";
                vector<Point2f> corners1;
                if (addImage(image1_1, &corners1, scres)) {
                    qDebug() << "Pattern found";

                    Mat r;
                    Rodrigues(R, r);

                    qDebug() << "Done Rodrigues";

                    int n = BOARD_WIDTH * BOARD_HEIGHT;
                    vector<Point3d> objpoints; //d?
                    for (int j = 0; j < n; j++) {
                        objpoints.push_back(Point3d(j % BOARD_WIDTH, j / BOARD_WIDTH, 0.0f));
                    }
                    //objectPoints.push_back(objpoints);//

                    qDebug() << "Done creation objectPoints";

                    vector<Point2d> imagePoints;
                    qDebug() << "Start projectPoints";

                    qDebug() << "r: " << matToString(r);
                    qDebug() << "T: " << matToString(T);
                    qDebug() << "cameraMatrix1: " << matToString(cameraMatrix1);
                    qDebug() << "distCoeffs1: " << matToString(distCoeffs1);
                    /*for(unsigned int i = 0; i < objpoints.size(); ++i) {
                        qDebug() << objpoints[i].x << objpoints[i].y << objpoints[i].z;
                    }*/
                    qDebug() << "Start projectPoints";
                    //projectPoints(objpoints, r, T, cameraMatrix1, distCoeffs1, imagePoints);
                    projectPoints(Mat(objpoints), rvec1, tvec1, cameraMatrix1, distCoeffs1, imagePoints);
                    //projectPoints(objpoints, r, T, cameraMatrix1, distCoeffs1, imagePointsP);

                    qDebug() << "Done projectPoints";

                    Size pattern_size = Size(BOARD_WIDTH, BOARD_HEIGHT);

                    //drawChessboardCorners(scres, pattern_size, imagePoints1, true);
                    //qDebug() << "Done drawChessboardCorners";

                    //imagePoints.push_back(Point2d(50, 50));

                    for(unsigned int i = 0; i < imagePoints.size(); ++i) {
                        circle(rimage1, Point2d(imagePoints[i].x, imagePoints[i].y), 20, Scalar( 255, 0, 0 ), CV_FILLED, 8, 0);
                        qDebug() << "x:" << imagePoints[i].x << "y:" << imagePoints[i].y;
                    }

                    //resize(scres, scres, image1_2.size(), 0, 0, CV_INTER_LINEAR);
                    //imshow("scres1", scres);

                    resize(rimage1, rimage1, image1_2.size(), 0, 0, CV_INTER_LINEAR);
                    imshow("scres1", rimage1);
                }
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
    Mat E, F;

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

        qDebug() << "imagePoints1 size:" << imagePoints1.size();
        qDebug() << "imagePoints2 size:" << imagePoints2.size();
        qDebug() << "corners1 size:" << corners1.size();
        qDebug() << "corners2 size:" << corners2.size();
        qDebug() << "objectPoints size:" << objectPoints.size();
        qDebug() << "obj size:" << obj.size();

        //calibrate

        Mat cameraMatrix = Mat(3, 3, CV_32FC1);
        Mat distCoeffs;
        vector<Mat> rvecs, tvecs;

        cameraMatrix.at<float>(0, 0) = 1;
        cameraMatrix.at<float>(1, 1) = 1;

        calibrateCamera(objectPoints, imagePoints1, imageSize1, cameraMatrix, distCoeffs, rvecs, tvecs);

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

        qDebug() << "R" << matToString(R);
        qDebug() << "T" << matToString(T);

        //resize(scres, scres, image1_2.size(), 0, 0, CV_INTER_LINEAR);
        imshow("scres", scres);

        //project points
        Mat r;
        Rodrigues(R, r);
        Mat t = (Mat_<double>(1,3) << -1.19373, -0.452709, -1.7799);
        //Mat r = (Mat_<double>(1,5) << -0.19803,0.0611625,-0.00181152,-0.000889885,0);

        qDebug() << "Done Rodrigues";

        vector<Point3f> objpoints; //d?
        for (int j = 0; j < n; j++) {
            objpoints.push_back(Point3f(j % BOARD_WIDTH, j / BOARD_WIDTH, 0.0f));
        }

        qDebug() << "Done creation objectPoints";

        vector<Point2f> imagePoints;
        qDebug() << "Start projectPoints";

        qDebug() << "r: " << matToString(r);
        qDebug() << "T: " << matToString(T);
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

void Stereoscopy::check() {
    // Read points
      std::vector<cv::Point2d> imagePoints = Generate2DPoints();
      std::vector<cv::Point3d> objectPoints = Generate3DPoints();

      // Create the known projection matrix
      cv::Mat P(3,4,cv::DataType<double>::type);
      P.at<double>(0,0) = -2.8058e-01;
      P.at<double>(1,0) = -6.8326e-02;
      P.at<double>(2,0) = 5.1458e-07;

      P.at<double>(0,1) = 2.0045e-02;
      P.at<double>(1,1) = -3.1718e-01;
      P.at<double>(2,1) = 4.5840e-06;

      P.at<double>(0,2) = 1.8102e-01;
      P.at<double>(1,2) = -7.2974e-02;
      P.at<double>(2,2) = 2.6699e-06;

      P.at<double>(0,3) = 6.6062e-01;
      P.at<double>(1,3) = 5.8402e-01;
      P.at<double>(2,3) = 1.5590e-03;

        // Decompose the projection matrix into:
      cv::Mat K(3,3,cv::DataType<double>::type); // intrinsic parameter matrix
      cv::Mat rvec(3,3,cv::DataType<double>::type); // rotation matrix

      cv::Mat Thomogeneous(4,1,cv::DataType<double>::type); // translation vector

      cv::decomposeProjectionMatrix(P, K, rvec, Thomogeneous);

      cv::Mat T(3,1,cv::DataType<double>::type); // translation vector
      //cv::Mat T;
      //cv::convertPointsHomogeneous(Thomogeneous, T);
      convertPointsFromHomogeneous(Thomogeneous, T);

      std::cout << "K: " << K << std::endl;
      std::cout << "rvec: " << rvec << std::endl;
      std::cout << "T: " << T << std::endl;

      // Create zero distortion
      cv::Mat distCoeffs(4,1,cv::DataType<double>::type);
      distCoeffs.at<double>(0) = 0;
      distCoeffs.at<double>(1) = 0;
      distCoeffs.at<double>(2) = 0;
      distCoeffs.at<double>(3) = 0;

      std::vector<cv::Point2f> projectedPoints;

      cv::Mat rvecR(3,1,cv::DataType<double>::type);//rodrigues rotation matrix
      cv::Rodrigues(rvec,rvecR);

      cv::projectPoints(objectPoints, rvecR, T, K, distCoeffs, projectedPoints);

      for(unsigned int i = 0; i < projectedPoints.size(); ++i)
        {
        std::cout << "Image point: " << imagePoints[i] << " Projected to " << projectedPoints[i] << std::endl;
        }
}

std::vector<cv::Point2d> Stereoscopy::Generate2DPoints()
{
  std::vector<cv::Point2d> points;

  double x,y;

  x=282;y=274;
  points.push_back(cv::Point2d(x,y));

  x=397;y=227;
  points.push_back(cv::Point2d(x,y));

  x=577;y=271;
  points.push_back(cv::Point2d(x,y));

  x=462;y=318;
  points.push_back(cv::Point2d(x,y));

  x=270;y=479;
  points.push_back(cv::Point2d(x,y));

  x=450;y=523;
  points.push_back(cv::Point2d(x,y));

  x=566;y=475;
  points.push_back(cv::Point2d(x,y));
  /*
  for(unsigned int i = 0; i < points.size(); ++i)
    {
    std::cout << points[i] << std::endl;
    }
  */
  return points;
}


std::vector<cv::Point3d> Stereoscopy::Generate3DPoints()
{
  std::vector<cv::Point3d> points;

  double x,y,z;

  x=.5;y=.5;z=-.5;
  points.push_back(cv::Point3d(x,y,z));

  x=.5;y=.5;z=.5;
  points.push_back(cv::Point3d(x,y,z));

  x=-.5;y=.5;z=.5;
  points.push_back(cv::Point3d(x,y,z));

  x=-.5;y=.5;z=-.5;
  points.push_back(cv::Point3d(x,y,z));

  x=.5;y=-.5;z=-.5;
  points.push_back(cv::Point3d(x,y,z));

  x=-.5;y=-.5;z=-.5;
  points.push_back(cv::Point3d(x,y,z));

  x=-.5;y=-.5;z=.5;
  points.push_back(cv::Point3d(x,y,z));

  /*
  for(unsigned int i = 0; i < points.size(); ++i)
    {
    std::cout << points[i] << std::endl;
    }
  */
  return points;
}
