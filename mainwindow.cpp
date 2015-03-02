#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "asmopencv.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);

    ui->label->setText("SADWindowSize");
    ui->label_2->setText("numberOfDisparities");
    ui->label_3->setText("preFilterCap");
    ui->label_4->setText("minDisparity");
    ui->label_5->setText("uniquenessRatio");
    ui->label_6->setText("speckleWindowSize");
    ui->label_7->setText("speckleRange");
    ui->label_8->setText("disp12MaxDiff");
    ui->label_9->setText("P1");
    ui->label_10->setText("P2");

    ui->spinBox->setRange(0, 100);
    ui->spinBox_2->setRange(0, 500);
    ui->spinBox_3->setRange(0, 200);
    ui->spinBox_4->setRange(-100, 100);
    ui->spinBox_5->setRange(0, 200);
    ui->spinBox_6->setRange(0, 300);
    ui->spinBox_7->setRange(0, 200);
    ui->spinBox_8->setRange(0, 100);
    ui->spinBox_9->setRange(0, 4000);
    ui->spinBox_10->setRange(0, 4000);

    disparityMap = new DisparityMap();

    ui->spinBox->setValue(disparityMap->sgbm.SADWindowSize);
    ui->spinBox_2->setValue(disparityMap->sgbm.numberOfDisparities);
    ui->spinBox_3->setValue(disparityMap->sgbm.preFilterCap);
    ui->spinBox_4->setValue(disparityMap->sgbm.minDisparity);
    ui->spinBox_5->setValue(disparityMap->sgbm.uniquenessRatio);
    ui->spinBox_6->setValue(disparityMap->sgbm.speckleWindowSize);
    ui->spinBox_7->setValue(disparityMap->sgbm.speckleRange);
    ui->spinBox_8->setValue(disparityMap->sgbm.disp12MaxDiff);
    ui->spinBox_9->setValue(disparityMap->sgbm.P1);
    ui->spinBox_10->setValue(disparityMap->sgbm.P2);

    /*connect(this->ui->spinBox, SIGNAL(valueChanged(int)), this, SLOT(setSgbmSADWindowSize(int)));
    connect(this->ui->spinBox_2, SIGNAL(valueChanged(int)), this, SLOT(setSgbmNumberOfDisparities(int)));
    connect(this->ui->spinBox_3, SIGNAL(valueChanged(int)), this, SLOT(setSgbmPreFilterCap(int)));
    connect(this->ui->spinBox_4, SIGNAL(valueChanged(int)), this, SLOT(setSgbmMinDisparity(int)));
    connect(this->ui->spinBox_5, SIGNAL(valueChanged(int)), this, SLOT(setSgbmUniquenessRatio(int)));
    connect(this->ui->spinBox_6, SIGNAL(valueChanged(int)), this, SLOT(setSgbmSpeckleWindowSize(int)));
    connect(this->ui->spinBox_7, SIGNAL(valueChanged(int)), this, SLOT(setSgbmSpeckleRange(int)));
    connect(this->ui->spinBox_8, SIGNAL(valueChanged(int)), this, SLOT(setSgbmDisp12MaxDiff(int)));
    connect(this->ui->spinBox_9, SIGNAL(valueChanged(int)), this, SLOT(setSgbmP1(int)));
    connect(this->ui->spinBox_10, SIGNAL(valueChanged(int)), this, SLOT(setSgbmP2(int)));*/

    camera3d = new Camera3D();
    stereoImage = new StereoImage();
    stereoImage1 = new StereoImage();
    stereoImage2 = new StereoImage();
    stereoImage3 = new StereoImage();
    stereoParametres = new StereoParametres();
    ioData = new IOData();
    ioData->loadStereoParametres("data.txt", stereoParametres);
    stereoParametres->print();
    stereoProcessing = new StereoProcessing();
    //stereoProcessing->setStereoParametres(stereoParametres);

    isStarted = false;

    isShowingStereoImage1 = true;
    isShowingStereoImage2 = true;
    isShowingStereoImage3 = false;

    //ui->label_15->setVisible(false);
    //ui->label_16->setVisible(false);
    ui->groupBox_3->setVisible(false);

    connect(this->ui->actionVisible1, SIGNAL(triggered(bool)), this, SLOT(setIsShowingStereoImage1(bool)));
    connect(this->ui->actionVisible2, SIGNAL(triggered(bool)), this, SLOT(setIsShowingStereoImage2(bool)));
    connect(this->ui->actionVisible3, SIGNAL(triggered(bool)), this, SLOT(setIsShowingStereoImage3(bool)));
}

void MainWindow::setSgbmSADWindowSize(int value) {
    disparityMap->sgbm.SADWindowSize = value;
}
void MainWindow::setSgbmNumberOfDisparities(int value) {
    disparityMap->sgbm.numberOfDisparities = value;
}
void MainWindow::setSgbmPreFilterCap(int value) {
    disparityMap->sgbm.preFilterCap = value;
}
void MainWindow::setSgbmMinDisparity(int value) {
    disparityMap->sgbm.minDisparity = value;
}
void MainWindow::setSgbmUniquenessRatio(int value) {
    disparityMap->sgbm.uniquenessRatio = value;
}
void MainWindow::setSgbmSpeckleWindowSize(int value) {
    disparityMap->sgbm.speckleWindowSize = value;
}
void MainWindow::setSgbmSpeckleRange(int value) {
    disparityMap->sgbm.speckleRange = value;
}
void MainWindow::setSgbmDisp12MaxDiff(int value) {
    disparityMap->sgbm.disp12MaxDiff = value;
}
void MainWindow::setSgbmP1(int value) {
    disparityMap->sgbm.P1 = value;
}
void MainWindow::setSgbmP2(int value) {
    disparityMap->sgbm.P2 = value;
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_clicked()
{

    if (!isStarted) {
        loadLocalStereoImage("image1_aqua1.jpg", "image2_aqua1.jpg");
        //stereoProcessing->setStereoImage(stereoImage);
        calcStereoImages();
        showStereoImages();

        //stereoscopy->startCapture();
        //stereoscopy->checkUndistortFromCapture();
        //stereoscopy->checkDisparityMapFromCapture();
        //stereoscopy->checkDisparityMapFromCapture2();

        //camera3d->startCapture();

        /*stereoscopy->startCapture();
        stereoscopy->loopCapture();
        stereoscopy->endCapture();*/

        //stereoscopy->checkProjectPoints("image1_1.jpg", "image2_1.jpg");

        //stereoscopy->checkDisparityMap("image1_a.jpg", "image2_a.jpg");
        //stereoscopy->checkDisparityMap("image1_cal1.jpg", "image2_cal1.jpg");
        //stereoscopy->checkUndistort("image1_aqua3.jpg", "image2_aqua3.jpg");

        //stereoscopy->checkDisparityMap2("image1_a.jpg", "image2_a.jpg");
        //stereoscopy->checkDisparityMap2("image1_b.jpg", "image2_b.jpg");

        //stereoscopy->checkDisparityMap2("image1_aqua3.jpg", "image2_aqua3.jpg");
        //stereoscopy->checkDisparityMap2("image1.jpg", "image2.jpg");

        //stereoscopy->triangulate("image1_aqua1.jpg", "image2_aqua1.jpg");

        //stereoscopy->drawCirclesPattern();

        isStarted = true;
    } else {
        /*disparityMap->sgbm.SADWindowSize = ui->spinBox->value();
        disparityMap->sgbm.numberOfDisparities = ui->spinBox_2->value();
        disparityMap->sgbm.preFilterCap = ui->spinBox_3->value();
        disparityMap->sgbm.minDisparity = ui->spinBox_4->value();
        disparityMap->sgbm.uniquenessRatio = ui->spinBox_5->value();
        disparityMap->sgbm.speckleWindowSize = ui->spinBox_6->value();
        disparityMap->sgbm.speckleRange = ui->spinBox_7->value();
        disparityMap->sgbm.disp12MaxDiff = ui->spinBox_8->value();
        disparityMap->sgbm.P1 = ui->spinBox_9->value();
        disparityMap->sgbm.P2 = ui->spinBox_10->value();*/

        //stereoscopy->showDisparityMap();
        //stereoscopy->checkDisparityMapFromCapture2();
        //stereoscopy->checkUndistortFromCapture();

        //stereoscopy->checkDisparityMapFromCapture2();

        //camera3d->stopCapture();

        //BUG!!! sometimes not update labels
        showStereoImages();
    }
    /*stereoscopy->startCapture();
    stereoscopy->showImagesFromCameras();
    stereoscopy->endCapture();*/
}

//need move to IOData
void MainWindow::loadLocalStereoImage(string fn1, string fn2) {
    Mat image1 = IOData::getMatFromFile(fn1);
    Mat image2 = IOData::getMatFromFile(fn2);
    stereoImage->setImages(image1, image2);
}

void MainWindow::calcStereoImages() {
    stereoImage1 = stereoImage;
    stereoImage2 = stereoProcessing->undistortStereoImage(stereoImage, stereoParametres);
    stereoImage3 = stereoImage;
}

void MainWindow::showStereoImages() {
    if (!stereoImage->isEmpty()) {
        //Mat image1 = stereoImage->getLeft();
        //Mat image2 = stereoImage->getRight();
        Size size;
        //qDebug() << ui->label_11->width() << ui->label_11->height();
        if ((double)ui->label_11->width() / ui->label_11->height() < (double)stereoImage->getLeft().cols / stereoImage->getLeft().rows) {
            size = Size(ui->label_11->width(), ui->label_11->width() * (double)stereoImage->getLeft().rows / stereoImage->getLeft().cols);
        } else {
            size = Size(ui->label_11->height() * (double)stereoImage->getLeft().cols / stereoImage->getLeft().rows, ui->label_11->height());
        }



        //BUG!!! sometimes not update labels

        if (isShowingStereoImage1) {
            Mat image1 = stereoImage1->getLeft();
            Mat image2 = stereoImage1->getRight();

            cv::resize(image1, image1, size, 0, 0, CV_INTER_LINEAR);
            cv::resize(image2, image2, size, 0, 0, CV_INTER_LINEAR);

            ui->label_11->setPixmap(ASM::cvMatToQPixmap(image1));
            ui->label_12->setPixmap(ASM::cvMatToQPixmap(image2));
        }

        if (isShowingStereoImage2) {
            /*StereoImage *undistortStereoImage = new StereoImage();
            undistortStereoImage = stereoProcessing->undistortStereoImage();

            Mat uimage1 = undistortStereoImage->getLeft();
            Mat uimage2 = undistortStereoImage->getRight();

            cv::resize(uimage1, uimage1, size, 0, 0, CV_INTER_LINEAR);
            cv::resize(uimage2, uimage2, size, 0, 0, CV_INTER_LINEAR);*/

            Mat image1 = stereoImage2->getLeft();
            Mat image2 = stereoImage2->getRight();

            cv::resize(image1, image1, size, 0, 0, CV_INTER_LINEAR);
            cv::resize(image2, image2, size, 0, 0, CV_INTER_LINEAR);

            ui->label_13->setPixmap(ASM::cvMatToQPixmap(image1));
            ui->label_14->setPixmap(ASM::cvMatToQPixmap(image2));
        }
        if (isShowingStereoImage3) {

            Mat image1 = stereoImage3->getLeft();
            Mat image2 = stereoImage3->getRight();

            cv::resize(image1, image1, size, 0, 0, CV_INTER_LINEAR);
            cv::resize(image2, image2, size, 0, 0, CV_INTER_LINEAR);

            ui->label_15->setPixmap(ASM::cvMatToQPixmap(image1));
            ui->label_16->setPixmap(ASM::cvMatToQPixmap(image2));
        }
    }
}

void MainWindow::resizeEvent(QResizeEvent* event) {
   QMainWindow::resizeEvent(event);
   showStereoImages();
}

void MainWindow::resizeDone() {
   showStereoImages();
   qDebug() << "resizeDone";
}

void MainWindow::setIsShowingStereoImage1(bool value) {
    //qDebug() << value;
    isShowingStereoImage1 = value;
    ui->groupBox->setVisible(value);
    //ui->label_11->setVisible(value);
    //ui->label_12->setVisible(value);
    showStereoImages();
}

void MainWindow::setIsShowingStereoImage2(bool value) {
    //qDebug() << value;
    isShowingStereoImage2 = value;
    ui->groupBox_2->setVisible(value);
    //ui->label_13->setVisible(value);
    //ui->label_14->setVisible(value);
    showStereoImages();
}

void MainWindow::setIsShowingStereoImage3(bool value) {
    qDebug() << value;
    isShowingStereoImage3 = value;
    ui->groupBox_3->setVisible(value);
    //ui->label_15->setVisible(value);
    //ui->label_16->setVisible(value);
    showStereoImages();

}
