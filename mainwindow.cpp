#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "asmopencv.h"
#include "iodata.h"

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

    stereoscopy = new Stereoscopy();

    ui->spinBox->setValue(stereoscopy->sgbm.SADWindowSize);
    ui->spinBox_2->setValue(stereoscopy->sgbm.numberOfDisparities);
    ui->spinBox_3->setValue(stereoscopy->sgbm.preFilterCap);
    ui->spinBox_4->setValue(stereoscopy->sgbm.minDisparity);
    ui->spinBox_5->setValue(stereoscopy->sgbm.uniquenessRatio);
    ui->spinBox_6->setValue(stereoscopy->sgbm.speckleWindowSize);
    ui->spinBox_7->setValue(stereoscopy->sgbm.speckleRange);
    ui->spinBox_8->setValue(stereoscopy->sgbm.disp12MaxDiff);
    ui->spinBox_9->setValue(stereoscopy->sgbm.P1);
    ui->spinBox_10->setValue(stereoscopy->sgbm.P2);

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

    isStarted = false;

    isShowingStereoImage1 = true;
    isShowingStereoImage2 = true;
    isShowingStereoImage3 = false;

    ui->label_15->setVisible(false);
    ui->label_16->setVisible(false);

    connect(this->ui->actionStereoImage1, SIGNAL(triggered(bool)), this, SLOT(setIsShowingStereoImage1(bool)));
    connect(this->ui->actionStereoImage2, SIGNAL(triggered(bool)), this, SLOT(setIsShowingStereoImage2(bool)));
    connect(this->ui->actionStereoImage3, SIGNAL(triggered(bool)), this, SLOT(setIsShowingStereoImage3(bool)));
}

void MainWindow::setSgbmSADWindowSize(int value) {
    stereoscopy->sgbm.SADWindowSize = value;
}
void MainWindow::setSgbmNumberOfDisparities(int value) {
    stereoscopy->sgbm.numberOfDisparities = value;
}
void MainWindow::setSgbmPreFilterCap(int value) {
    stereoscopy->sgbm.preFilterCap = value;
}
void MainWindow::setSgbmMinDisparity(int value) {
    stereoscopy->sgbm.minDisparity = value;
}
void MainWindow::setSgbmUniquenessRatio(int value) {
    stereoscopy->sgbm.uniquenessRatio = value;
}
void MainWindow::setSgbmSpeckleWindowSize(int value) {
    stereoscopy->sgbm.speckleWindowSize = value;
}
void MainWindow::setSgbmSpeckleRange(int value) {
    stereoscopy->sgbm.speckleRange = value;
}
void MainWindow::setSgbmDisp12MaxDiff(int value) {
    stereoscopy->sgbm.disp12MaxDiff = value;
}
void MainWindow::setSgbmP1(int value) {
    stereoscopy->sgbm.P1 = value;
}
void MainWindow::setSgbmP2(int value) {
    stereoscopy->sgbm.P2 = value;
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_clicked()
{

    if (!isStarted) {
        loadLocalStereoImage("image1_aqua1.jpg", "image2_aqua1.jpg");
        showStereoImage();

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
        stereoscopy->sgbm.SADWindowSize = ui->spinBox->value();
        stereoscopy->sgbm.numberOfDisparities = ui->spinBox_2->value();
        stereoscopy->sgbm.preFilterCap = ui->spinBox_3->value();
        stereoscopy->sgbm.minDisparity = ui->spinBox_4->value();
        stereoscopy->sgbm.uniquenessRatio = ui->spinBox_5->value();
        stereoscopy->sgbm.speckleWindowSize = ui->spinBox_6->value();
        stereoscopy->sgbm.speckleRange = ui->spinBox_7->value();
        stereoscopy->sgbm.disp12MaxDiff = ui->spinBox_8->value();
        stereoscopy->sgbm.P1 = ui->spinBox_9->value();
        stereoscopy->sgbm.P2 = ui->spinBox_10->value();
        //stereoscopy->showDisparityMap();
        //stereoscopy->checkDisparityMapFromCapture2();
        //stereoscopy->checkUndistortFromCapture();

        //stereoscopy->checkDisparityMapFromCapture2();

        //camera3d->stopCapture();

        showStereoImage();
    }
    /*stereoscopy->startCapture();
    stereoscopy->showImagesFromCameras();
    stereoscopy->endCapture();*/
}

void MainWindow::loadLocalStereoImage(string fn1, string fn2) {
    Mat image1 = IOData::getMatFromFile(fn1);
    Mat image2 = IOData::getMatFromFile(fn2);
    stereoImage->setImages(image1, image2);
}

void MainWindow::showStereoImage() {
    if (!stereoImage->isEmpty()) {
        Mat image1 = stereoImage->getLeft();
        Mat image2 = stereoImage->getRight();
        Size size;
        qDebug() << ui->label_11->width() << ui->label_11->height();
        if ((double)ui->label_11->width() / ui->label_11->height() < (double)stereoImage->getLeft().cols / stereoImage->getLeft().rows) {
            size = Size(ui->label_11->width(), ui->label_11->width() * (double)stereoImage->getLeft().rows / stereoImage->getLeft().cols);
        } else {
            size = Size(ui->label_11->height() * (double)stereoImage->getLeft().cols / stereoImage->getLeft().rows, ui->label_11->height());
        }
        cv::resize(image1, image1, size, 0, 0, CV_INTER_LINEAR);
        cv::resize(image2, image2, size, 0, 0, CV_INTER_LINEAR);

        if (isShowingStereoImage1) {
            ui->label_11->setPixmap(ASM::cvMatToQPixmap(image1));
            ui->label_12->setPixmap(ASM::cvMatToQPixmap(image2));
        }

        if (isShowingStereoImage2) {
            ui->label_13->setPixmap(ASM::cvMatToQPixmap(image1));
            ui->label_14->setPixmap(ASM::cvMatToQPixmap(image2));
        }
        if (isShowingStereoImage3) {
            ui->label_15->setPixmap(ASM::cvMatToQPixmap(image1));
            ui->label_16->setPixmap(ASM::cvMatToQPixmap(image2));
        }
    }
}

void MainWindow::resizeEvent(QResizeEvent* event) {
   QMainWindow::resizeEvent(event);
   showStereoImage();
}

void MainWindow::resizeDone() {
   showStereoImage();
   qDebug() << "resizeDone";
}

void MainWindow::setIsShowingStereoImage1(bool value) {
    //qDebug() << value;
    isShowingStereoImage1 = value;
    ui->label_11->setVisible(value);
    ui->label_12->setVisible(value);
    showStereoImage();
}

void MainWindow::setIsShowingStereoImage2(bool value) {
    //qDebug() << value;
    isShowingStereoImage2 = value;
    ui->label_13->setVisible(value);
    ui->label_14->setVisible(value);
    showStereoImage();
}

void MainWindow::setIsShowingStereoImage3(bool value) {
    qDebug() << value;
    isShowingStereoImage3 = value;
    ui->label_15->setVisible(value);
    ui->label_16->setVisible(value);
    showStereoImage();

}
