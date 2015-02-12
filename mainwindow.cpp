#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
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

    ui->spinBox_6->setRange(0, 200);

    ui->spinBox_6->setRange(0, 200);

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

    connect(this->ui->spinBox, SIGNAL(valueChanged(int)), this, SLOT(setSgbmSADWindowSize(int)));

    stereoscopy = new Stereoscopy();
    stereoscopy->startCapture();
    stereoscopy->loopCapture();
    stereoscopy->endCapture();
    //stereoscopy->checkProjectPoints("image1_1.jpg", "image2_1.jpg");
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
