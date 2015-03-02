#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "disparitymap.h"
#include "camera3d.h"
#include "stereoparametres.h"
#include "stereoprocessing.h"
#include "iodata.h"

using namespace stereo;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

public slots:
    void setSgbmSADWindowSize(int value);
    void setSgbmNumberOfDisparities(int value);
    void setSgbmPreFilterCap(int value);
    void setSgbmMinDisparity(int value);
    void setSgbmUniquenessRatio(int value);
    void setSgbmSpeckleWindowSize(int value);
    void setSgbmSpeckleRange(int value);
    void setSgbmDisp12MaxDiff(int value);
    void setSgbmP1(int value);
    void setSgbmP2(int value);

protected:
    void resizeEvent(QResizeEvent* event);
    void resizeDone();

private slots:
    void on_pushButton_clicked();

    void setIsShowingStereoImage1(bool value);
    void setIsShowingStereoImage2(bool value);
    void setIsShowingStereoImage3(bool value);

private:
    Ui::MainWindow *ui;

    bool isStarted;

    Camera3D *camera3d;
    StereoImage *stereoImage;
    StereoImage *stereoImage1;
    StereoImage *stereoImage2;
    StereoImage *stereoImage3;
    StereoParametres *stereoParametres;
    StereoProcessing *stereoProcessing;
    DisparityMap *disparityMap;
    IOData *ioData;

    void loadLocalStereoImage(string fn1, string fn2);
    void calcStereoImages();
    void showStereoImages();

    bool isShowingStereoImage1;
    bool isShowingStereoImage2;
    bool isShowingStereoImage3;
};

#endif // MAINWINDOW_H
