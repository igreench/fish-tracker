#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QStringListModel>
#include <QDir>
#include <QTimer>

#include "disparitymap.h"
#include "camera3d.h"
#include "stereoparametres.h"
#include "stereoprocessing.h"
#include "iodata.h"

using namespace stereo;
using namespace std;

namespace Ui {
class MainWindow;
}

static void myMessageOutput(QtMsgType type, const QMessageLogContext &context, const QString &msg);

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

    void setStereoViewMode();

    void start();

protected:
    void resizeEvent(QResizeEvent* event);

private slots:
    void on_pushButton_clicked();
    void setIsShowingStereoImage(bool value);
    void on_pushButton_8_clicked();
    void on_pushButton_9_clicked();
    void on_pushButton_5_clicked();
    void on_pushButton_4_clicked();
    void on_pushButton_3_clicked();
    void on_pushButton_10_clicked();
    void update();

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

    int countMode1;
    int countMode2;
    int countMode3;
    Size currentSizeStereoImage;

    void createMenu();
    void resizeStereoViews();

    void loadLocalStereoImage(string fn1, string fn2);
    StereoImage *currentStereoImage(int countMode);
    void calcStereoImages();
    void showStereoImage(StereoImage *stereoImage, int countView);
    void showStereoImages();

    Size sizeStereoImage(int sourceWidth, int sourseHeight, int destWidth, int destHeight);
    void updateDisparityMap();

    bool isShowingStereoImage1;
    bool isShowingStereoImage2;
    bool isShowingStereoImage3;
    bool isExistStereoImage;
    bool isExistStereoParametres;
    bool isCapture;
    bool isResized;

    QTimer *timer;

    QStringList commands;

};

#endif // MAINWINDOW_H
