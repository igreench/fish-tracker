#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "stereoscopy.h"

#include "camera3d.h"

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


private slots:
    void on_pushButton_clicked();

private:
    Ui::MainWindow *ui;

    Stereoscopy *stereoscopy;
    bool isStarted;

    Camera3D *camera3d;
};

#endif // MAINWINDOW_H
