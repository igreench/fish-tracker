#ifndef CAMERA3D_H
#define CAMERA3D_H

#include <QObject>

#include "FlyCapture2.h"
#include <opencv2/core/core.hpp>

#include "pgrcamera.h"
#include "stereoimage.h"

using namespace stereo;
using namespace FlyCapture2;
using namespace cv;

class Camera3D : public QObject
{
    Q_OBJECT

public:
    Camera3D();

    void startCapture();
    void stopCapture();

    bool isConnected();

    StereoImage *getStereoImage();

    PGRCamera *camera1;
    PGRCamera *camera2;

public slots:
    void update();

private:

    StereoImage *stereoImage;

    bool isConnect;
};

#endif // CAMERA3D_H
