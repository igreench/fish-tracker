#ifndef CAMERA3D_H
#define CAMERA3D_H

#include <QObject>

#include "FlyCapture2.h"

#include "pgrcamera.h"
#include "stereoimage.h"

using namespace stereo;
using namespace FlyCapture2;

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

    int SIZE_W;
    int SIZE_H;
};

#endif // CAMERA3D_H
