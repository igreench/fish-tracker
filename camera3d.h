#ifndef CAMERA3D_H
#define CAMERA3D_H

#include <QObject>

#include "pgrcamera.h"
#include "stereoimage.h"

using namespace stereo;

class Camera3D : public QObject
{
    Q_OBJECT

public:
    Camera3D();

    void startCapture();
    void stopCapture();

    bool isConnected();

    StereoImage *getStereoImage();

public slots:
    void update();

private:
    PGRCamera *camera1;
    PGRCamera *camera2;
    StereoImage *stereoImage;

    bool isConnect;
};

#endif // CAMERA3D_H
