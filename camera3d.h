#ifndef CAMERA3D_H
#define CAMERA3D_H

#include <QObject>
#include <QTimer>

#include "pgrcamera.h"

class Camera3D : public QObject
{
    Q_OBJECT

public:
    Camera3D();

    void startCapture();
    void stopCapture();

public slots:
    void update();

private:
    PGRCamera *camera1;
    PGRCamera *camera2;

    QTimer *timer;
};

#endif // CAMERA3D_H
