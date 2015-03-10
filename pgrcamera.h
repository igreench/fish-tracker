#ifndef PGRCAMERA_H
#define PGRCAMERA_H

#include "FlyCapture2.h"

#include <opencv2/core/core.hpp>

using namespace FlyCapture2;
using namespace cv;

class PGRCamera
{
public:
    PGRCamera(int id);

    void startCapture();
    Mat getFrame();
    void stopCapture();
    bool isConnected();

private:
    int id;
    Error error;
    Camera camera;
    bool isConnect;
};

#endif // PGRCAMERA_H
