#ifndef PGRCAMERA_H
#define PGRCAMERA_H

#include "FlyCapture2.h"

#include <opencv2/core/core.hpp>

#include "imagedata.h"

using namespace FlyCapture2;
using namespace cv;

class PGRCamera
{
public:
    PGRCamera(int id);

    void startCapture();
    ImageData *getFrame();
    void stopCapture();
    bool isConnected();

private:
    int id;
    Error error;
    Camera camera;
    bool isConnect;
    Mat image;
    Image rawImage;
    Image rgbImage;
};

#endif // PGRCAMERA_H
