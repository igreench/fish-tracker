#ifndef PGRCAMERA_H
#define PGRCAMERA_H

#include "FlyCapture2.h"

#include "imagedata.h"

using namespace FlyCapture2;

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
    Image rawImage;
    Image rgbImage;
};

#endif // PGRCAMERA_H
