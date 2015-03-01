#include "iodata.h"

#include <opencv2/highgui/highgui.hpp>

#include <QDebug>

IOData::IOData()
{
}

Mat IOData::getMatFromFile(string fn) {
    Mat image = imread(fn);
    if (!image.data) {
        qDebug() <<  "Could not open or find the image";
        //return Mat();
    }
    return image;
}
