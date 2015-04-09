#ifndef IMAGEDATA_H
#define IMAGEDATA_H

#include <opencv2/core/core.hpp>

class ImageData
{
public:
    ImageData(int rows, int cols, int type, unsigned char *data, size_t step);
    ~ImageData();

    int getRows();
    int getCols();
    int getType();
    unsigned char *getData();
    size_t getStep();

private:
    int rows;
    int cols;
    int type;
    unsigned char *data;
    size_t step;
};

#endif // IMAGEDATA_H
