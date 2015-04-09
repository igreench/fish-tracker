#include "imagedata.h"

ImageData::ImageData(int rows, int cols, int type, unsigned char *data, size_t step) {
    this->rows = rows;
    this->cols = cols;
    this->type = type;
    this->data = data;
    this->step = step;
}

ImageData::~ImageData() {
}

int ImageData::getRows() {
    return rows;
}

int ImageData::getCols() {
    return cols;
}

int ImageData::getType() {
    return type;
}

unsigned char *ImageData::getData() {
    return data;
}

size_t ImageData::getStep() {
    return step;
}

