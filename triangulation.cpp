#include "triangulation.h"

Triangulation::Triangulation()
{
    blurWidth = 20;
    blurHeight = 20;
    thresh = 50;
    threshMaxval = 80;

    mode = 2;
}

Triangulation::~Triangulation()
{

}

void Triangulation::setObjects(std::vector < cv::Point3d > objects) {
    this->objects = objects;
}

std::vector < cv::Point3d > Triangulation::getObjects() {
    return objects;
}
