#include "triangulation.h"

Triangulation::Triangulation()
{

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
