#ifndef TRIANGULATION_H
#define TRIANGULATION_H

#include <opencv2/core/core.hpp>

class Triangulation
{
public:
    Triangulation();
    ~Triangulation();

    void setObjects(std::vector < cv::Point3d > objects);
    std::vector < cv::Point3d > getObjects();

    int blurWidth;
    int blurHeight;
    int thresh;
    int threshMaxval;

    int mode;

private:
    std::vector < cv::Point3d > objects;
};

#endif // TRIANGULATION_H
