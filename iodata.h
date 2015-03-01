#ifndef IODATA_H
#define IODATA_H

#include <opencv2/core/core.hpp>

using namespace cv;
using namespace std;

class IOData
{
public:
    IOData();

    static Mat getMatFromFile(string fn);
};

#endif // IODATA_H
