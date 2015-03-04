#ifndef IODATA_H
#define IODATA_H

#include <opencv2/core/core.hpp>

#include "stereoparametres.h"

using namespace cv;
using namespace std;
using namespace stereo;

class IOData
{
public:
    //Is it utility class?
    IOData();

    static Mat getMatFromFile(string filename);
    void saveStereoParametres(QString filename, StereoParametres* stereoParametres);
    void loadStereoParametres(QString filename, StereoParametres* stereoParametres, int mode);
    void loadStereoParametres(QString filename, StereoParametres* stereoParametres);
    void loadInternalParametres(QString filename, StereoParametres* stereoParametres);
    void loadExternalParametres(QString filename, StereoParametres* stereoParametres);
};

#endif // IODATA_H
