#include "disparitymap.h"

DisparityMap::DisparityMap()
{
    /*sgbm.SADWindowSize = 5;
    sgbm.numberOfDisparities = 192;
    sgbm.preFilterCap = 4;
    sgbm.minDisparity = -64;
    sgbm.uniquenessRatio = 1;
    sgbm.speckleWindowSize = 150;
    sgbm.speckleRange = 2;
    sgbm.disp12MaxDiff = 10;
    sgbm.fullDP = false;
    sgbm.P1 = 600;
    sgbm.P2 = 2400;*/

    sgbm.SADWindowSize = 5; //5
    sgbm.numberOfDisparities = 128; //192
    sgbm.preFilterCap = 1; //4
    sgbm.minDisparity = 0; //-64
    sgbm.uniquenessRatio = 1; //1
    sgbm.speckleWindowSize = 150; //150
    sgbm.speckleRange = 2; //2
    sgbm.disp12MaxDiff = 2; //10
    sgbm.fullDP = false; //false
    sgbm.P1 = 400; //600
    sgbm.P2 = 1600; //2400
}
