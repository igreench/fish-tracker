#include "iodata.h"

#include <opencv2/highgui/highgui.hpp>

#include <QApplication>
#include <QFile>
#include <QTextStream>
#include <QIODevice>
#include <QDebug>

//using namespace stereo;

IOData::IOData()
{
}

Mat IOData::getMatFromFile(string filename) {
    Mat image = imread(filename);
    if (!image.data) {
        qDebug() <<  "Could not open or find the image";
        //return Mat();
    }
    return image;
}

void IOData::saveMat(Mat image, string filename) {
    imwrite(filename, image);

}

void IOData::saveStereoParametres(QString filename, StereoParametres* stereoParametres) {
    QFile file(filename);
    if (file.open(QIODevice::ReadWrite)) {
        QTextStream out(&file);
        out << "cameraMatrix1 \"" << matToString(stereoParametres->getCameraMatrix1()) << "\"" << endl;
        out << "cameraMatrix2 \"" << matToString(stereoParametres->getCameraMatrix2()) << "\"" << endl;
        out << "distCoeffs1 \"" << matToString(stereoParametres->getDistCoeffs1()) << "\"" << endl;
        out << "distCoeffs2 \"" << matToString(stereoParametres->getDistCoeffs2()) << "\"" << endl;
        out << "R \"" << matToString(stereoParametres->getR()) << "\"" << endl;
        out << "T \"" << matToString(stereoParametres->getT()) << "\"" << endl;
    }
}

void IOData::loadStereoParametres(QString filename, StereoParametres* stereoParametres, int mode) {
    //QFile file(QApplication::applicationDirPath() + "/" + filename);
    QFile file(filename);
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qDebug() << "Error opening file: " << file.error();
        return;
    }
    QTextStream in(&file);
    while(!in.atEnd()) {
        QString line = in.readLine();
        line.replace("\"", "");
        QStringList fields = line.split(" ");
        if (mode == 0 || mode == 1) {
            if ("cameraMatrix1" == fields[0]) {
                stereoParametres->setCameraMatrix1(stringToMat(fields[1]));
            }
            if ("cameraMatrix2" == fields[0]) {
                stereoParametres->setCameraMatrix2(stringToMat(fields[1]));
            }
            if ("distCoeffs1" == fields[0]) {
                stereoParametres->setDistCoeffs1(stringToMat(fields[1]));
            }
            if ("distCoeffs2" == fields[0]) {
                stereoParametres->setDistCoeffs2(stringToMat(fields[1]));
            }
        }
        if (mode == 1 || mode == 2) {
            if ("R" == fields[0]) {
                stereoParametres->setR(stringToMat(fields[1]));
            }
            if ("T" == fields[0]) {
                stereoParametres->setT(stringToMat(fields[1]));
            }
        }
    }
    file.close();
}

void IOData::loadStereoParametres(QString filename, StereoParametres* stereoParametres) {
    loadStereoParametres(filename, stereoParametres, 1);
}

void IOData::loadInternalParametres(QString filename, StereoParametres* stereoParametres) {
    loadStereoParametres(filename, stereoParametres, 0);
}

void IOData::loadExternalParametres(QString filename, StereoParametres* stereoParametres) {
    loadStereoParametres(filename, stereoParametres, 2);
}
