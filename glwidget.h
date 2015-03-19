#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QGLWidget>

#include <opencv2/core/core.hpp>

struct Cube {
    Cube(cv::Point3d pos, GLfloat side) {
        this->pos = pos;
        this->side = side;
    }
    cv::Point3d pos;
    GLfloat side;
};

class GLWidget : public QGLWidget
{
    Q_OBJECT

public:
    GLWidget(QWidget* parent = 0);
    ~GLWidget();

    void setCubes(std::vector < cv::Point3d > objects);

protected:
    void initializeGL();
    void resizeGL(int nWidth, int nHeight);
    void paintGL();
    void mousePressEvent(QMouseEvent *);
    void mouseMoveEvent(QMouseEvent *);
    void wheelEvent(QWheelEvent *);

private:
    void setupViewport(int width, int height);
    //values
    QPoint ptrMousePosition;
    GLfloat xRot;
    GLfloat yRot;
    GLfloat zRot;
    GLfloat xTra;
    GLfloat yTra;
    GLfloat zTra;
    GLfloat nSca;
    void drawCube(Cube cube);
    std::vector < Cube > cubes;

private slots:
    void change();
};

#endif // GLWIDGET_H
