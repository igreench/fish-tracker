#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QGLWidget>

class GLWidget : public QGLWidget
{
    Q_OBJECT

public:
    GLWidget(QWidget* parent = 0);
    ~GLWidget();
    void mousePressEvent(QMouseEvent *);
    void mouseMoveEvent(QMouseEvent *);
    void wheelEvent(QWheelEvent *);

protected:
    void initializeGL();
    void resizeGL(int nWidth, int nHeight);
    void paintGL();

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
    //cubes
    struct Cube {
        GLfloat x1;
        GLfloat y1;
        GLfloat x2;
        GLfloat y2;
        GLfloat z;
        GLfloat h;
        float m;
    };
    Cube cube1;
    Cube cube2;
    GLfloat heightBlock;

private slots:
    void change();
};

#endif // GLWIDGET_H
