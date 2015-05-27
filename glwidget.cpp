#include <QtGui>
#include <QtOpenGL>
#include "glwidget.h"
#include <math.h>

/* 1.3 */
#ifndef GL_MULTISAMPLE
#define GL_MULTISAMPLE  0x809D
#endif

const GLfloat PI180 = M_PI / 180;
const GLfloat METER = 1.0f; //0.1f 1.0f
const GLfloat OFFSET = 5.0f;


GLWidget::GLWidget(QWidget* parent/*= 0*/) : QGLWidget(parent) {
    xRot = -90;
    yRot = 0;
    zRot = 0;
    xTra = 0;
    yTra = -0.5f;
    zTra = 0;
    nSca = 1;

    /*cubes.push_back(Cube(cv::Point3d(0.1f, 0.1f, 0.1f), 0.1f));
    cubes.push_back(Cube(cv::Point3d(0.3f, 0.1f, 0.1f), 0.1f));
    cubes.push_back(Cube(cv::Point3d(0.3f, 0.3f, 0.1f), 0.1f));*/

    cubes.push_back(Cube(cv::Point3d(0.36f, -0.44f, 0.3f), 0.04f));
    cubes.push_back(Cube(cv::Point3d(0.26f, -0.46f, 0.33f), 0.04f));
    cubes.push_back(Cube(cv::Point3d(-0.43f, -0.4f, 0.31f), 0.04f));

    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(change()));
    //timer->start(30);
}

GLWidget::~GLWidget() {
}

void GLWidget::setCubes(std::vector < cv::Point3d > objects) {
    cubes.clear();
    for (int i = 0; i < objects.size(); i++) {
        //cubes.push_back(Cube(cv::Point3d((objects[i].x - OFFSET - 40) / 10, (objects[i].y - OFFSET - 30) / 10, (objects[i].z + 100) / 10), 0.1f));
        //cubes.push_back(Cube(cv::Point3d((objects[i].x * 10 + 238) * METER, (objects[i].y * 10 + 236) * METER, ( - objects[i].z * 10 + 388) * METER), 0.1f));
        //cubes.push_back(Cube(cv::Point3d(objects[i].x * METER, objects[i].y  * METER, objects[i].z * METER), 0.1f));
        cubes.push_back(Cube(cv::Point3d(objects[i].x * 5, objects[i].y  * 5, objects[i].z * 5), 0.04f));
        qDebug() << "x: " << objects[i].x << " y: " << objects[i].y << " z: " << objects[i].z;
    }
}

void GLWidget::initializeGL() {
    qglClearColor(Qt::black);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);
}

void GLWidget::paintGL() {
    //init
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glScalef(nSca, nSca, nSca);
    glTranslatef(xTra, yTra, zTra);
    glRotatef(xRot, 1.0f, 0.0f, 0.0f);
    glRotatef(yRot, 0.0f, 1.0f, 0.0f);
    glRotatef(zRot, 0.0f, 0.0f, 1.0f);
    //field
    glLineWidth(1);
    qglColor(Qt::gray);
    glBegin(GL_LINES);
    for (int i = 0; i <= 20; i++) {
        glVertex3f(-1.0f + (GLfloat)i * 0.1f,-1.0f, 0.0f);
        glVertex3f(-1.0f + (GLfloat)i * 0.1f, 1.0f, 0.0f);
        glVertex3f(-1.0f,-1.0f + (GLfloat)i * 0.1f, 0.0f);
        glVertex3f( 1.0f,-1.0f + (GLfloat)i * 0.1f, 0.0f);
    }
    glEnd();
    //field
    qglColor(Qt::gray);
    glBegin(GL_LINES);
    for (int i = 0; i <= 10; i++) {
        glVertex3f(1.0f, 1.0f, 0.0f + (GLfloat)i * 0.1f);
        glVertex3f(1.0f, -1.0f, 0.0f + (GLfloat)i * 0.1f);

        glVertex3f(1.0f, 1.0f, 0.0f + (GLfloat)i * 0.1f);
        glVertex3f(-1.0f, 1.0f, 0.0f + (GLfloat)i * 0.1f);
    }
    glEnd();
    glLineWidth(4);
    glBegin(GL_LINES);
    qglColor(Qt::red);
    glVertex3f(1.0f, 1.0f, 0.0f);
    glVertex3f(-1.0f, 1.0f, 0.0f);
    qglColor(Qt::green);
    glVertex3f(1.0f, 1.0f, 0.0f);
    glVertex3f(1.0f, 1.0f, 1.0f);
    qglColor(Qt::blue);
    glVertex3f(1.0f, 1.0f, 0.0f);
    glVertex3f(1.0f, -1.0f, 0.0f);
    glEnd();
    //objects
    qglColor(Qt::green);
    for (int i = 0; i < cubes.size(); i++) {
        drawCube(cubes[i]);
    }
}

void GLWidget::resizeGL(int nWidth, int nHeight) {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    GLfloat ratio = (GLfloat)nHeight / (GLfloat)nWidth;
    //glOrtho(-0.5, 0.5, -0.5, 0.5, -5.0, 5.0);//
    if (nWidth >= nHeight)
       glOrtho(-1.0/ratio, 1.0/ratio, -1.0, 1.0, -10.0, 10.0);
    else
       glOrtho(-1.0, 1.0, -1.0*ratio, 1.0*ratio, -10.0, 10.0);
    //*/
    // glFrustum(-1.0, 1.0, -1.0, 1.0, 1.0, 10.0);
    glViewport(0, 0, (GLint)nWidth, (GLint)nHeight);
}

void GLWidget::mousePressEvent(QMouseEvent* e) {
    ptrMousePosition = e->pos();
}

void GLWidget::mouseMoveEvent(QMouseEvent* e) {
    if (e->buttons() & Qt::MiddleButton) {
        xTra += (GLfloat)(e->pos().x() - ptrMousePosition.x()) / (GLfloat)400;
        yTra -= (GLfloat)(e->pos().y() - ptrMousePosition.y()) / (GLfloat)400;
    }
    if (e->buttons() & Qt::LeftButton) {
        GLfloat t = xRot + 180 /*/ nSca*/ * (GLfloat)(e->y() - ptrMousePosition.y()) / height();
        /*if (t < -90) {
            xRot = -90;
        }
        else {
            if (t > -30) {
               xRot = -30;
            }
            else {
                */xRot = t;/*
            }
        }
        //*/
        zRot += 180 /*/ nSca*/ * (GLfloat)(e->x() - ptrMousePosition.x()) / width();
    }
    ptrMousePosition = e->pos();
    updateGL();
}

void GLWidget::wheelEvent(QWheelEvent *e) {
    e->delta() > 0 ? nSca *= 1.1f : nSca /= 1.1f;
    updateGL();
}

void GLWidget::drawCube(Cube cube) {
    glBegin(GL_QUADS);//0.1f = 1
        glVertex3f(cube.pos.x - cube.side / 2, cube.pos.y - cube.side / 2, cube.pos.z - cube.side / 2);
        glVertex3f(cube.pos.x - cube.side / 2, cube.pos.y - cube.side / 2, cube.pos.z + cube.side / 2);
        glVertex3f(cube.pos.x + cube.side / 2, cube.pos.y - cube.side / 2, cube.pos.z + cube.side / 2);
        glVertex3f(cube.pos.x + cube.side / 2, cube.pos.y - cube.side / 2, cube.pos.z - cube.side / 2);
        //
        glVertex3f(cube.pos.x - cube.side / 2, cube.pos.y - cube.side / 2, cube.pos.z - cube.side / 2);
        glVertex3f(cube.pos.x - cube.side / 2, cube.pos.y - cube.side / 2, cube.pos.z + cube.side / 2);
        glVertex3f(cube.pos.x - cube.side / 2, cube.pos.y + cube.side / 2, cube.pos.z + cube.side / 2);
        glVertex3f(cube.pos.x - cube.side / 2, cube.pos.y + cube.side / 2, cube.pos.z - cube.side / 2);
        //
        glVertex3f(cube.pos.x + cube.side / 2, cube.pos.y - cube.side / 2, cube.pos.z - cube.side / 2);
        glVertex3f(cube.pos.x + cube.side / 2, cube.pos.y - cube.side / 2, cube.pos.z + cube.side / 2);
        glVertex3f(cube.pos.x + cube.side / 2, cube.pos.y + cube.side / 2, cube.pos.z + cube.side / 2);
        glVertex3f(cube.pos.x + cube.side / 2, cube.pos.y + cube.side / 2, cube.pos.z - cube.side / 2);
        //
        glVertex3f(cube.pos.x - cube.side / 2, cube.pos.y + cube.side / 2, cube.pos.z - cube.side / 2);
        glVertex3f(cube.pos.x - cube.side / 2, cube.pos.y + cube.side / 2, cube.pos.z + cube.side / 2);
        glVertex3f(cube.pos.x + cube.side / 2, cube.pos.y + cube.side / 2, cube.pos.z + cube.side / 2);
        glVertex3f(cube.pos.x + cube.side / 2, cube.pos.y + cube.side / 2, cube.pos.z - cube.side / 2);
        //
        glVertex3f(cube.pos.x - cube.side / 2, cube.pos.y - cube.side / 2, cube.pos.z - cube.side / 2);
        glVertex3f(cube.pos.x - cube.side / 2, cube.pos.y + cube.side / 2, cube.pos.z - cube.side / 2);
        glVertex3f(cube.pos.x + cube.side / 2, cube.pos.y + cube.side / 2, cube.pos.z - cube.side / 2);
        glVertex3f(cube.pos.x + cube.side / 2, cube.pos.y - cube.side / 2, cube.pos.z - cube.side / 2);
        //
        glVertex3f(cube.pos.x - cube.side / 2, cube.pos.y - cube.side / 2, cube.pos.z + cube.side / 2);
        glVertex3f(cube.pos.x - cube.side / 2, cube.pos.y + cube.side / 2, cube.pos.z + cube.side / 2);
        glVertex3f(cube.pos.x + cube.side / 2, cube.pos.y + cube.side / 2, cube.pos.z + cube.side / 2);
        glVertex3f(cube.pos.x + cube.side / 2, cube.pos.y - cube.side / 2, cube.pos.z + cube.side / 2);
    glEnd();
}

void GLWidget::change() {
    //TODO
    updateGL();
}
