#include <QtGui>
#include <QtOpenGL>
#include "glwidget.h"
#include <math.h>

/* 1.3 */
#ifndef GL_MULTISAMPLE
#define GL_MULTISAMPLE  0x809D
#endif

const GLfloat PI180 = M_PI / 180;
const GLfloat METER = 0.1f;


GLWidget::GLWidget(QWidget* parent/*= 0*/) : QGLWidget(parent) {
    xRot = -90;
    yRot = 0;
    zRot = 0;
    xTra = 0;
    yTra = -0.5f;
    zTra = 0;
    nSca = 1;
    //
    cube1.x1 = -0.6f;
    cube1.y1 = 0;
    cube1.x2 = -0.3f;
    cube1.y2 = 0.3f;
    cube1.z = 0;
    cube1.h = 0.3f;
    cube1.m = 50;
    //
    cube2.x1 = 0.3f;
    cube2.y1 = 0;
    cube2.x2 = 0.6f;
    cube2.y2 = 0.3f;
    cube2.z = 0.5f;
    cube2.h = 0.3f;
    cube2.m = 10;
    //
    heightBlock = 1;
    //
    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(change()));
    //timer->start(30);
}

GLWidget::~GLWidget() {
}

void GLWidget::initializeGL() {
    qglClearColor(Qt::black);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);
}

void GLWidget::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glScalef(nSca, nSca, nSca);
    glTranslatef(xTra, yTra, zTra);
    glRotatef(xRot, 1.0f, 0.0f, 0.0f);
    glRotatef(yRot, 0.0f, 1.0f, 0.0f);
    glRotatef(zRot, 0.0f, 0.0f, 1.0f);
    //
    qglColor(Qt::gray);
    glBegin(GL_LINES);
    for (int i = 0; i <= 20; i++) {
        glVertex3f(-1.0f + (GLfloat)i * 0.1f,-1.0f, 0.0f);
        glVertex3f(-1.0f + (GLfloat)i * 0.1f, 1.0f, 0.0f);
        glVertex3f(-1.0f,-1.0f + (GLfloat)i * 0.1f, 0.0f);
        glVertex3f( 1.0f,-1.0f + (GLfloat)i * 0.1f, 0.0f);
    }
    glEnd();
    qglColor(Qt::blue);
    glBegin(GL_LINES);
        glVertex3f((cube1.x1 + cube1.x2) / 2, (cube1.y1 + cube1.y2) / 2, cube1.z + cube1.h);
        glVertex3f((cube1.x1 + cube1.x2) / 2, (cube1.y1 + cube1.y2) / 2, heightBlock);
        glVertex3f((cube2.x1 + cube2.x2) / 2, (cube2.y1 + cube2.y2) / 2, cube2.z + cube2.h);
        glVertex3f((cube2.x1 + cube2.x2) / 2, (cube2.y1 + cube2.y2) / 2, heightBlock);
    glEnd();
    glBegin(GL_TRIANGLES);
        glVertex3f((cube1.x1 + cube1.x2) / 2, (cube1.y1 + cube1.y2) / 2, heightBlock);
        glVertex3f((cube1.x1 + cube1.x2 + cube2.x1 + cube2.x2) / 4, (cube1.y1 + cube1.y2 + cube2.y1 + cube2.y2) / 4, heightBlock + 0.2f);
        glVertex3f((cube2.x1 + cube2.x2) / 2, (cube2.y1 + cube2.y2) / 2, heightBlock);
    glEnd();
    qglColor(Qt::green);
    glBegin(GL_QUADS);//0.1f = 1
        glVertex3f(cube1.x1, cube1.y1, cube1.z);
        glVertex3f(cube1.x1, cube1.y1, cube1.z + cube1.h);
        glVertex3f(cube1.x2, cube1.y1, cube1.z + cube1.h);
        glVertex3f(cube1.x2, cube1.y1, cube1.z);
        //
        glVertex3f(cube1.x1, cube1.y1, cube1.z);
        glVertex3f(cube1.x1, cube1.y1, cube1.z + cube1.h);
        glVertex3f(cube1.x1, cube1.y2, cube1.z + cube1.h);
        glVertex3f(cube1.x1, cube1.y2, cube1.z);
        //
        glVertex3f(cube1.x2, cube1.y1, cube1.z);
        glVertex3f(cube1.x2, cube1.y1, cube1.z + cube1.h);
        glVertex3f(cube1.x2, cube1.y2, cube1.z + cube1.h);
        glVertex3f(cube1.x2, cube1.y2, cube1.z);
        //
        glVertex3f(cube1.x1, cube1.y2, cube1.z);
        glVertex3f(cube1.x1, cube1.y2, cube1.z + cube1.h);
        glVertex3f(cube1.x2, cube1.y2, cube1.z + cube1.h);
        glVertex3f(cube1.x2, cube1.y2, cube1.z);
        //
        glVertex3f(cube1.x1, cube1.y1, cube1.z);
        glVertex3f(cube1.x1, cube1.y2, cube1.z);
        glVertex3f(cube1.x2, cube1.y2, cube1.z);
        glVertex3f(cube1.x2, cube1.y1, cube1.z);
        //
        glVertex3f(cube1.x1, cube1.y1, cube1.z + cube1.h);
        glVertex3f(cube1.x1, cube1.y2, cube1.z + cube1.h);
        glVertex3f(cube1.x2, cube1.y2, cube1.z + cube1.h);
        glVertex3f(cube1.x2, cube1.y1, cube1.z + cube1.h);
    glEnd();
    qglColor(Qt::red);
    glBegin(GL_QUADS);
        glVertex3f(cube2.x1, cube2.y1, cube2.z);
        glVertex3f(cube2.x1, cube2.y1, cube2.z + cube2.h);
        glVertex3f(cube2.x2, cube2.y1, cube2.z + cube2.h);
        glVertex3f(cube2.x2, cube2.y1, cube2.z);
        //
        glVertex3f(cube2.x1, cube2.y1, cube2.z);
        glVertex3f(cube2.x1, cube2.y1, cube2.z + cube2.h);
        glVertex3f(cube2.x1, cube2.y2, cube2.z + cube2.h);
        glVertex3f(cube2.x1, cube2.y2, cube2.z);
        //
        glVertex3f(cube2.x2, cube2.y1, cube2.z);
        glVertex3f(cube2.x2, cube2.y1, cube2.z + cube2.h);
        glVertex3f(cube2.x2, cube2.y2, cube2.z + cube2.h);
        glVertex3f(cube2.x2, cube2.y2, cube2.z);
        //
        glVertex3f(cube2.x1, cube2.y2, cube2.z);
        glVertex3f(cube2.x1, cube2.y2, cube2.z + cube2.h);
        glVertex3f(cube2.x2, cube2.y2, cube2.z + cube2.h);
        glVertex3f(cube2.x2, cube2.y2, cube2.z);
        //
        glVertex3f(cube2.x1, cube2.y1, cube2.z);
        glVertex3f(cube2.x1, cube2.y2, cube2.z);
        glVertex3f(cube2.x2, cube2.y2, cube2.z);
        glVertex3f(cube2.x2, cube2.y1, cube2.z);
        //
        glVertex3f(cube2.x1, cube2.y1, cube2.z + cube2.h);
        glVertex3f(cube2.x1, cube2.y2, cube2.z + cube2.h);
        glVertex3f(cube2.x2, cube2.y2, cube2.z + cube2.h);
        glVertex3f(cube2.x2, cube2.y1, cube2.z + cube2.h);
    glEnd();
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

void GLWidget::change() {
    if ((cube1.m != cube2.m) && (cube1.z + cube1.h < heightBlock) && (cube2.z + cube2.h < heightBlock))
    {
        cube1.z += (cube1.m - cube2.m) / abs(cube1.m - cube2.m) * 0.01f;
        cube2.z += (cube2.m - cube1.m) / abs(cube1.m - cube2.m) * 0.01f;
        if (cube1.z + cube1.h > heightBlock)
            cube1.z = heightBlock - cube1.h;
        if (cube2.z + cube2.h > heightBlock)
            cube2.z = heightBlock - cube2.h;
    }
    updateGL();
}
