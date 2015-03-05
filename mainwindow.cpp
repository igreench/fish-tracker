#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "asmopencv.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);    

    countMode1 = 0;
    countMode2 = 1;
    countMode3 = 2;
    isShowingStereoImage1 = true;
    isShowingStereoImage2 = true;
    isShowingStereoImage3 = false;
    isStarted = false;

    commands.append("Original");
    commands.append("Undistort");
    commands.append("ProjectPoints");
    commands.append("UndistortRectify");
    commands.append("DisparityMap");
    commands.append("Triangulate");
    commands.append("CirclesPattern");

    camera3d = new Camera3D();
    stereoImage = new StereoImage();
    stereoImage1 = new StereoImage();
    stereoImage2 = new StereoImage();
    stereoImage3 = new StereoImage();
    stereoParametres = new StereoParametres();
    stereoProcessing = new StereoProcessing();
    disparityMap = new DisparityMap();

    ioData = new IOData();
    ioData->loadStereoParametres("data.txt", stereoParametres);
    stereoParametres->print();

    createMenu();

    //qInstallMessageHandler(myMessageOutput);
    QStringListModel *model;
    model = new QStringListModel(this);
    model->setStringList(commands);
    ui->listView_3->setModel(model);

    ui->groupBox->setTitle("View1 - " + commands[countMode1]);
    ui->groupBox_2->setTitle("View2 - " + commands[countMode2]);
    ui->groupBox_3->setTitle("View3 - " + commands[countMode3]);
    ui->groupBox_3->setVisible(false);

    ui->label->setText("SADWindowSize");
    ui->label_2->setText("numberOfDisparities");
    ui->label_3->setText("preFilterCap");
    ui->label_4->setText("minDisparity");
    ui->label_5->setText("uniquenessRatio");
    ui->label_6->setText("speckleWindowSize");
    ui->label_7->setText("speckleRange");
    ui->label_8->setText("disp12MaxDiff");
    ui->label_9->setText("P1");
    ui->label_10->setText("P2");

    ui->spinBox->setRange(0, 100);
    ui->spinBox_2->setRange(0, 500);
    ui->spinBox_3->setRange(0, 200);
    ui->spinBox_4->setRange(-100, 100);
    ui->spinBox_5->setRange(0, 200);
    ui->spinBox_6->setRange(0, 300);
    ui->spinBox_7->setRange(0, 200);
    ui->spinBox_8->setRange(0, 100);
    ui->spinBox_9->setRange(0, 4000);
    ui->spinBox_10->setRange(0, 4000);

    ui->spinBox->setValue(disparityMap->sgbm.SADWindowSize);
    ui->spinBox_2->setValue(disparityMap->sgbm.numberOfDisparities);
    ui->spinBox_3->setValue(disparityMap->sgbm.preFilterCap);
    ui->spinBox_4->setValue(disparityMap->sgbm.minDisparity);
    ui->spinBox_5->setValue(disparityMap->sgbm.uniquenessRatio);
    ui->spinBox_6->setValue(disparityMap->sgbm.speckleWindowSize);
    ui->spinBox_7->setValue(disparityMap->sgbm.speckleRange);
    ui->spinBox_8->setValue(disparityMap->sgbm.disp12MaxDiff);
    ui->spinBox_9->setValue(disparityMap->sgbm.P1);
    ui->spinBox_10->setValue(disparityMap->sgbm.P2);

    /*
    connect(this->ui->spinBox, SIGNAL(valueChanged(int)), this, SLOT(setSgbmSADWindowSize(int)));
    connect(this->ui->spinBox_2, SIGNAL(valueChanged(int)), this, SLOT(setSgbmNumberOfDisparities(int)));
    connect(this->ui->spinBox_3, SIGNAL(valueChanged(int)), this, SLOT(setSgbmPreFilterCap(int)));
    connect(this->ui->spinBox_4, SIGNAL(valueChanged(int)), this, SLOT(setSgbmMinDisparity(int)));
    connect(this->ui->spinBox_5, SIGNAL(valueChanged(int)), this, SLOT(setSgbmUniquenessRatio(int)));
    connect(this->ui->spinBox_6, SIGNAL(valueChanged(int)), this, SLOT(setSgbmSpeckleWindowSize(int)));
    connect(this->ui->spinBox_7, SIGNAL(valueChanged(int)), this, SLOT(setSgbmSpeckleRange(int)));
    connect(this->ui->spinBox_8, SIGNAL(valueChanged(int)), this, SLOT(setSgbmDisp12MaxDiff(int)));
    connect(this->ui->spinBox_9, SIGNAL(valueChanged(int)), this, SLOT(setSgbmP1(int)));
    connect(this->ui->spinBox_10, SIGNAL(valueChanged(int)), this, SLOT(setSgbmP2(int)));
    */
}

void myMessageOutput(QtMsgType type, const QMessageLogContext &context, const QString &msg) {
    QByteArray localMsg = msg.toLocal8Bit();
    switch (type) {
    case QtDebugMsg:
        fprintf(stderr, "Debug: %s (%s:%u, %s)\n", localMsg.constData(), context.file, context.line, context.function);
        break;
    case QtWarningMsg:
        fprintf(stderr, "Warning: %s (%s:%u, %s)\n", localMsg.constData(), context.file, context.line, context.function);
        break;
    case QtCriticalMsg:
        fprintf(stderr, "Critical: %s (%s:%u, %s)\n", localMsg.constData(), context.file, context.line, context.function);
        break;
    case QtFatalMsg:
        fprintf(stderr, "Fatal: %s (%s:%u, %s)\n", localMsg.constData(), context.file, context.line, context.function);
        abort();
    }
}

void MainWindow::createMenu() {
    int n = 3;
    for (int i = 0; i < n; i++) {
        QMenu *menu = new QMenu("View" + QString::number(i + 1));
        QAction *action = new QAction("Enabled", this);
        action->setCheckable(true);
        if (i < 2) {
            action->setChecked(true);
        }
        action->setData(QString::number(i));
        connect(action, SIGNAL(triggered(bool)), this, SLOT(setIsShowingStereoImage(bool)));
        menu->addAction(action);
        menu->addSeparator();
        QActionGroup* group = new QActionGroup( this );
        vector< QAction* > actions;
        for (int j = 0; j < commands.size(); j++) {
            QString name = commands[j];
            QAction *action = new QAction(name, this);
            action->setCheckable(true);
            if (1 != i && 0 == j) {
                action->setChecked(true);
            }
            if (1 == i && 1 == j) {
                action->setChecked(true);
            }
            action->setActionGroup(group);
            action->setData(QString::number(i) + ";" + QString::number(j));
            connect(action, SIGNAL(triggered()), this, SLOT(setStereoViewMode()));
            actions.push_back(action);
        }
        //Why two fors?
        for (int i = 0; i < actions.size(); i++) {
            menu->addAction(actions[i]);
        }
        menuBar()->addMenu(menu);
    }
}

void MainWindow::resizeStereoViews() {
    int space = 10;
    int offsetx = 10;
    int offsety = 15;
    if (isShowingStereoImage1) {
        int w = (ui->groupBox->width() - space) / 2 - offsetx;
        int h = ui->groupBox->height() - offsety * 1.5;
        cv::Size size = sizeStereoImage(stereoImage1->getLeft().cols, stereoImage1->getLeft().rows, w, h);
        qDebug() << "w: " << size.width << " h: " << size.height;
        ui->label_11->setGeometry(offsetx, offsety, size.width, size.height);
        size = sizeStereoImage(stereoImage1->getRight().cols, stereoImage1->getRight().rows, w, h);
        ui->label_12->setGeometry(offsetx + size.width + space, offsety, size.width, size.height);
    }
    if (isShowingStereoImage2) {
        int w = (ui->groupBox_2->width() - space) / 2 - offsetx;
        int h = ui->groupBox_2->height() - offsety * 1.5;
        cv::Size size = sizeStereoImage(stereoImage2->getLeft().cols, stereoImage2->getLeft().rows, w, h);
        ui->label_13->setGeometry(offsetx, offsety, size.width, size.height);
        size = sizeStereoImage(stereoImage2->getRight().cols, stereoImage2->getRight().rows, w, h);
        ui->label_14->setGeometry(offsetx + size.width + space, offsety, size.width, size.height);
    }
    if (isShowingStereoImage3) {
        int w = (ui->groupBox_3->width() - space) / 2 - offsetx;
        int h = ui->groupBox_3->height() - offsety * 1.5;
        cv::Size size = sizeStereoImage(stereoImage3->getLeft().cols, stereoImage3->getLeft().rows, w, h);
        ui->label_15->setGeometry(offsetx, offsety, size.width, size.height);
        size = sizeStereoImage(stereoImage3->getRight().cols, stereoImage3->getRight().rows, w, h);
        ui->label_16->setGeometry(offsetx + size.width + space, offsety, size.width, size.height);
    }
}

void MainWindow::setSgbmSADWindowSize(int value) {
    disparityMap->sgbm.SADWindowSize = value;
}
void MainWindow::setSgbmNumberOfDisparities(int value) {
    disparityMap->sgbm.numberOfDisparities = value;
}
void MainWindow::setSgbmPreFilterCap(int value) {
    disparityMap->sgbm.preFilterCap = value;
}
void MainWindow::setSgbmMinDisparity(int value) {
    disparityMap->sgbm.minDisparity = value;
}
void MainWindow::setSgbmUniquenessRatio(int value) {
    disparityMap->sgbm.uniquenessRatio = value;
}
void MainWindow::setSgbmSpeckleWindowSize(int value) {
    disparityMap->sgbm.speckleWindowSize = value;
}
void MainWindow::setSgbmSpeckleRange(int value) {
    disparityMap->sgbm.speckleRange = value;
}
void MainWindow::setSgbmDisp12MaxDiff(int value) {
    disparityMap->sgbm.disp12MaxDiff = value;
}
void MainWindow::setSgbmP1(int value) {
    disparityMap->sgbm.P1 = value;
}
void MainWindow::setSgbmP2(int value) {
    disparityMap->sgbm.P2 = value;
}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::on_pushButton_clicked() {
    if (!isStarted) {
        //loadLocalStereoImage("image1_aqua1.jpg", "image2_aqua1.jpg");
        loadLocalStereoImage("image1.jpg", "image2.jpg");
        calcStereoImages();
        resizeStereoViews();
        showStereoImages();
        isStarted = true;
    } else {
        showStereoImages();
    }
}

//need move to IOData
void MainWindow::loadLocalStereoImage(string fn1, string fn2) {
    Mat image1 = IOData::getMatFromFile(fn1);
    Mat image2 = IOData::getMatFromFile(fn2);
    stereoImage->setImages(image1, image2);
}

StereoImage *MainWindow::currentStereoImage(int countMode) {
    Mat image;
    switch(countMode) {
        case 0:
        return stereoImage;
        case 1:
        return stereoProcessing->undistortStereoImage(stereoImage, stereoParametres);
        case 2:
        image = stereoProcessing->projectPoints(stereoImage, stereoParametres);
        return new StereoImage(image, image);
        case 3:
        return stereoProcessing->undistortRectify(stereoImage, stereoParametres);
        case 4:
        image = stereoProcessing->disparityMap(stereoImage, stereoParametres);
        return new StereoImage(image, image);
        case 5:
        return stereoProcessing->triangulate(stereoImage, stereoParametres);
        case 6:
        image = stereoProcessing->circlesPattern();
        return new StereoImage(image, image);
    }
    return stereoImage;
}

//Am I need in StereoView class?
void MainWindow::calcStereoImages() {
    stereoImage1 = currentStereoImage(countMode1);
    stereoImage2 = currentStereoImage(countMode2);
    stereoImage3 = currentStereoImage(countMode3);
}

void MainWindow::showStereoImage(StereoImage *stereoImage, int countView) {
    Mat image1 = stereoImage->getLeft();
    Mat image2 = stereoImage->getRight();
    if (1 == countView) {
        cv::Size size = cv::Size(ui->label_11->width(), ui->label_11->height());
        cv::resize(image1, image1, size, 0, 0, CV_INTER_LINEAR);
        ui->label_11->setPixmap(ASM::cvMatToQPixmap(image1));
        size = cv::Size(ui->label_12->width(), ui->label_12->height());
        cv::resize(image2, image2, size, 0, 0, CV_INTER_LINEAR);
        ui->label_12->setPixmap(ASM::cvMatToQPixmap(image2));
    }
    if (2 == countView) {
        cv::Size size = cv::Size(ui->label_13->width(), ui->label_13->height());
        cv::resize(image1, image1, size, 0, 0, CV_INTER_LINEAR);
        ui->label_13->setPixmap(ASM::cvMatToQPixmap(image1));
        size = cv::Size(ui->label_14->width(), ui->label_14->height());
        cv::resize(image2, image2, size, 0, 0, CV_INTER_LINEAR);
        ui->label_14->setPixmap(ASM::cvMatToQPixmap(image2));
    }
    if (3 == countView) {
        cv::Size size = cv::Size(ui->label_15->width(), ui->label_15->height());
        cv::resize(image1, image1, size, 0, 0, CV_INTER_LINEAR);
        ui->label_15->setPixmap(ASM::cvMatToQPixmap(image1));
        size = cv::Size(ui->label_16->width(), ui->label_16->height());
        cv::resize(image2, image2, size, 0, 0, CV_INTER_LINEAR);
        ui->label_16->setPixmap(ASM::cvMatToQPixmap(image2));
    }
}

void MainWindow::showStereoImages() {
    if (!stereoImage->isEmpty()) {
        if (isShowingStereoImage1) {
            showStereoImage(stereoImage1, 1);
        }
        if (isShowingStereoImage2) {
            showStereoImage(stereoImage2, 2);
        }
        if (isShowingStereoImage3) {
            showStereoImage(stereoImage3, 3);
        }
    }
}

void MainWindow::resizeEvent(QResizeEvent* event) {
   QMainWindow::resizeEvent(event);
   if (isStarted) {
       resizeStereoViews();
       showStereoImages();
   }
}

Size MainWindow::sizeStereoImage(int sourceWidth, int sourseHeight, int destWidth, int destHeight) {
   if (sourceWidth > 0 && sourseHeight > 0 && destWidth > 0 && destHeight > 0) {
       if ((double)destWidth / destHeight < (double)sourceWidth / sourseHeight) {
           return cv::Size(destWidth, destWidth * (double)sourseHeight / sourceWidth);
       }
       return cv::Size(destHeight * (double)sourceWidth / sourseHeight, destHeight);
   } else {
       qDebug() << "ERROR SIZE";
       return cv::Size(10, 10);
   }
}

void MainWindow::setIsShowingStereoImage(bool value) {
    QAction* action = qobject_cast<QAction*>(sender());
    if (action) {
        QString data = action->data().toString();
        //qDebug() << data;
        QStringList list = data.split(";");
        if (0 == list[0].toInt()) {
            isShowingStereoImage1 = value;
            ui->groupBox->setVisible(value);
        }
        if (1 == list[0].toInt()) {
            isShowingStereoImage2 = value;
            ui->groupBox_2->setVisible(value);
        }
        if (2 == list[0].toInt()) {
            isShowingStereoImage3 = value;
            ui->groupBox_3->setVisible(value);
        }        
        QCoreApplication::processEvents();
        if (isStarted) {
            resizeStereoViews();
            showStereoImages();
        }
    }
}

void MainWindow::updateDisparityMap() {
    disparityMap->sgbm.SADWindowSize = ui->spinBox->value();
    disparityMap->sgbm.numberOfDisparities = ui->spinBox_2->value();
    disparityMap->sgbm.preFilterCap = ui->spinBox_3->value();
    disparityMap->sgbm.minDisparity = ui->spinBox_4->value();
    disparityMap->sgbm.uniquenessRatio = ui->spinBox_5->value();
    disparityMap->sgbm.speckleWindowSize = ui->spinBox_6->value();
    disparityMap->sgbm.speckleRange = ui->spinBox_7->value();
    disparityMap->sgbm.disp12MaxDiff = ui->spinBox_8->value();
    disparityMap->sgbm.P1 = ui->spinBox_9->value();
    disparityMap->sgbm.P2 = ui->spinBox_10->value();
}

void MainWindow::setStereoViewMode() {
    QAction* action = qobject_cast<QAction*>(sender());
    if (action) {
        QString data = action->data().toString();
        //qDebug() << data;
        QStringList list = data.split(";");
        if (0 == list[0].toInt()) {
            countMode1 = list[1].toInt();
            ui->groupBox->setTitle("View1 - " + commands[countMode1]);
            if (isStarted) {
                stereoImage1 = currentStereoImage(countMode1);
                showStereoImage(stereoImage1, 1);
            }
        }
        if (1 == list[0].toInt()) {
            countMode2 = list[1].toInt();
            ui->groupBox_2->setTitle("View2 - " + commands[countMode2]);
            if (isStarted) {
                stereoImage2 = currentStereoImage(countMode2);
                showStereoImage(stereoImage2, 2);
            }
        }
        if (2 == list[0].toInt()) {
            countMode3 = list[1].toInt();
            ui->groupBox_3->setTitle("View3 - " + commands[countMode3]);
            if (isStarted) {
                stereoImage3 = currentStereoImage(countMode3);
                showStereoImage(stereoImage3, 3);
            }
        }
    }
}
