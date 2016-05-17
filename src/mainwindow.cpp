#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QImage>





using namespace cv;




QImage Mat2QImage(Mat& image)
{
    QImage img;

    if (image.channels()==3) {
        cvtColor(image, image, CV_BGR2RGB);
        img = QImage((const unsigned char *)(image.data), image.cols, image.rows,
                image.cols*image.channels(), QImage::Format_RGB888);
    } else if (image.channels()==1) {
        img = QImage((const unsigned char *)(image.data), image.cols, image.rows,
                image.cols*image.channels(), QImage::Format_ARGB32);
    } else {
        img = QImage((const unsigned char *)(image.data), image.cols, image.rows,
                image.cols*image.channels(), QImage::Format_RGB888);
    }

    return img;
}


void MainWindow::ImageShow(const sensor_msgs::CompressedImageConstPtr& msg)
{
//  cvi = cv_bridge::toCvCopy(msg);
//  ROS_INFO_STREAM(cvi->header.stamp);

//  QImage img = Mat2QImage(cvi->image);

//  ROS_INFO_STREAM(cvi->image.cols);
//  ROS_INFO_STREAM(cvi->image.rows);
//  ROS_INFO_STREAM("dddddddd");

  ui->widget->setImage(cv_bridge::toCvCopy(msg));


}



MainWindow::MainWindow(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}
