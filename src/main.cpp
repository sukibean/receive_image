/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QApplication>
#include <QThread>
#include "test.h"
#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "ros/ros.h"

/*****************************************************************************
** Main
*****************************************************************************/

#include <sensor_msgs/CompressedImage.h>
#include <tld_msgs/Target.h>

Q_DECLARE_METATYPE(sensor_msgs::CompressedImageConstPtr)
Q_DECLARE_METATYPE(tld_msgs::Target)

int main(int argc, char **argv) {

    /*********************
    ** Qt
    **********************/
    QApplication app(argc, argv);


    qRegisterMetaType<sensor_msgs::CompressedImageConstPtr>("sensor_msgs::CompressedImageConstPtr");
    qRegisterMetaType<tld_msgs::Target>("tld_msgs::Target");
    


    MainWindow w;
    w.show();
    ros::init(argc, argv, "listener");


    ros::NodeHandle node;  
    Test test(node);
    test.start();  
    QObject::connect(&test, SIGNAL(ImageReceived(sensor_msgs::CompressedImageConstPtr)), &w, SLOT(ImageShow(sensor_msgs::CompressedImageConstPtr)));
    QObject::connect(w.ui->widget, SIGNAL(signalSelectRect(tld_msgs::Target)), &test, SLOT(selectRectReceived(tld_msgs::Target)));

    int result = app.exec();

	return result;
}
