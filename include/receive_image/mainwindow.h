#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QWidget>
#include <sensor_msgs/CompressedImage.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

namespace Ui {
class MainWindow;
}

class MainWindow : public QWidget
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
public Q_SLOTS:
    void ImageShow(const sensor_msgs::CompressedImageConstPtr& msg);

public:
    Ui::MainWindow *ui;

    cv_bridge::CvImagePtr cvi;
};

#endif // MAINWINDOW_H
