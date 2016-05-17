#ifndef QIMAGEVIEWER_H
#define QIMAGEVIEWER_H

#include <QWidget>
#include <QMouseEvent>
#include <QPixmap>
#include <QPoint>
#include "croprect.h"

#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
//#include <tld_msgs/BoundingBox.h>
#include <tld_msgs/Target.h>
class QImageViewer : public QWidget
{
    Q_OBJECT
public:
    explicit QImageViewer(QWidget *parent = 0);
    void paintEvent(QPaintEvent *event);

    bool isContainPoint(QPoint p);
    QPoint mapToPixmap(QPoint screenPoint);

    void setPixmap(const QPixmap& pixmap);
    void setImage(const cv_bridge::CvImagePtr& image);

    void mousePressEvent(QMouseEvent *event);

    void mouseMoveEvent(QMouseEvent *event);

    void mouseReleaseEvent(QMouseEvent *e);

    void cropFinished();

Q_SIGNALS:
    void signalSelectRect(const tld_msgs::Target& msg);

private:
    QPixmap m_pixmap;
    cv_bridge::CvImagePtr m_image;
    float scaling;
    bool isLoaded;
    bool isInitialised;
    bool isCropping;
    bool isStartingCrop;

    CropRect cropRect;


};

#endif // QIMAGEVIEWER_H
