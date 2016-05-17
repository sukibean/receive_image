#include "qimageviewer.h"
#include <QPainter>
#include <QDebug>
//#include <tld_msgs/Target.h>
#include <tld_msgs/Target.h>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;

extern QImage Mat2QImage(Mat& image);

QImageViewer::QImageViewer(QWidget *parent)
    : QWidget(parent)
    , isStartingCrop(false)
    , isLoaded(true)
    , isCropping(true)
{

}

void QImageViewer::setPixmap(const QPixmap& pixmap){
    m_pixmap = pixmap;
    update();
}

void QImageViewer::setImage(const cv_bridge::CvImagePtr& cvi)
{
    m_image = cvi;
    setPixmap(QPixmap::fromImage(Mat2QImage(cvi->image)));
}

void QImageViewer::paintEvent(QPaintEvent *event)
{
    QWidget::paintEvent(event);
    if (m_pixmap.isNull())
    {
        return;
    }

    QPainter painter(this);
    if (isLoaded)
    {
        painter.setRenderHint(QPainter::SmoothPixmapTransform);
        QSize pixSize = m_pixmap.size();

        //For canvas's size not change when window's size change.


        if (!isInitialised)
        {
            QSize initialSize = event->rect().size();
            scaling = 1.0 * initialSize.width() / pixSize.width();
            isInitialised = true;
        }
        pixSize.scale(scaling * pixSize, Qt::KeepAspectRatio);
        this->setMinimumSize(pixSize);

        QPoint topleft;
        topleft.setX((this->width() - pixSize.width()) / 2);
        topleft.setY((this->height() - pixSize.height()) / 2);

        painter.drawPixmap(topleft, m_pixmap.scaled(pixSize, Qt::KeepAspectRatio, Qt::SmoothTransformation));

        if (isCropping)
        {
//            qDebug() << cropRect.width() << cropRect.height();
            //painter.setPen(Qt::darkGreen);
            QPen pen;
            pen.setBrush(Qt::darkGreen);
            pen.setStyle(Qt::DashLine);
            pen.setWidth(1);
            painter.setPen(pen);

            //start point in the left to the end point.
            if (cropRect.startPoint().x() < cropRect.endPoint().x())
            {

                if (cropRect.startPoint().y() < cropRect.endPoint().y())
                {
                    //start point in the top to the end point.
                    painter.drawRect(topleft.x() + cropRect.startPoint().x() * scaling, topleft.y() + cropRect.startPoint().y() * scaling, cropRect.width() * scaling, cropRect.height() * scaling);
                }
                else{
                    //start point in the bottom to the end point.
                    painter.drawRect(topleft.x() + cropRect.startPoint().x() * scaling, topleft.y() + cropRect.endPoint().y() * scaling, cropRect.width() * scaling, cropRect.height() * scaling);
                }
            }
            else
            {
                if (cropRect.startPoint().y() > cropRect.endPoint().y())
                {
                    painter.drawRect(topleft.x() + cropRect.endPoint().x() * scaling, topleft.y() + cropRect.endPoint().y() * scaling, cropRect.width() * scaling, cropRect.height() * scaling);
                }
                else{
                    painter.drawRect(topleft.x() + cropRect.endPoint().x() * scaling, topleft.y() + cropRect.startPoint().y() * scaling, cropRect.width() * scaling, cropRect.height() * scaling);
                }
            }
        }

    }

}

bool QImageViewer::isContainPoint(QPoint p)
{
    QSize s = m_pixmap.size();
    s.scale(scaling * s, Qt::KeepAspectRatio);

    //If pixmap bigger than current window.
    if ((s.height() > this->rect().height()) && (s.width() > this->rect().width()))
    {
        return true;
    }

    QPoint topleft;
    topleft.setX((this->width() - s.width()) / 2);
    topleft.setY((this->height() - s.height()) / 2);

    QRect rect(topleft, s);
    return rect.contains(p);
}

QPoint QImageViewer::mapToPixmap(QPoint screenPoint)
{
    QSize pixmapSize = m_pixmap.size();
    pixmapSize.scale(scaling * pixmapSize, Qt::KeepAspectRatio);

    //Get the position of screenPoint to the pixmap in show.
    QPoint tmpPos;
    if (pixmapSize.width() > this->width() && pixmapSize.height() > this->height())
    {
        tmpPos.setX(pixmapSize.width() - (this->width() - screenPoint.x()));
        tmpPos.setY(pixmapSize.height() - (this->height() - screenPoint.y()));
    }
    else if (pixmapSize.width() < this->width() && pixmapSize.height() > this->height())
    {
        tmpPos.setX(screenPoint.x() - (this->width() - pixmapSize.width()) / 2);
        tmpPos.setY(pixmapSize.height() - (this->height() - screenPoint.y()));
    }
    else if (pixmapSize.width() > this->width() && pixmapSize.height() < this->height())
    {
        tmpPos.setX(pixmapSize.width() - (this->width() - screenPoint.x()));
        tmpPos.setY(screenPoint.y() - (this->height() - pixmapSize.height()) / 2);
    }
    else{
        QPoint topleft;
        topleft.setX((this->width() - pixmapSize.width()) / 2);
        topleft.setY((this->height() - pixmapSize.height()) / 2);
        tmpPos.setX(screenPoint.x() - topleft.x());
        tmpPos.setY(screenPoint.y() - topleft.y());
    }
    //return the position to the real pixmap.*/
    return QPoint(tmpPos.x() / scaling, tmpPos.y() / scaling);
}

void QImageViewer::mousePressEvent(QMouseEvent *event)
{
    if ((event->buttons() == Qt::LeftButton) && isContainPoint(event->pos()) && isCropping)
    {
        cropRect.setStart(mapToPixmap(event->pos()));
        cropRect.setEnd(mapToPixmap(event->pos()));
        isStartingCrop = true;
    }
}

void QImageViewer::mouseMoveEvent(QMouseEvent *event)
{
    if ((event->buttons() == Qt::LeftButton) && isStartingCrop)
    {
        if (isContainPoint(event->pos()))
        {
            cropRect.setEnd(mapToPixmap(event->pos()));
            update();
        }
    }
}

void QImageViewer::mouseReleaseEvent(QMouseEvent *e)
{
    if(m_image == 0){
        return;
    }
    QRect rect(cropRect.startPoint(), cropRect.endPoint());
    isStartingCrop = false;
    tld_msgs::Target msg;
    tld_msgs::BoundingBox bb;
    bb.header = m_image->header;
    bb.x = rect.x();
    bb.y = rect.y();
    bb.width = rect.width();
    bb.height = rect.height();
    msg.bb = bb;
    m_image->toImageMsg(msg.img);
    Q_EMIT signalSelectRect(msg);
}

void QImageViewer::cropFinished()
{
    QRect crop(cropRect.startPoint(), QSize(cropRect.width(), cropRect.height()));
    QPixmap cropped = m_pixmap.copy(crop);
    m_pixmap = cropped;
    cropRect.reset();
    isCropping = false;
    this->update();
}

