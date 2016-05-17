#include <QThread>
#include "ros/ros.h"

#include <sensor_msgs/CompressedImage.h>
#include <tld_msgs/Target.h>

//#include <image_transport/image_transport.h>
//#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/image_encodings.h>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>

//using namespace cv;

class MainWindow;

class Test:public QThread
{  
    Q_OBJECT 
Q_SIGNALS:
    void ImageReceived(const sensor_msgs::CompressedImageConstPtr& msg); 

public:  

    Test(ros::NodeHandle node);

    ros::Subscriber sub;
    ros::Publisher pub;

    void chatterCallback(const sensor_msgs::CompressedImageConstPtr& msg);

 
public Q_SLOTS:
    void selectRectReceived(const tld_msgs::Target& msg);

 
public:  
    void run();  
};

