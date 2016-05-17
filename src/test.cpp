#include "test.h"



Test::Test(ros::NodeHandle node)
{
    sub = node.subscribe("im_acq", 1000, &Test::chatterCallback, this);
    pub = node.advertise<tld_msgs::Target>("selectRect", 1000, true);
}


void Test::run() {
	ros::spin();//must be ros::spin()
}


void Test::chatterCallback(const sensor_msgs::CompressedImageConstPtr& msg) 
{
    Q_EMIT ImageReceived(msg);
}

void Test::selectRectReceived(const tld_msgs::Target& msg)
{
    pub.publish(msg);
}
