#include <opencv2/highgui/highgui.hpp>
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "webcam");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    image_transport::Publisher pub = it.advertise("/camera/color/image_raw",1);

    VideoCapture capture(0);
    if (!capture.isOpened())
    {
        ROS_INFO("Webcam doesn't open.\n");
        return 1;
    }
    
    Mat frame;
    sensor_msgs::ImagePtr msg;
    
    ros::Rate loop_rate(50);
    while(ros::ok())
    {
	capture >> frame;
	if(!frame.empty())
	{
	    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
	    pub.publish(msg);
	}
	ros::spinOnce();
	loop_rate.sleep();
    }
}


