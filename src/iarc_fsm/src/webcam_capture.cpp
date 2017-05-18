#include <opencv2/highgui/highgui.hpp>
#include "ros/ros.h"
#include <string>
#include <stdio.h>
#include <ueye.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include "UEyeOpenCV.hpp"
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

    
    Mat frame;
    sensor_msgs::ImagePtr msg;
    UeyeOpencvCam cam = UeyeOpencvCam(752,480);

    ros::Rate loop_rate(50);
    while(ros::ok())
    {
	frame = cam.getFrame();
	if(!frame.empty())
	{
	    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
	    pub.publish(msg);
	}
	ros::spinOnce();
	loop_rate.sleep();
    }
}


