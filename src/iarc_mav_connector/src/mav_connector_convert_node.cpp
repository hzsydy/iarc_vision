#include <ros/ros.h>
#include <iarc_mav_connector/mav_connector_msg.h>
#include <iostream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mav_connector_convert_node");
    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<iarc_mav_connector::mav_connector_msg>("mav_connector_chatter", 1000);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate loop_rate(10);
    
    float x=0, y=0, z=0;
    
    while(ros::ok())
    {
	iarc_mav_connector::mav_connector_msg msg;
	printf("waiting for [x y z]:   ");
	std::cin>>x>>y>>z;
	printf("get\n");
	msg.x = x;
	msg.y = y;
	msg.z = z;
	ROS_INFO("SENT:  %f  %f  %f", msg.x, msg.y, msg.z);
	chatter_pub.publish(msg);
    	ros::spinOnce();
    	loop_rate.sleep();
    }

    return 0;
}
