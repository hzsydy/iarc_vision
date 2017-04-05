#include <iostream>
#include "ros/ros.h"
#include "FSM.h"
#include "geometry_msgs/Vector3.h"
#include <vector>

std::vector<float> pos(3);
FSM fsm;

void fsmCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
	pos[0] = msg->x;
	pos[1] = msg->y;
	pos[2] = msg->z;

	fsm.changeAction(pos);
	fsm.changeStatus();

	switch(fsm.getStatus())
	{
		case Up:
			ROS_INFO("Fly up.");
			break;
		case Down:
			ROS_INFO("Fly Down.");
			break;
		case Wait:
			ROS_INFO("Is waiting.");
			break;
		case FlyTo:
			ROS_INFO("Fly to object.The angle is %f", fsm.getAngle());
			break;
		default:
			break;
	}
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "FSM_node");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("planePos", 20, fsmCallback);
	ros::spin();
	return 0;
}
