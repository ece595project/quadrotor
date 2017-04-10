#include "ros/ros.h"
#include "geometry/Twist"

void swarmCommandCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "QuadBrain");
	ros::NodeHandle n;
	ros::Publisher cmd_vel_pub;
	ros::Subscriber swarm_cmd_sub;
	
	if (argc < 2)
	{
		char* cmd_vel = argv[0];
		ROS_INFO("Taking control of command velocity: [%s]", cmd_vel);
		cmd_vel_pub = n.publisher(cmd_vel, 1000, swarmCommandCallback);	
		char* swarm_cmd = argv[1];
		swarm_cmd_sub = n.subscriber(
		ROS_INFO("Tunning in to");
	}
	else
	{
		return 1;
	}

	
	
	ros::loop_rate(60); //Running at 60 hertz
	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
