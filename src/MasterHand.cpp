#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "math.h"

int global = 0;

int main (int argc, char** argv)
{
	ros::init(argc, argv, "MasterHand");

	ros::NodeHandle n1;
        ros::Publisher uav1_pub =     			n1.advertise<geometry_msgs::Twist>("uav1/cmd_vel", 1000);
	ros::NodeHandle n2;
	ros::Publisher uav2_pub = n2.advertise<geometry_msgs::Twist>("uav2/cmd_vel", 1000);
	ros::NodeHandle n3;
	ros::Publisher uav3_pub = n3.advertise<geometry_msgs::Twist>("uav3/cmd_vel", 1000);

  	ros::Rate loop_rate(10);

	float pi = 3.1415926;
  	int count = 0;
  	while (ros::ok())
  	{
		/*
    		std_msgs::String msg;

    		std::stringstream ss;
    		ss << "hello world " << count;
    		msg.data = ss.str();

    		ROS_INFO("%s", msg.data.c_str());

    		chatter_pub.publish(msg);
		*/
		float upward = sin(pi * ((float)count / 16.0));
		//float side2side = sin(pi * ((float)count / 32.0));
		//float spin = sin(pi * ((float)count / 16));
		ROS_INFO("Z vel: %f", upward);

		geometry_msgs::Twist twist1;
		geometry_msgs::Vector3 linear1;
		linear1.x = 0;
		linear1.y = 0;
		linear1.z = upward;		

		geometry_msgs::Vector3 angular1;
		angular1.x = 0;
		angular1.y = 0;
		angular1.z = 0;

		twist1.linear = linear1;
		twist1.angular = angular1;

		twist1.linear = linear1;
		twist1.angular = angular1;

		uav1_pub.publish(twist1);
		uav2_pub.publish(twist1);
		uav3_pub.publish(twist1);

    		ros::spinOnce();

    		loop_rate.sleep();
    		++count;
  	}


  	return 0;

}
