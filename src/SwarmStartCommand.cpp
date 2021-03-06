#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "ece595project/swarm_start.h"

#define PI 3.14159265

int main (int argc, char** argv)
{
	ros::init(argc, argv, "swarm_start_command");
	ece595project::swarm_start start_msg;	

	geometry_msgs::Point origin;
	origin.x = 0.0;
	origin.y = 0.0;
	origin.z = 0.0;
	start_msg.origin = origin;
	
	std::vector<double> region_start_degrees;
	region_start_degrees.push_back( 0.0 );
	region_start_degrees.push_back( (2.0 * PI) / 3.0 );
	region_start_degrees.push_back( (4.0 * PI) / 3.0 );
	start_msg.region_start_degrees = region_start_degrees;

	std::vector<double> region_stop_degrees;
	region_stop_degrees.push_back( (2.0 * PI) / 3.0 );
	region_stop_degrees.push_back( (4.0 * PI) / 3.0 );
	region_stop_degrees.push_back( 2.0 * PI );
	start_msg.region_stop_degrees = region_stop_degrees;

	std::vector<double> start_offsets;
	start_offsets.push_back( 1.0 );
	start_offsets.push_back( 1.0 );
	start_offsets.push_back( 1.0 );
	start_msg.start_offsets = start_offsets;

	start_msg.region_count = 3;

	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<ece595project::swarm_start>("swarm_start", 1);

	ros::Rate loop_rate(1);
	while(ros::ok())
	{
		ROS_INFO("Publish");
		pub.publish(start_msg);

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
