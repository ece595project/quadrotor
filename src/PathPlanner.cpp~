#include "ros/ros.h"
#include "ece595project/next_position.h"
#include "ece595project/init_sine_path.h"
#include "geometry_msgs/Point.h"
#include "math.h"
#define PI 3.14159265

class PathPlanner
{
	private:
		geometry_msgs::Point origin;
		double current_relative_x = 0.0;
		double increment_x;
		double region_start;
		double region_stop;
		double axis_angle;
		double amp_rate;
		double path_freq = 1.0;
		bool initialized = false;
		bool started = false;

	public:
		bool nextPositionCallback(ece595project::next_position::Request&, ece595project::next_position::Response&);
		bool initSinePathCallback(ece595project::init_sine_path::Request&, ece595project::init_sine_path::Response&);
		//bool startSinePathCallback(ece595project::start_sine_path::Request&, ece595project::start_sine_path::Response&);
		geometry_msgs::Point regionToWorld(geometry_msgs::Point);
		PathPlanner();	
		~PathPlanner();
} planner;

//Gets next position in sine trajectory
bool PathPlanner::nextPositionCallback(ece595project::next_position::Request& request, ece595project::next_position::Response& response)
{
	geometry_msgs::Point point;
	if (initialized)
	{
		point.x = current_relative_x;
		point.y = (amp_rate * current_relative_x) * sin(path_freq * current_relative_x); // basically x*sin(x)
		point.z = 1.0;
		current_relative_x += increment_x;
		response.next_point = regionToWorld(point);
		response.success = true;
	}
	else
	{
		response.next_point = point;
		response.success = false;
	}
	return true;
}

bool PathPlanner::initSinePathCallback(ece595project::init_sine_path::Request& request, ece595project::init_sine_path::Response& response)
{
	if (!initialized)
	{		
		//Set system variables
		origin = request.origin;
		path_freq = request.path_freq;
		increment_x = request.increment_per_call;
		region_start = request.region_degree_start;
		region_stop = request.region_degree_stop;
		double local_axis_angle = (region_stop - region_start) / 2;
		amp_rate = tan(local_axis_angle);
		axis_angle = region_start + local_axis_angle;

		//set current x based on offset
		int increments = (int)(request.start_offset / increment_x);
		current_relative_x = increments * increment_x;
		
		//Return a starting point
		geometry_msgs::Point pnt;
		pnt.x = request.start_offset;
		pnt.y = 0.0;
		pnt.z = 1.0;
			
		response.start_point = regionToWorld(pnt);
		response.success = true;		

		initialized = true;
	}
	return true;
}

//Converts local position to world position
//Does not convert z coordinate, only x and y
geometry_msgs::Point PathPlanner::regionToWorld(geometry_msgs::Point localPoint)
{
	geometry_msgs::Point worldPoint;
	double local_x = localPoint.x;
	double local_y = localPoint.y;
	double local_angle = atan(local_y / local_x);
	double world_angle = axis_angle - local_angle;
	double local_dist = sqrt( pow( local_x, 2) + pow( local_y, 2));
	ROS_INFO("angle = %f distance = %f local_x = %f local_y = %f", local_angle, local_dist, local_x, local_y);
	worldPoint.x = origin.x + (local_dist * sin(world_angle));
	worldPoint.y = origin.y + (local_dist * cos(world_angle));
	worldPoint.z = localPoint.z;

	return worldPoint;
}

PathPlanner::PathPlanner()
{
}

PathPlanner::~PathPlanner()
{
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "path_planner");
	ros::NodeHandle n;
	std::string id_str = "";
	if (argc >= 2)
	{
		id_str = argv[1];
	}

	ros::ServiceServer srv1 = n.advertiseService(id_str + "/next_position", &PathPlanner::nextPositionCallback, &planner);
	ros::ServiceServer srv2 = n.advertiseService(id_str + "/init_sine_path", &PathPlanner::initSinePathCallback, &planner);

	ros::Rate loop_rate(60);
	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
