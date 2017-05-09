#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "ece595project/init_sine_path.h"
#include "ece595project/next_position.h"
#include "ece595project/set_position.h"
#include "ece595project/movement_status.h"
#include "ece595project/uav_status.h"
#include "ece595project/swarm_start.h"
#include "stdlib.h"

#define PI 3.14159265

class QuadBrain
{
	private:
		std::string _id;
		ros::NodeHandle _n;
		ros::ServiceClient _init_planner;
		ros::ServiceClient _get_next_position;
		ros::ServiceClient _set_next_position;
		ros::Publisher _status_pub;
		geometry_msgs::Point _current_point;
		//System priority
		int _priority_index = -1;
		int _priority_slots = -1;
		//Other swarm elements
		char** _swarm_elements;
		bool* _element_ready;
		int _element_count = -1;

		void defaultPlannerInit();

	public:
		enum QuadState { WaitingOnCommand = 2001, 
						 InitPlanner = 2002, 
						 MovingToStart = 2003, 
						 AtStart = 2004, 
						 Pathing = 2005, 
						 FindNextPoint = 2006, 
						 Finished = 2007} _current_state;
		QuadBrain(std::string);
		~QuadBrain();

		void quadUpdate();
		void movementStatusCallback(ece595project::movement_status);
		void uavStatusCallback(ece595project::uav_status);
		void swarmStartCallback(ece595project::swarm_start);
		void setPriority(int, int);
		void setSwarmElements(char**, int);
};

void QuadBrain::movementStatusCallback(ece595project::movement_status status)
{
	int state_code = status.state_code;
	if (state_code == 1006)
	{
		if (_current_state == MovingToStart)
		{
			ROS_INFO("At start");
			_current_state = AtStart;
		}
		else if (_current_state == Pathing)
		{
			ROS_INFO("Need next point");
			_current_state = FindNextPoint;
		}
	}	
}

void QuadBrain::uavStatusCallback(ece595project::uav_status status)
{
	int status_code = status.status_code;
	std::string uav_id = status.uav_id;
	int uav_priority = status.uav_priority;
	if (status_code == AtStart)
	{
		_element_ready[uav_priority] = true;
	}
}

void QuadBrain::swarmStartCallback(ece595project::swarm_start start_msg)
{
	ROS_INFO("current state = %d priority index = %d", _current_state, _priority_index); 
	if (_current_state == WaitingOnCommand && 
	_priority_index < start_msg.region_count &&
	_priority_index > -1)
	{
		ece595project::init_sine_path init;
		init.request.origin = start_msg.origin;
		init.request.path_freq = 2 * PI;
		init.request.increment_per_call = 1.0 / 16.0;
		init.request.region_degree_start = start_msg.region_start_degrees[_priority_index];
		init.request.region_degree_stop = start_msg.region_stop_degrees[_priority_index];
		init.request.start_offset = start_msg.start_offsets[_priority_index];
		//ROS_INFO("Trying to init");
		if (_init_planner.call(init))
		{
			_current_point = init.response.start_point;
			_current_state = InitPlanner;
			ROS_INFO("Initializing path planner x = %f y = %f z = %f", _current_point.x, _current_point.y, _current_point.z);
		}
	}
}

void QuadBrain::quadUpdate()
{
	ece595project::uav_status status;
	status.uav_id = _id;
	status.uav_priority = _priority_index;
	switch(_current_state)
	{
		case WaitingOnCommand:
		{
			status.status_code = (int)WaitingOnCommand;
			status.status_name = "Waiting on Command";
			break;
		}
		case InitPlanner:
		{
			status.status_code = (int)InitPlanner;
			status.status_name = "Initialize Planner";
			ece595project::set_position new_set_request;
			new_set_request.request.new_position = _current_point;
			new_set_request.request.movement_speed = 1.0;
			new_set_request.request.stop_at_point = true;
			if (_set_next_position.call(new_set_request))
			{
				ROS_INFO("Moving to start");
				_current_state = MovingToStart;
			}
			break;
		}
		case MovingToStart:
		{
			status.status_code = (int)MovingToStart;	
			status.status_name = "Moving to Start";
			break;
		}
		case AtStart:
		{
			status.status_code = (int)AtStart;
			status.status_name = "At Start";
			bool allReady = true;
			for (int i = 0; i < _priority_slots; i++)
			{
				if (!_element_ready[i])
				{	
					allReady = false;
				}
			}
			if (allReady)
			{
				_current_state = FindNextPoint;
			}
			break;
		}
		case Pathing:
		{
			status.status_code = (int)Pathing;	
			status.status_name = "Pathing";
			break;
		}
		case FindNextPoint:
		{
			status.status_code = (int)FindNextPoint;	
			status.status_name = "Find Next Point";
			ece595project::next_position new_next_request;
			new_next_request.request.current_attainable = true;
			if (_get_next_position.call(new_next_request))
			{
				_current_point = new_next_request.response.next_point;
				ece595project::set_position new_set_request;
				new_set_request.request.new_position = _current_point;
				new_set_request.request.movement_speed = 1.0;
				new_set_request.request.stop_at_point = false;
				if (_set_next_position.call(new_set_request))
				{
					ROS_INFO("Next position at x = %f y = %f z = %f", _current_point.x, _current_point.y, _current_point.z);
					_current_state = Pathing;
				}
			}
			break;
		}
		case Finished:
		{
			status.status_code = (int)Finished;	
			status.status_name = "Finished";
			break;
		}
	}
	status.current_destination = _current_point;
	_status_pub.publish(status);
}

void QuadBrain::defaultPlannerInit()
{
	ece595project::init_sine_path init;
	geometry_msgs::Point origin;
	origin.x = 0.0;
	origin.y = 0.0;
	origin.z = 0.0;
	init.request.origin = origin;
	init.request.path_freq = 2 * PI;
	init.request.increment_per_call = 1.0 / 16.0;
	init.request.region_degree_start = (2.0 * PI) / 3.0;
	init.request.region_degree_stop = (4.0 * PI) / 3.0;
	init.request.start_offset = 1.0;
	ROS_INFO("Trying to init");
	if (_init_planner.call(init))
	{
		_current_point = init.response.start_point;
		ROS_INFO("Initializing path planner x = %f y = %f z = %f", _current_point.x, _current_point.y, _current_point.z);
	}
	
}

void QuadBrain::setPriority(int priority_index, int priority_slots)
{
	_priority_index = priority_index;
	_priority_slots = priority_slots;
} 

void QuadBrain::setSwarmElements(char** swarm_elements, int element_count)
{
	_swarm_elements = new char*[element_count];
	_swarm_elements = swarm_elements;
	_element_ready = new bool[_priority_slots];
	for (int i = 0; i < _priority_slots; i++)
	{
		_element_ready[i] = false;
	}
	_element_count = element_count;
}

QuadBrain::QuadBrain(std::string uav_id)
{
	_id = uav_id;
	_status_pub = _n.advertise<ece595project::uav_status>("uav_status_general", 100);
	_init_planner = _n.serviceClient<ece595project::init_sine_path>(uav_id + "/init_sine_path");
	_get_next_position = _n.serviceClient<ece595project::next_position>(uav_id + "/next_position");
	_set_next_position = _n.serviceClient<ece595project::set_position>(uav_id + "/set_position");
	_current_state = WaitingOnCommand;
	//defaultPlannerInit();
}

QuadBrain::~QuadBrain()
{
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "quad_brain");
	std::string id_str = "";
	//First arg is the name of uav
	if (argc >= 2)
	{
		id_str = argv[1];
	}
	else
	{
		ros::spin();
		return -1;
	}
	//Second arg is the priority of the uav
	int priority = -1;
	if (argc >= 3)
	{
		priority = atoi(argv[2]);
	}
	//Third arg is the number of priority slots in the swarm
	int priorities = -1;
	if (argc >= 4)
	{
		priorities = atoi(argv[3]);
	}
	//Remaining args are the other uavs in the swarm
	int uav_count = argc - 4;
	char** other_uavs;
	for (int i = 4; i < argc; i++)
	{
		other_uavs[i - 4] = argv[i];
	}

	QuadBrain brain(id_str);
	brain.setPriority(priority, priorities);
	brain.setSwarmElements(other_uavs, uav_count);
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe<ece595project::movement_status>(id_str + "/movement_status", 1, &QuadBrain::movementStatusCallback, &brain);
	ros::Subscriber sub1 = n.subscribe<ece595project::uav_status>("uav_status_general", 100, &QuadBrain::uavStatusCallback, &brain);
	ros::Subscriber sub2 = n.subscribe<ece595project::swarm_start>("swarm_start", 1, &QuadBrain::swarmStartCallback, &brain);

	ros::Rate loop_rate(60); //Running at 60 hertz
	while(ros::ok())
	{
		brain.quadUpdate();
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
