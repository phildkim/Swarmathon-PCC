#include <cstdint>
#include <iostream>
#include <string>

#include "ros/ros.h"
#include "GridService.h"
#include "GridService.cpp"

int main(int argc, char **argv) {
	ros::init(argc, argv, "registration_testing");
	GridService server;
	
	ros::ServiceServer is_occupied_all =		server.registerStatusGetter("is_occupied_all",			0, 0, (1 << 0) | (1 << 1));
	ros::ServiceServer is_occupied_obstacle =	server.registerStatusGetter("is_occupied_obstacle",		0, 0, 1		);
	ros::ServiceServer is_occupied_rover =		server.registerStatusGetter("is_occupied_rover",		0, 1, 1 << 1	);
	ros::ServiceServer is_occupied_planned_route =	server.registerStatusGetter("is_occupied_planned_route",	0, 2, 1 << 2	);	
	ros::ServiceServer has_samples =		server.registerStatusGetter("has_samples",			0, 3, 1 << 3	);
	ros::ServiceServer has_been_searched =		server.registerStatusGetter("has_been_searched",		0, 4, 1 << 4	);
	ros::ServiceServer get_num_of_samples =		server.registerStatusGetter("get_num_of_samples",		1, 0, ~0	);
	
	ros::ServiceServer set_occupied_obstacle =	server.registerStatusSetter("set_occupied_obstacle",		0, 0, 1		);
	ros::ServiceServer set_occupied_rover =		server.registerStatusSetter("set_occupied_rover",		0, 1, 1 << 1	);
	ros::ServiceServer set_occupied_planned_route =	server.registerStatusSetter("set_occupied_planned_route",	0, 2, 1 << 2	);
	ros::ServiceServer mark_samples =		server.registerStatusSetter("mark_samples",			0, 3, 1 << 3	);
	ros::ServiceServer mark_searched =		server.registerStatusSetter("mark_searched",			0, 4, 1 << 4	);
	ros::ServiceServer update_num_of_samples =	server.registerStatusSetter("update_num_of_samples",		1, 0, ~0	);

	ros::spin();
	
	return 0;
}
