#include <cstdint>
#include <iostream>
#include <string>

#include <ros/ros.h>
#include <ccny_srvs/GetStatus.h>
#include <ccny_srvs/SetStatus.h>

uint8_t d = 0;

bool getStatus(ccny_srvs::GetStatus::Request& req, ccny_srvs::GetStatus::Response& res, uint8_t stride, uint8_t offset, uint8_t mask) {
	res.data = (d & mask) >> offset;
	res.success = true;
	return true;
}

bool setStatus(ccny_srvs::SetStatus::Request& req, ccny_srvs::SetStatus::Response& res, uint8_t stride, uint8_t offset, uint8_t mask) {
	if(req.data != (d & mask) >> offset)
		d ^= (1 << offset);
	
	res.success = true;
	return true;
}

/*      Dynamic Callback Registration
        -----------------------------
        These functions accept four arguments, and create (getter and setter) closures,
		which may be invoked at a future point in time.
        
        The first argument specifies a parameter name to query for on the server. Arguments 2, 3, and 4
                specify where the parameter resides in memory.

        (See std::function and std::bind for more details.)
*/
ros::ServiceServer registerStatusGetter(const std::string &name, uint8_t stride, uint8_t offset, uint8_t mask) {
	return node.advertiseService<ccny_srvs::GetStatus::Request, ccny_srvs::GetStatus::Response>(name, boost::bind(getStatus, _1, _2, stride, offset, mask));
}

ros::ServiceServer registerStatusSetter(const std::string &name, uint8_t stride, uint8_t offset, uint8_t mask) {
        return node.advertiseService<ccny_srvs::SetStatus::Request, ccny_srvs::SetStatus::Response>(name, boost::bind(setStatus, _1, _2, stride, offset, mask));
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "registration_testing");
	ros::NodeHandle node;
	
	ros::ServiceServer is_occupied_all =		registerStatusGetter("is_occupied_all",			0, 0, (1 << 0) | (1 << 1));
	ros::ServiceServer is_occupied_obstacle =	registerStatusGetter("is_occupied_obstacle",		0, 0, 1		);
	ros::ServiceServer is_occupied_rover =		registerStatusGetter("is_occupied_rover",		0, 1, 1 << 1	);
	ros::ServiceServer is_occupied_planned_route =	registerStatusGetter("is_occupied_planned_route",	0, 2, 1 << 2	);	
	ros::ServiceServer has_samples =		registerStatusGetter("has_samples",			0, 3, 1 << 3	);
	ros::ServiceServer has_been_searched =		registerStatusGetter("has_been_searched",		0, 4, 1 << 4	);
	ros::ServiceServer get_num_of_samples =		registerStatusGetter("get_num_of_samples",		1, 0, ~0	);
	
	ros::ServiceServer set_occupied_obstacle =	registerStatusSetter("set_occupied_obstacle",		0, 0, 1		);
	ros::ServiceServer set_occupied_rover =		registerStatusSetter("set_occupied_rover",		0, 1, 1 << 1	);
	ros::ServiceServer set_occupied_planned_route =	registerStatusSetter("set_occupied_planned_route",	0, 2, 1 << 2	);
	ros::ServiceServer mark_samples =		registerStatusSetter("mark_samples",			0, 3, 1 << 3	);
	ros::ServiceServer mark_searched =		registerStatusSetter("mark_searched",			0, 4, 1 << 4	);
	ros::ServiceServer update_num_of_samples =	registerStatusSetter("update_num_of_samples",		1, 0, ~0	);

	ros::spin();
	
	return 0;
}
