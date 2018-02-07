#include <cstdint>
#include <functional>
#include <iostream>

#include <unistd.h>
#include <sys/types.h>
#include <sys/mman.h>

#include "ros/ros.h"
#include "ccny_srvs/GetCoordinates.h"
#include "ccny_srvs/SetCoordinates.h"

bool getCoordinateData(ccny_srvs::GetCoordinates::Request& req, ccny_srvs::GetCoordinates::Response& res, uint8_t * ptr) {
	res.data = *(ptr + req.offset);
	ROS_INFO("Reading from address [%p]", (ptr + req.offset));
}

bool setCoordinateData(ccny_srvs::SetCoordinates::Request& req, ccny_srvs::SetCoordinates::Response& res, uint8_t * ptr) {
	*(ptr + req.offset) = req.data;
	ROS_INFO("Writing to address [%p]", (ptr + req.offset));
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "mmap_testing");
	
	uint8_t * base_addr = (uint8_t *) mmap(
		NULL,
		getpagesize(),
		PROT_READ | PROT_WRITE,
		MAP_ANONYMOUS | MAP_PRIVATE,
		-1,
		0
	);
	
	if(*base_addr == -1) {
		ROS_INFO("Failed to allocate Memory Map.");
		return 1;
	}
	
	ros::NodeHandle node;
	ros::ServiceServer get_coordinate_data = node.advertiseService<ccny_srvs::GetCoordinates::Request, ccny_srvs::GetCoordinates::Response>(
		"get_coordinate_data",
		boost::bind(&getCoordinateData, _1, _2, boost::ref(base_addr))
	);
	
	ros::ServiceServer set_coordinate_data = node.advertiseService<ccny_srvs::SetCoordinates::Request, ccny_srvs::SetCoordinates::Response>(
		"set_coordinate_data",
		boost::bind(&setCoordinateData, _1, _2, boost::ref(base_addr))
	);
	
	ros::spin();

	munmap(base_addr, getpagesize());
	return 0;
}
