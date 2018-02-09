#ifndef GRID_SERVICE_H
#define GRID_SERVICE_H

#include <cstdint>
#include <memory>
#include <string>

#include <ros/ros.h>
#include <ccny_srvs/GetStatus.h>
#include <ccny_srvs/SetStatus.h>

//#include <Grid.h>

class GridService {
private:
	// Grid grid;
	uint8_t data;
	ros::NodeHandle node;
	
	// For Boolean Values.	
	bool getStatus(ccny_srvs::GetStatus::Request&, ccny_srvs::GetStatus::Response&, uint8_t, uint8_t);
	
	// For Integer Values.
	bool getStatus(ccny_srvs::GetStatus::Request&, ccny_srvs::GetStatus::Response&, uint8_t, uint8_t, uint8_t);
	bool setStatus(ccny_srvs::SetStatus::Request&, ccny_srvs::SetStatus::Response&, uint8_t, uint8_t, uint8_t);
	
public:
	GridService();
	~GridService();
	
//	Dynamic Callback Registration
//	-----------------------------
//	These functions accept four arguments, and create (getter and setter) closures,
//		which may be invoked at a future point in time.
//
//	The first argument specifies a parameter name to query for on the server. Arguments 2, 3, and 4
//		specify where the parameter resides in memory.
//
//	See std::function and std::bind for more details.
//	(ROS is actually using boost::function and boost::bind for compatibility with older versions of C++.)
	
	ros::ServiceServer registerStatusGetter(const std::string&, uint8_t, uint8_t);
	ros::ServiceServer registerStatusGetter(const std::string&, uint8_t, uint8_t, uint8_t);
	ros::ServiceServer registerStatusSetter(const std::string&, uint8_t, uint8_t, uint8_t);
};

#endif
