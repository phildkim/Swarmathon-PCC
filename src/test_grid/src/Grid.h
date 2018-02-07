#ifndef GRID_H
#define GRID_H

#include <cstdint>
#include <deque>
#include <memory>
#include <string>

#include "AllocatedCell.h"
#include "MappedCell.h"
#include "ros/ros.h"

#include <ccny_srvs/GetStatus.h>
#include <ccny_srvs/SetStatus.h>

class Grid {
private:
	// Grid typedefs
        typedef std::unique_ptr<Cell>           cell_t;
        typedef std::deque<cell_t>              column_t;
        typedef std::deque<column_t>            row_t;
	
	// Message typedefs
        typedef ccny_srvs::GetStatus::Request   get_status_request_t;
        typedef ccny_srvs::GetStatus::Response  get_status_response_t;
        typedef ccny_srvs::SetStatus::Request   set_status_request_t;
        typedef ccny_srvs::SetStatus::Response  set_status_response_t;
	
	// Grid variables
	std::deque<column_t> grid;
	uint8_t x_size, y_size;
        uint8_t x_center, y_center;
	
        ros::NodeHandle node;
	
	// Grid mutators
        std::unique_ptr<Cell> createCell();
	void addColumn(int8_t); 
	void addRow(int8_t);
	
	// Cell accessors & mutators
        bool getStatus(get_status_request_t&, get_status_response_t&, uint8_t, uint8_t);
        bool getStatus(get_status_request_t&, get_status_response_t&, uint8_t, uint8_t, uint8_t);
        bool setStatus(set_status_request_t&, set_status_response_t&, uint8_t, uint8_t, uint8_t);
	
	// Convenience functions
	Cell * getCell(int8_t, int8_t);

public:
	Grid();
	Grid(uint8_t);
	~Grid();
	
	// Dynamic Callback Registration
	// -----------------------------
	// These functions accept four arguments, and create (getter and setter) closures,
	//	which may be invoked at a future point in time.
	//
	// The first argument specifies a parameter name to query for on the server.
	//	Arguments 2, 3, and 4 specify where the parameter resides in memory.
	//
	// See std::function and std::bind for more details. (ROS uses boost::function
	//	 and boost::bind internally for compatibility with C++03.)
        ros::ServiceServer registerStatusGetter(const std::string&, uint8_t, uint8_t);
        ros::ServiceServer registerStatusGetter(const std::string&, uint8_t, uint8_t, uint8_t);
        ros::ServiceServer registerStatusSetter(const std::string&, uint8_t, uint8_t, uint8_t);
};

#endif
