/** 
 *  @file    Grid.h
 *  @author  Eric Tung (Treayn)
 *  @date    2/06/2017
 *  @version 1.0 
 *  
 *  @brief Grid Class
 *
 *  @section DESCRIPTION
 *  
 *  The Grid Class has three important functions:
 *  
 *  Firstly, the Grid Class is responsible for updating
 *    the state and size of the grid based on data
 *    recieved from the local and/or remote rovers. 
 *  
 *  Secondly, given the state of the Grid, the Grid Class
 *    periodically runs algorithms which allow it to
 *    generate statistics such as:
 *      - Percentage of map explored
 *      - Probability of soil samples in an area of the map
 *      - Probability of collisions with objects in an area
 *          of the map
 *      - etc.
 *  
 *  And Thirdly, the Grid Class is designed to expose an
 *    API which allows for simple addition of Remote
 *    Procedure Calls (in the form of ROS Services) to
 *    be easily added to monitor new statuses and
 *    statistics.
**/

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
	
        ros::NodeHandle node;
	
	// Grid variables
	std::deque<column_t> grid;
	uint8_t x_size, y_size;
        uint8_t x_center, y_center;
	
	// Grid mutators
	
	/**
	 *  Grid::createCell()
         *  ------------------
         *  Cell() factory function. Call this instead of Cell()
	 *    or Derived class constructors to create new Cells
	 *    in the Grid.
	**/
        std::unique_ptr<Cell> createCell();
	void addColumns(int8_t); 
	void addRows(int8_t);
	
	// Cell accessors & mutators
        bool getStatus(get_status_request_t&, get_status_response_t&, uint8_t, uint8_t);
        bool getStatus(get_status_request_t&, get_status_response_t&, uint8_t, uint8_t, uint8_t);
        bool setStatus(set_status_request_t&, set_status_response_t&, uint8_t, uint8_t, uint8_t);
	
	// Convenience functions
	Cell * getCell(int8_t, int8_t);
	void checkGridBounds(int8_t, int8_t);

public:
	Grid();
	Grid(uint8_t);
	~Grid();
	
	/**
	 *  Grid::registerStatusGetter(),
	 *  Grid::registerStatusSetter()
	 *  -----------------------------
	 *  These functions allow for Dynamic Callback
	 *    Registration for ROS Services. They create
	 *    (getter and setter) closures which may be invoked
	 *    by a future Remote Procedure Call.
	 *  
         *  These functions accept four arguments:
         *  
	 *  The first argument specifies a parameter name to
	 *    query for on the server.
	 *  
	 *  Arguments 2, 3, and 4 specify where the parameter
	 *    resides in memory.
	 *
	 *  See std::function and std::bind for more details.
	 *    (ROS uses boost::function and boost::bind internally
	 *    for compatibility with C++03.)
	**/
        ros::ServiceServer registerStatusGetter(const std::string&, uint8_t, uint8_t);
        ros::ServiceServer registerStatusGetter(const std::string&, uint8_t, uint8_t, uint8_t);
        ros::ServiceServer registerStatusSetter(const std::string&, uint8_t, uint8_t, uint8_t);
};

#endif
