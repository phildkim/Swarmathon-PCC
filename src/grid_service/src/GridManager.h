/** 
 *  @file    GridManager.h
 *  @author  Eric Tung (Treayn) & Ronuel Diaz (rdiaz93)
 *  @date    2/06/2017
 *  @version 2.0.0
 *  
 *  @brief Grid Class
 *
 *  @section DESCRIPTION
 *  
 *  The GridManager Class has three important functions:
 *  
 *  Firstly, the GridManager Class is responsible for updating
 *    the state and size of the grid based on data
 *    recieved from the local and/or remote rovers. 
 *  
 *  Secondly, given the state of the Grid, the GridManager Class
 *    periodically runs algorithms which allow it to
 *    generate statistics such as:
 *      - Percentage of map explored
 *      - Probability of soil samples in an area of the map
 *      - Probability of collisions with objects in an area
 *          of the map
 *      - etc.
 *  
 *  And Thirdly, the GridManager Class is designed to expose an
 *    API which allows for simple addition of Remote
 *    Procedure Calls (in the form of ROS Services) to
 *    be easily added to monitor new statuses and
 *    statistics.
**/

#ifndef GRID_MANAGER_H
#define GRID_MANAGER_H

#include <cstdint>
#include <deque>
#include <list>
#include <map>
#include <memory>
#include <string>

#include "AllocatedCell.h"
#include "MappedCell.h"
#include "ros/ros.h"

#include "ccny_srvs/GetAddress.h"
#include "ccny_srvs/GetStatistic.h"
#include "ccny_srvs/SetStatistic.h"
#include "ccny_srvs/GetStatus.h"
#include "ccny_srvs/SetStatus.h"

class GridManager {
private:
	// 
	typedef std::function<float(int8_t, int8_t, int8_t, int8_t)>	operation_t;

	// Inner class
	class Grid {
	private:
		// Specify Layout of cell here.
        struct Layout {
			uint8_t status_registers[4];
			float statistic_registers[2];
        };
		typedef struct {
			int8_t x, y;
			float data;
		} data_t;

		// Mmap variables.
		const size_t cell_size, page_size, mmap_stride;	
        uint8_t * mmap_base_addr;
        uint8_t * mmap_free_addr;

		// Grid typedefs
		typedef std::shared_ptr<Cell>		cell_t;
		typedef std::deque<cell_t>      	column_t;
		typedef std::deque<column_t>    	row_t;

		std::map<std::string, std::list<data_t>> fresh_nodes;

		// Grid variables
		std::deque<column_t> grid;
		uint8_t x_size, x_center, y_size, y_center;

		// Grid constructor helper functions.
		uint8_t * initialize();
		void populate(uint8_t);

		// Grid mutators
		/**
		*  Grid::createCell()
		*  ------------------
		*  Cell() factory function. Call this instead of Cell()
		*    or Derived class constructors to create new Cells
		*    in the Grid.
		**/
		std::shared_ptr<Cell> createCell();
		void addColumns(int8_t); 
		void addRows(int8_t);

	public:
		Grid();
		Grid(uint8_t);
		~Grid();
		
		// Algorithm functions
		void convolve(int8_t, int8_t, operation_t, uint8_t);

		// Convenience functions
		void addTopic(const std::string&);
		void enqueue(int8_t, int8_t, float, uint8_t, const std::string&);

		void checkGridBounds(int8_t, int8_t);
		Cell * getCell(int8_t, int8_t);
	};
	
	

	// Message typedefs
	typedef ccny_srvs::GetAddress::Request		get_address_request_t;
	typedef ccny_srvs::GetAddress::Response		get_address_response_t;

	typedef ccny_srvs::GetStatistic::Request   	get_statistic_request_t;
	typedef ccny_srvs::GetStatistic::Response  	get_statistic_response_t;
	typedef ccny_srvs::SetStatistic::Request   	set_statistic_request_t;
	typedef ccny_srvs::SetStatistic::Response	set_statistic_response_t;

	typedef ccny_srvs::GetStatus::Request		get_status_request_t;
	typedef ccny_srvs::GetStatus::Response		get_status_response_t;
	typedef ccny_srvs::SetStatus::Request		set_status_request_t;
	typedef ccny_srvs::SetStatus::Response		set_status_response_t;

	typedef ros::TimerEvent						timer_event_t;

	ros::NodeHandle node;
	
	// Cell accessors & mutators
	bool getAddress(get_address_request_t&, get_address_response_t&);

	bool getStatistic(get_statistic_request_t&, get_statistic_response_t&, uint8_t);
	bool setStatistic(set_statistic_request_t&, set_statistic_response_t&, uint8_t, const std::string&);

	bool getStatus(get_status_request_t&, get_status_response_t&, uint8_t, uint8_t);
	bool getStatus(get_status_request_t&, get_status_response_t&, uint8_t, uint8_t, uint8_t);
	bool setStatus(set_status_request_t&, set_status_response_t&, uint8_t, uint8_t, uint8_t);

	//bool performDataOperation(const timer_event_t&, operation_t, uint8_t, const std::string&);

	Grid grid;

public:
	GridManager();
	GridManager(uint8_t);
	~GridManager();
	
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
	ros::ServiceServer addressGetter();

	ros::ServiceServer registerStatisticGetter(const std::string&, uint8_t);
	ros::ServiceServer registerStatisticSetter(const std::string&, uint8_t);

	ros::ServiceServer registerStatusGetter(const std::string&, uint8_t, uint8_t);
	ros::ServiceServer registerStatusGetter(const std::string&, uint8_t, uint8_t, uint8_t);
	ros::ServiceServer registerStatusSetter(const std::string&, uint8_t, uint8_t, uint8_t);

	//ros::Timer registerGridOperation(float, operation_t, const std::string&, uint8_t);

	static uint8_t bit(uint8_t);
	static int8_t to_coordinate(float);
	static float to_float(int8_t);
};

#endif