#include <cstdint>
#include <functional>
#include <iostream>

#include <unistd.h>
#include <sys/types.h>
#include <sys/mman.h>

#include "ros/ros.h"
#include "GridManager.h"
#include "GridManager.cpp"

int main(int argc, char **argv) {
	ros::init(argc, argv, "grid_service");
	
	GridManager grid_manager;

        ros::ServiceServer get_cell_address =           grid_manager.addressGetter();
        ros::ServiceServer get_most_valuable_point =    grid_manager.MVPGetter();

        // Statistics
        ros::ServiceServer get_traversal_cost =         grid_manager.registerStatisticGetter("get_traversal_cost",              0);
        ros::ServiceServer get_desirability_index =     grid_manager.registerStatisticGetter("get_desirability_index",          1);

        ros::ServiceServer update_traversal_cost =      grid_manager.registerStatisticSetter("update_traversal_cost",           0);
        ros::ServiceServer update_desirability_index =  grid_manager.registerStatisticSetter("update_desirability_index",       1);
        
        // Status
	ros::ServiceServer is_occupied_any =            grid_manager.registerStatusGetter("is_occupied_any",		        0, 1 | (1 << 1) | (1 << 2));
        ros::ServiceServer is_occupied_obstacle =       grid_manager.registerStatusGetter("is_occupied_obstacle",	        0, GridManager::bit(0));
        ros::ServiceServer is_occupied_rover =          grid_manager.registerStatusGetter("is_occupied_rover",		        0, GridManager::bit(1));
        ros::ServiceServer is_occupied_planned_route =  grid_manager.registerStatusGetter("is_occupied_planned_route",	        0, GridManager::bit(2));
        ros::ServiceServer has_samples =                grid_manager.registerStatusGetter("has_samples",		        0, GridManager::bit(3));
        ros::ServiceServer has_been_searched =          grid_manager.registerStatusGetter("has_been_searched",		        0, GridManager::bit(4));

        ros::ServiceServer has_recently_updated =       grid_manager.registerStatusGetter("has_recently_updated",	        1, 1 | (1 << 1) );
        ros::ServiceServer has_new_occupancy_data =     grid_manager.registerStatusGetter("has_new_occupancy_data",     	1, GridManager::bit(1));

        ros::ServiceServer get_num_of_samples =         grid_manager.registerStatusGetter("get_num_of_samples",		        2, ~0           );

        ros::ServiceServer set_occupied_obstacle =      grid_manager.registerStatusSetter("set_occupied_obstacle",	        0, 0, GridManager::bit(0));
        ros::ServiceServer set_occupied_rover =         grid_manager.registerStatusSetter("set_occupied_rover",		        0, 1, GridManager::bit(1));
        ros::ServiceServer set_occupied_planned_route = grid_manager.registerStatusSetter("set_occupied_planned_route",	        0, 2, GridManager::bit(2));
        ros::ServiceServer mark_samples =               grid_manager.registerStatusSetter("mark_samples",		        0, 3, GridManager::bit(3));
        ros::ServiceServer mark_searched =              grid_manager.registerStatusSetter("mark_searched",		        0, 4, GridManager::bit(4));
	
	ros::ServiceServer flag_new_occupancy_data =	grid_manager.registerStatusSetter("flag_new_occupancy_data",	        1, 0, GridManager::bit(1));

        ros::ServiceServer update_num_of_samples =      grid_manager.registerStatusSetter("update_num_of_samples",	        2, 0, ~0        );
        
        ros::Timer desirability_decay =                 grid_manager.registerGridOperation(10.0, "update_desirability_index", 1);

	ros::spin();

	return 0;
}
