#include <cstdint>
#include <functional>
#include <iostream>

#include <unistd.h>
#include <sys/types.h>
#include <sys/mman.h>

#include "ros/ros.h"
#include "Cell.h"
#include "Grid.h"

int main(int argc, char **argv) {
	MappedCell::initializeMap();
	ros::init(argc, argv, "grid_mmap");
	
	Grid grid;
	ros::ServiceServer is_occupied_any =            grid.registerStatusGetter("is_occupied_any",		0, 1 | (1 << 1) | (1 << 2));
        ros::ServiceServer is_occupied_obstacle =       grid.registerStatusGetter("is_occupied_obstacle",	0, 1            );
        ros::ServiceServer is_occupied_rover =          grid.registerStatusGetter("is_occupied_rover",		0, 1 << 1       );
        ros::ServiceServer is_occupied_planned_route =  grid.registerStatusGetter("is_occupied_planned_route",	0, 1 << 2       );
        ros::ServiceServer has_samples =                grid.registerStatusGetter("has_samples",		0, 1 << 3       );
        ros::ServiceServer has_been_searched =          grid.registerStatusGetter("has_been_searched",		0, 1 << 4       );

        ros::ServiceServer has_recently_updated =       grid.registerStatusGetter("has_recently_updated",	1, 1 | (1 << 1) );
        ros::ServiceServer has_new_occupancy_data =     grid.registerStatusGetter("has_new_occupancy_data",	1, 1            );

        ros::ServiceServer get_num_of_samples =         grid.registerStatusGetter("get_num_of_samples",		2, ~0           );

        ros::ServiceServer set_occupied_obstacle =      grid.registerStatusSetter("set_occupied_obstacle",	0, 0, 1         );
        ros::ServiceServer set_occupied_rover =         grid.registerStatusSetter("set_occupied_rover",		0, 1, 1 << 1    );
        ros::ServiceServer set_occupied_planned_route = grid.registerStatusSetter("set_occupied_planned_route",	0, 2, 1 << 2    );
        ros::ServiceServer mark_samples =               grid.registerStatusSetter("mark_samples",		0, 3, 1 << 3    );
        ros::ServiceServer mark_searched =              grid.registerStatusSetter("mark_searched",		0, 4, 1 << 4    );
	
	ros::ServiceServer flag_new_occupancy_data =	grid.registerStatusSetter("flag_new_occupancy_data",	1, 0, 1		);

        ros::ServiceServer update_num_of_samples =      grid.registerStatusSetter("update_num_of_samples",	2, 0, ~0        );

	ros::spin();

	munmap(base_addr, getpagesize());
	return 0;
}
