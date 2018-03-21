#include <utility>

#include "GridManager.h"
#include "Grid.cpp"

/* Public Methods */
GridManager::GridManager() : GridManager(150)
{ /* Delegating Constructor */ }

GridManager::GridManager(uint8_t size) :
	grid(size)
{ }

GridManager::~GridManager() {
	// Grid contains either unique_ptr-managed objects or stack-allocated
	//   variables. Destructor is left empty as a result.
}

ros::ServiceServer GridManager::addressGetter() {
	return node.advertiseService("get_cell_address", &GridManager::getAddress, this);
}

ros::ServiceServer GridManager::MVPGetter() {
	return node.advertiseService("most_valuable_point", &GridManager::getMostValuablePoint, this);
}

ros::ServiceServer GridManager::registerStatisticGetter(const std::string &name, uint8_t stride) {
	return node.advertiseService<get_statistic_request_t, get_statistic_response_t>(
		name,
		boost::bind(&GridManager::getStatistic, this, _1, _2, stride)
	);
}

ros::ServiceServer GridManager::registerStatisticSetter(const std::string &name, uint8_t stride) {
	grid.addTopic(name);

	return node.advertiseService<set_statistic_request_t, set_statistic_response_t>(
		name,
		boost::bind(&GridManager::setStatistic, this, _1, _2, stride, name)
	);
}

ros::ServiceServer GridManager::registerStatusGetter(const std::string &name, uint8_t stride, uint8_t mask) {
    return node.advertiseService<get_status_request_t, get_status_response_t>(
		name,
		boost::bind(&GridManager::getStatus, this, _1, _2, stride, mask)
	);
}

ros::ServiceServer GridManager::registerStatusGetter(const std::string &name, uint8_t stride, uint8_t offset, uint8_t mask) {
    return node.advertiseService<get_status_request_t, get_status_response_t>(
		name,
		boost::bind(&GridManager::getStatus, this, _1, _2, stride, offset, mask)
	);
}

ros::ServiceServer GridManager::registerStatusSetter(const std::string &name, uint8_t stride, uint8_t offset, uint8_t mask) {
    return node.advertiseService<set_status_request_t, set_status_response_t>(
		name,
		boost::bind(&GridManager::setStatus, this, _1, _2, stride, offset, mask)
	);
}


ros::Timer GridManager::registerGridOperation(float duration, const std::string &name, uint8_t stride) {
	return node.createTimer(
		ros::Duration(duration),
		[this, name, stride] (const timer_event_t& event) { 
			this->grid.update(name, stride);
			ROS_INFO("[grid_service] Updated active nodes");
		}
	);
}

/* Static Methods */
uint8_t GridManager::bit(uint8_t bit) {
	return (1 << bit);
}

int8_t GridManager::to_coordinate(float input) {
	return (int8_t) std::round(input * 10.0);
}

float GridManager::to_float(int8_t input) {
	return input;
}

/* Private Methods */
bool GridManager::getAddress(get_address_request_t& req, get_address_response_t& res) {
	res.address = reinterpret_cast<uint64_t>(&(*grid.getCell(req.x, req.y)));

	return true;
}

bool GridManager::getMostValuablePoint(mvpoint_request_t& req, mvpoint_response_t& res) {
	res.x = GridManager::to_float(grid.getMVP_X())/10.0;
	res.y = GridManager::to_float(grid.getMVP_Y())/10.0;
	res.exists = !((res.x == 0.0) && (res.y == 0.0));

	ROS_INFO(
		"[grid_service] Most valuable point at (%f, %f)",
		res.x,
		res.y
	);

	return true;
}

bool GridManager::getStatistic(get_statistic_request_t& req, get_statistic_response_t& res, uint8_t stride) {
	int8_t x = GridManager::to_coordinate(req.x);
	int8_t y = GridManager::to_coordinate(req.y);
	grid.checkGridBounds(x, y);

    res.data = grid.getCell(x, y)->getCellStatistic(stride);
	
	ROS_DEBUG(
		"[grid_service] Statistic Register of Cell (%d, %d) at address [%p] contains: [%f]",
		x,
		y,
		grid.getCell(x, y)->getCellAddress(),
		res.data
	);
	
	return true;
}

bool GridManager::setStatistic(set_statistic_request_t& req, set_statistic_response_t& res, uint8_t stride, const std::string& topic) {
	int8_t x = GridManager::to_coordinate(req.x);
	int8_t y = GridManager::to_coordinate(req.y);
	grid.checkGridBounds(x, y);

	grid.enqueue(x, y, req.data, stride, topic);

	ROS_INFO(
		"[grid_service] Put coordinates (%d, %d) and statistical data [%f] on queue [%s] for processing",
		x,
		y,
		req.data,
		topic.c_str()
	);

	return true;
}

bool GridManager::getStatus(get_status_request_t& req, get_status_response_t& res, uint8_t stride, uint8_t mask) {
	int8_t x = GridManager::to_coordinate(req.x);
	int8_t y = GridManager::to_coordinate(req.y);
	grid.checkGridBounds(x, y);
    
	res.data = true && (grid.getCell(x, y)->getCellStatus(stride) & mask);
	
	//ROS_DEBUG(
	ROS_INFO(
		"[grid_service] Status Register of Cell (%d, %d) at address [%p] contains: [%u]",
		x,
		y,
		grid.getCell(x, y)->getCellAddress(),
		res.data
	);
	
	return true;
}

bool GridManager::getStatus(get_status_request_t& req, get_status_response_t& res, uint8_t stride, uint8_t offset, uint8_t mask) {
	int8_t x = GridManager::to_coordinate(req.x);
	int8_t y = GridManager::to_coordinate(req.y);
	grid.checkGridBounds(x, y);

	res.data = (grid.getCell(x, y)->getCellStatus(stride) & mask) >> offset;
	
	//ROS_DEBUG(
	ROS_INFO(
		"[grid_service] Status Register of Cell (%d, %d) at address [%p] contains: [%u]",
		x,
		y,
		grid.getCell(x, y)->getCellAddress(),
		res.data
	);
	
	return true;
}

bool GridManager::setStatus(set_status_request_t& req, set_status_response_t& res, uint8_t stride, uint8_t offset, uint8_t mask) {
	int8_t x = GridManager::to_coordinate(req.x);
	int8_t y = GridManager::to_coordinate(req.y);
	grid.checkGridBounds(x, y);
	
	uint8_t status_data = grid.getCell(x, y)->getCellStatus(stride);
	if(req.data != (status_data & mask) >> offset)
		grid.getCell(x, y)->setCellStatus(stride, 1 << offset);
	
	//ROS_DEBUG(
	ROS_INFO(
		"[grid_service] Status Register of Cell (%d, %d) at address [%p] is now set to: [%u]",
		x,
		y,
		grid.getCell(x, y)->getCellAddress(),
		grid.getCell(x, y)->getCellStatus(stride)
	);
	
	return true;
}

void GridManager::performDataOperation(const timer_event_t& event, uint8_t stride, const std::string& name) {
	grid.update(name, stride);
	ROS_DEBUG("[grid_service] Updated active nodes");
}