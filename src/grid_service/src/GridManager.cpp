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

ros::ServiceServer GridManager::registerStatisticGetter(const std::string &name, uint8_t stride) {
	return node.advertiseService<get_statistic_request_t, get_statistic_response_t>(
		name,
		boost::bind(&GridManager::getStatistic, this, _1, _2, stride)
	);
}

ros::ServiceServer GridManager::registerStatisticSetter(const std::string &name, uint8_t stride) {
	return node.advertiseService<set_statistic_request_t, set_statistic_response_t>(
		name,
		boost::bind(&GridManager::setStatistic, this, _1, _2, stride)
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

uint8_t GridManager::bit(uint8_t bit) {
	return (1 << bit);
}

/* Private Methods */
bool GridManager::getAddress(get_address_request_t& req, get_address_response_t& res) {
	res.address = reinterpret_cast<uint64_t>(&(*grid.getCell(req.x, req.y)));

	return true;
}

bool GridManager::getStatistic(get_statistic_request_t& req, get_statistic_response_t& res, uint8_t stride) {
	grid.checkGridBounds(req.x, req.y);
    res.data = grid.getCell(req.x, req.y)->getCellStatistic(stride);
	
	ROS_DEBUG(
		"[grid_service] Statistic Register of Cell (%d, %d) at address [%p] contains: [%f]",
		req.x,
		req.y,
		grid.getCell(req.x, req.y)->getCellAddress(),
		res.data
	);
	
	return true;
}

bool GridManager::setStatistic(set_statistic_request_t& req, set_statistic_response_t& res, uint8_t stride) {
	grid.checkGridBounds(req.x, req.y);

	grid.getCell(req.x, req.y)->setCellStatistic(stride, req.data);
	
	ROS_DEBUG(
		"[grid_service] Statistic Register of Cell (%d, %d) at address [%p] is now set to: [%f]",
		req.x,
		req.y,
		grid.getCell(req.x, req.y)->getCellAddress(),
		req.data
	);
	
	return true;
}

bool GridManager::getStatus(get_status_request_t& req, get_status_response_t& res, uint8_t stride, uint8_t mask) {
	grid.checkGridBounds(req.x, req.y);
        res.data = true && (grid.getCell(req.x, req.y)->getCellStatus(stride) & mask);
	
	//ROS_DEBUG(
	ROS_INFO(
		"[grid_service] Status Register of Cell (%d, %d) at address [%p] contains: [%u]",
		req.x,
		req.y,
		grid.getCell(req.x, req.y)->getCellAddress(),
		res.data
	);
	
	return true;
}

bool GridManager::getStatus(get_status_request_t& req, get_status_response_t& res, uint8_t stride, uint8_t offset, uint8_t mask) {
	grid.checkGridBounds(req.x, req.y);
	res.data = (grid.getCell(req.x, req.y)->getCellStatus(stride) & mask) >> offset;
	
	//ROS_DEBUG(
	ROS_INFO(
		"[grid_service] Status Register of Cell (%d, %d) at address [%p] contains: [%u]",
		req.x,
		req.y,
		grid.getCell(req.x, req.y)->getCellAddress(),
		res.data
	);
	
	return true;
}

bool GridManager::setStatus(set_status_request_t& req, set_status_response_t& res, uint8_t stride, uint8_t offset, uint8_t mask) {
	grid.checkGridBounds(req.x, req.y);
	
	uint8_t status_data = grid.getCell(req.x, req.y)->getCellStatus(stride);
	if(req.data != (status_data & mask) >> offset)
		grid.getCell(req.x, req.y)->setCellStatus(stride, 1 << offset);
	
	//ROS_DEBUG(
	ROS_INFO(
		"[grid_service] Status Register of Cell (%d, %d) at address [%p] is now set to: [%u]",
		req.x,
		req.y,
		grid.getCell(req.x, req.y)->getCellAddress(),
		grid.getCell(req.x, req.y)->getCellStatus(stride)
	);
	
	return true;
}