#include <algorithm>

#include "Grid.h"
#include "AllocatedCell.cpp"
#include "MappedCell.cpp"

/* Public Methods */
Grid::Grid() : Grid(150)
{ /* Delegating Constructor */ }

Grid::Grid(uint8_t size) :
	x_size(size),
	y_size(size),
	x_center(size/2),
	y_center(size/2)
{
	// Resize Rows.
	grid.resize(size);
	std::for_each(grid.begin(), grid.end(), [size] (column_t& column) {
		// Resize Columns and generate Cells.
		column.resize(size);
		std::generate(column.begin(), column.end(), []() {
			return std::unique_ptr<MappedCell>{ new MappedCell() };
		});
	});
}

Grid::~Grid() {
	// Grid contains either unique_ptr-managed objects or stack-allocated
	//   variables. Destructor is left empty as a result.
}

ros::ServiceServer Grid::registerStatusGetter(const std::string &name, uint8_t stride, uint8_t mask) {
        return node.advertiseService<get_status_request_t, get_status_response_t>(
		name,
		boost::bind(&Grid::getStatus, this, _1, _2, stride, mask)
	);
}

ros::ServiceServer Grid::registerStatusGetter(const std::string &name, uint8_t stride, uint8_t offset, uint8_t mask) {
        return node.advertiseService<get_status_request_t, get_status_response_t>(
		name,
		boost::bind(&Grid::getStatus, this, _1, _2, stride, offset, mask)
	);
}

ros::ServiceServer Grid::registerStatusSetter(const std::string &name, uint8_t stride, uint8_t offset, uint8_t mask) {
        return node.advertiseService<set_status_request_t, set_status_response_t>(
		name,
		boost::bind(&Grid::setStatus, this, _1, _2, stride, offset, mask)
	);
}

/* Private Methods */

// Grid::CreateCell()
// ------------------
// Cell() factory function. Call this instead of Cell() or derived class constructors
//	to create new Cells in the grid.
std::unique_ptr<Cell> Grid::createCell() {
	if(MappedCell::isMapFull())
		return std::unique_ptr<AllocatedCell>{ new AllocatedCell() };
	else
		return std::unique_ptr<MappedCell>{ new MappedCell() };
}

void Grid::addColumn(int8_t x) {
	x_size++; // Expand overall grid size.
	
	// Insert Column vector to the right and resize it.
	if(	(int8_t) (grid.size() - x_center) < x) {
		grid.push_back(column_t());
		grid.back().resize(y_size);
				
		// Fill Column vector with cells.
		std::generate(grid.back().begin(), grid.back().end(), [this]() {
			return this->createCell();
		});
	}
	// Insert Column vector to the left and shift grid center to the right.
	else if((int8_t) (x_center - grid.size()) > x) {
		x_center++;
		grid.push_front(column_t());
		grid.front().resize(y_size);
		
		std::generate(grid.front().begin(), grid.front().end(), [this]() {
			return this->createCell();
		});
	}
	else {
		x_size--;
	}
}

void Grid::addRow(int8_t y) {
	y_size++;
	
	// Insert Row at bottom by appending a Cell at the back of each Column vector.
	if(	(int8_t) (grid[0].size() - y_center) < y) {
		std::for_each(grid.begin(), grid.end(), [this](column_t& col) { col.push_back(this->createCell()); });
	}
	// Insert Row at top by appending a Cell at the front of each Column vector.
	else if((int8_t) (y_center - grid[0].size()) > y) {
		y_center++;
		std::for_each(grid.begin(), grid.end(), [this](column_t& col) { col.push_front(this->createCell()); });
	}
	else {
		y_size--;
	}
}

bool Grid::getStatus(get_status_request_t& req, get_status_response_t& res, uint8_t stride, uint8_t mask) {
        res.data = (bool) getCell(req.x, req.y)->getCellStatus(stride) & mask;
	
	ROS_INFO(
		"[grid_service] Status Register at [%p] contains: [%u]",
		getCell(req.x, req.y)->getCellAddress(),
		res.data
	);
	
	return true;
}

bool Grid::getStatus(get_status_request_t& req, get_status_response_t& res, uint8_t stride, uint8_t offset, uint8_t mask) {
	res.data = (getCell(req.x, req.y)->getCellStatus(stride) & mask) >> offset;
	
	ROS_INFO(
		"[grid_service] Status Register at [%p] contains: [%u]",
		getCell(req.x, req.y)->getCellAddress(),
		res.data
	);
	
	return true;
}

bool Grid::setStatus(set_status_request_t& req, set_status_response_t& res, uint8_t stride, uint8_t offset, uint8_t mask) {
	uint8_t status_data = (getCell(req.x, req.y)->getCellStatus(stride) & mask) >> offset;
	if(req.data != status_data)
		getCell(req.x, req.y)->setCellStatus(stride, status_data ^ (1 << offset));
	
	ROS_INFO(
		"[grid_service] Status Register at [%p] is now set to: [%u]",
		getCell(req.x, req.y)->getCellAddress(),
		status_data ^ (1 << offset)
	);
	
	return true;
}

Cell * Grid::getCell(int8_t x, int8_t y) {
	return grid.at((int8_t) x_center + x).at((int8_t) y_center + y).get();
}
