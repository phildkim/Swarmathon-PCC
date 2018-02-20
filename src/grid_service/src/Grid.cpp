#include <algorithm>
#include <iterator>

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
	grid.resize(size + 1);
	std::for_each(grid.begin(), grid.end(), [size] (column_t& column) {
		// Resize Columns and generate Cells.
		column.resize(size + 1);
		std::generate(column.begin(), column.end(), []() {			
			return std::unique_ptr<MappedCell>{ new MappedCell() };
		});
	});
	
	ROS_INFO(
		"[grid_service] Grid initialized with [%u] Rows and [%u] Columns with center at (%d, %d)",
		x_size,
		y_size,
		x_center,
		y_center
	);
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
std::unique_ptr<Cell> Grid::createCell() {
	if(MappedCell::isMapFull()) {
		ROS_DEBUG("[grid_service] Allocating Cell on Heap");
		return std::unique_ptr<AllocatedCell>{ new AllocatedCell() };
	}
	else {
		ROS_DEBUG("[grid_service] Allocating Cell on Memory Map");
		return std::unique_ptr<MappedCell>{ new MappedCell() };
	}
}

void Grid::addColumns(int8_t x) {
	if(	(int8_t) (x_size - x_center) < x) {	
		// Calculate difference & expand grid size.
		uint8_t diff = x - (x_size - x_center);
		x_size += diff;
		
		ROS_WARN("[grid_service] WARNING: Grid too small, adding [%d] Column(s) to the right of grid", diff);

		// Insert Column vector(s) to the right.
		std::generate_n(std::back_inserter(grid), diff, [this]() {
			// Resize and fill Column vector(s).
			column_t col;
			col.resize(grid[0].size());
			std::generate(col.begin(), col.end(), [this]() { return this->createCell(); });	
			return col;
		});
	}
	// Insert Column vector to the left and shift grid center to the right.
	else if((int8_t) -x_center > x) {
		// Calculate difference, expand grid size, & move grid center.
		uint8_t diff = -x_center - x;
		x_size += diff;
		x_center += diff;
		
		ROS_WARN("[grid_service] WARNING: Grid too small, adding [%d] Column(s) to the left of grid", diff);
		
		// Insert Column vector(s) to the left.
		std::generate_n(std::front_inserter(grid), diff, [this]() {
			// Resize and fill Column vector(s)
			column_t col;
			col.resize(grid[0].size());
			std::generate(col.begin(), col.end(), [this]() { return this->createCell(); });
			return col;
		});
	}
}

void Grid::addRows(int8_t y) {
	// Insert Row at bottom by appending a Cell at the back of each Column vector.
	if(	(int8_t) (y_size - y_center) < y) {
		// Calculate difference & expand grid size.
		uint8_t diff = y - (y_size - y_center);
		y_size += diff;
		
		ROS_WARN("[grid_service] WARNING: Grid too small, adding [%d] Row(s) to bottom of grid", diff);
		
		std::for_each(grid.begin(), grid.end(), [this, diff](column_t& col) {
			std::generate_n(std::back_inserter(col), diff, [this]() { return this->createCell(); });
		});
	}
	// Insert Row at top by appending a Cell at the front of each Column vector.
	else if((int8_t) -y_center > y) {
		uint8_t diff = -y_center - y;
		y_size += diff;
		y_center += diff;
		
		ROS_WARN(
			"[grid_service] WARNING: Grid too small, adding [%d] Row(s) to top of grid", diff);
		
		std::for_each(grid.begin(), grid.end(), [this, diff](column_t& col) {
			std::generate_n(std::front_inserter(col), diff, [this]() { return this->createCell(); });
		});
	}
}

bool Grid::getStatus(get_status_request_t& req, get_status_response_t& res, uint8_t stride, uint8_t mask) {
	checkGridBounds(req.x, req.y);
        res.data = (bool) getCell(req.x, req.y)->getCellStatus(stride) & mask;
	
	ROS_DEBUG(
		"[grid_service] Status Register of Cell (%d, %d) at address [%p] contains: [%u]",
		req.x,
		req.y,
		getCell(req.x, req.y)->getCellAddress(),
		res.data
	);
	
	return true;
}

bool Grid::getStatus(get_status_request_t& req, get_status_response_t& res, uint8_t stride, uint8_t offset, uint8_t mask) {
	checkGridBounds(req.x, req.y);
	res.data = (getCell(req.x, req.y)->getCellStatus(stride) & mask) >> offset;
	
	ROS_DEBUG(
		"[grid_service] Status Register of Cell (%d, %d) at address [%p] contains: [%u]",
		req.x,
		req.y,
		getCell(req.x, req.y)->getCellAddress(),
		res.data
	);
	
	return true;
}

bool Grid::setStatus(set_status_request_t& req, set_status_response_t& res, uint8_t stride, uint8_t offset, uint8_t mask) {
	checkGridBounds(req.x, req.y);
	uint8_t status_data = (getCell(req.x, req.y)->getCellStatus(stride) & mask) >> offset;
	if(req.data != status_data)
		getCell(req.x, req.y)->setCellStatus(stride, status_data ^ (1 << offset));
	
	ROS_DEBUG(
		"[grid_service] Status Register of Cell (%d, %d) at address [%p] is now set to: [%u]",
		req.x,
		req.y,
		getCell(req.x, req.y)->getCellAddress(),
		status_data ^ (1 << offset)
	);
	
	return true;
}

Cell * Grid::getCell(int8_t x, int8_t y) {
	return grid.at((int8_t) x_center + x).at((int8_t) y_center + y).get();
}

void Grid::checkGridBounds(int8_t x, int8_t y) {
	ROS_DEBUG(
		"[grid_service] Grid bounds currently are (%d, %d) (top-left) and (%d, %d) (bottom-right)",
		0 - x_center,
		0 - y_center,
		x_size - x_center,
		y_size - y_center
	);

	ROS_DEBUG("[grid_service] Attemping to access cell (%d, %d)...", x, y);	

	if(((int8_t) (y_size - y_center) < y) || ((int8_t) (0 - y_center) > y))
		addRows(y);

	if(((int8_t) (x_size - x_center) < x) || ((int8_t) (0 - x_center) > x))
		addColumns(x);
}
