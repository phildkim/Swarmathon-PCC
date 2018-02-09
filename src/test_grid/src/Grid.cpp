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
	uint8_t x_test = 0;		// For Testing
	
	// Resize Rows.
	grid.resize(size);
	std::for_each(grid.begin(), grid.end(), [size, &x_test] (column_t& column) {
		uint8_t y_test = 0;	// For Testing
		
		// Resize Columns and generate Cells.
		column.resize(size);
		std::generate(column.begin(), column.end(), [&x_test, &y_test]() {
			ROS_INFO(
				"[grid_service] Constructing new MappedCell at (%u, %u) with address [%p]",
				x_test,
				y_test,
				MappedCell::getFreeAddress()
			);
			y_test++;
			
			return std::unique_ptr<MappedCell>{ new MappedCell() };
		});
		
		x_test++;
	});
	
	ROS_INFO("[grid_service] Grid initialized with [%u] Rows and Columns", size);
	ROS_INFO("[grid_service] Size of Cell is [%zu] bytes", sizeof(uint8_t) * 3);
	ROS_INFO("[grid_service] Size of MappedCell is [%zu] bytes", sizeof(MappedCell));
	ROS_INFO("[grid_service] Stride of mmapped region is [%zu] cells", MappedCell::getStride());
	ROS_INFO("[grid_service] Start of mmapped region is [%p]", MappedCell::getBaseAddress());
	ROS_INFO("[grid_service] End of mmapped region is [%p]", MappedCell::getBaseAddress() + MappedCell::getStride() * MappedCell::getCellSize());
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
		ROS_INFO("[grid_service] Allocating Cell on Heap");
		return std::unique_ptr<AllocatedCell>{ new AllocatedCell() };
	}
	else {
		ROS_INFO("[grid_service] Allocating Cell on Memory Map at address [%p]", MappedCell::getFreeAddress());
		return std::unique_ptr<MappedCell>{ new MappedCell() };
	}
}

void Grid::addColumns(int8_t x) {
	if(	(int8_t) (grid.size() - x_center) < x) {
		// Expand grid size.
		x_size += x - (grid.size() - x_center);
		
		ROS_INFO(
			"[grid_service] Adding [%d] Column(s) to the right of grid",
			x_size - grid.size()
		);

		// Insert Column vector(s) to the right.
		std::generate_n(std::back_inserter(grid), x_size - grid.size(), [this]() {
			// Resize and fill Column vector(s).
			column_t col;
			col.resize(y_size);
			std::generate(col.begin(), col.end(), [this]() { return this->createCell(); });	
			return col;
		});
	}
	// Insert Column vector to the left and shift grid center to the right.
	else if((int8_t) (x_center - grid.size()) > x) {
		// Expand grid size & move grid center.
		x_size += (x_center - grid.size()) - x;
		x_center += x_size - grid.size();
		
		ROS_INFO(
			"[grid_service] Adding [%d] Column(s) to the left of grid",
			x_size - grid.size()
		);
		
		// Insert Column vector(s) to the left.
		std::generate_n(std::front_inserter(grid), x_size - grid.size(), [this]() {
			// Resize and fill Column vector(s)
			column_t col;
			col.resize(y_size);
			std::generate(col.begin(), col.end(), [this]() { return this->createCell(); });
			return col;
		});
	}
}

void Grid::addRows(int8_t y) {
	// Insert Row at bottom by appending a Cell at the back of each Column vector.
	if(	(int8_t) (grid[0].size() - y_center) < y) {
		// Expand grid size.
		y_size += y - (grid[0].size() - y_center);
		
		ROS_INFO(
			"[grid_service] Adding [%d] Row(s) to bottom of grid",
			y_size - grid[0].size()
		);
		
		std::for_each(grid.begin(), grid.end(), [this](column_t& col) {
			std::generate_n(std::back_inserter(col), y_size - grid[0].size(), [this]() { return this->createCell(); });
		});
	}
	// Insert Row at top by appending a Cell at the front of each Column vector.
	else if((int8_t) (y_center - grid[0].size()) > y) {
		y_size += (y_center - grid[0].size()) - y;
		y_center += y_size - grid[0].size();
		
		ROS_INFO(
			"[grid_service] Adding [%d] Row(s) to top of grid",
			y_size - grid[0].size()
		);
		
		std::for_each(grid.begin(), grid.end(), [this](column_t& col) {
			std::generate_n(std::front_inserter(col), y_size - grid[0].size(), [this]() { return this->createCell(); });
		});
	}
}

bool Grid::getStatus(get_status_request_t& req, get_status_response_t& res, uint8_t stride, uint8_t mask) {
	checkGridBounds(req.x, req.y);
        res.data = (bool) getCell(req.x, req.y)->getCellStatus(stride) & mask;
	
	ROS_INFO(
		"[grid_service] Status Register of Cell (%d, %d) at address [%p] contains: [%u]",
		req.x + x_center,
		req.y + y_center,
		getCell(req.x, req.y)->getCellAddress(),
		res.data
	);
	
	return true;
}

bool Grid::getStatus(get_status_request_t& req, get_status_response_t& res, uint8_t stride, uint8_t offset, uint8_t mask) {
	checkGridBounds(req.x, req.y);
	res.data = (getCell(req.x, req.y)->getCellStatus(stride) & mask) >> offset;
	
	ROS_INFO(
		"[grid_service] Status Register of Cell (%d, %d) at address [%p] contains: [%u]",
		req.x + x_center,
		req.y + y_center,
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
	
	ROS_INFO(
		"[grid_service] Status Register of Cell (%d, %d) at address [%p] is now set to: [%u]",
		req.x + x_center,
		req.y + y_center,
		getCell(req.x, req.y)->getCellAddress(),
		status_data ^ (1 << offset)
	);
	
	return true;
}

Cell * Grid::getCell(int8_t x, int8_t y) {
	return grid.at((int8_t) x_center + x).at((int8_t) y_center + y).get();
}

void Grid::checkGridBounds(int8_t x, int8_t y) {
	ROS_INFO(
		"[grid_service] Grid bounds currently are (%d, %d) (top-left) and (%d, %d) (bottom-right)",
		x_center - grid.size(),
		y_center - grid[0].size(),
		grid.size() - x_center,
		grid[0].size() - y_center
	);
	ROS_INFO(
		"[grid_service] Attemping to access cell (%d, %d):",
		x,
		y
	);	

	if((grid.size() - x_center < x) || (x_center - grid.size() > x)) {
		ROS_WARN("[grid_service] ERROR: Grid too small, adding Columns");
		addColumns(x);
	}
	if((grid[0].size() - y_center < y) || (y_center - grid[0].size() > y)) {
		ROS_WARN("[grid_service] ERROR: Grid too small, adding Rows");
		addRows(y);
	}
}
