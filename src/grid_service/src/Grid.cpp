#include <algorithm>
#include <iterator>
#include <sys/types.h>
#include <sys/mman.h>
#include <new>
#include <unistd.h>
#include "AllocatedCell.cpp"
#include "MappedCell.cpp"

GridManager::Grid::Grid(uint8_t size) :
    cell_size(sizeof(GridManager::Grid::Layout)),
    page_size( [size, this]() -> size_t {
        return std::ceil((size + 1) * (size + 1) * this->cell_size/(float) getpagesize()) * getpagesize(); }()
    ),
    mmap_stride(page_size/cell_size),
    mmap_base_addr(initialize()),
    mmap_free_addr(mmap_base_addr),
	x_size(size),
	y_size(size),
	x_center(size/2),
	y_center(size/2)
{	
    populate(size);

    ROS_INFO(
        "[grid_service] Cell size is [%zu] and Page size is [%zu].",
        cell_size,
        page_size
    );

    ROS_INFO(
        "[grid_service] Memory Map region begins at [%p] and ends at [%p]. Last allocation was at address [%p].",
        mmap_base_addr,
        mmap_base_addr + page_size,
        mmap_free_addr
    );
}

GridManager::Grid::~Grid() {
	munmap(mmap_base_addr, page_size);
}

uint8_t * GridManager::Grid::initialize() {
	uint8_t * addr = (uint8_t *) mmap(
        NULL,
        page_size,
        PROT_READ | PROT_WRITE,
        MAP_ANONYMOUS | MAP_PRIVATE,
        -1,
        0
    );

    // Check if mmap region has been initialized successfully.
    if(addr == (uint8_t *) -1)
        throw std::bad_alloc();
    
    return addr;
}

void GridManager::Grid::populate(uint8_t size) {
	// Resize Rows.
	grid.resize(size + 1);
	std::for_each(grid.begin(), grid.end(), [size, this] (column_t& column) {
		// Resize Columns and generate Cells.
		column.resize(size + 1);
		std::generate(column.begin(), column.end(), [this]() {
			return this->createCell();
		});
	});
	
	ROS_INFO(
		"[grid_service] Grid initialized with [%u] Rows and [%u] Columns with center at (%d, %d)",
		x_size, y_size, x_center, y_center
	);
}

std::shared_ptr<Cell> GridManager::Grid::createCell() {
    std::shared_ptr<Cell> ptr;

    // Check if memory-mapped region is full.
	if((mmap_free_addr - mmap_base_addr)/cell_size >= mmap_stride) {
		//ROS_DEBUG("[grid_service] Allocating Cell on Heap");
        ROS_INFO("[grid_service] Allocating Cell on Heap");
        ptr = std::shared_ptr<AllocatedCell>(new AllocatedCell());
	} else {
		//ROS_DEBUG("[grid_service] Allocating Cell on Memory Map");
        ROS_INFO("[grid_service] Allocating Cell on Memory Map at [%p]", mmap_free_addr);
		ptr = std::shared_ptr<MappedCell>(new MappedCell(mmap_free_addr));
        mmap_free_addr += cell_size;
	}

    return ptr;
}

void GridManager::Grid::addColumns(int8_t x) {
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

void GridManager::Grid::addRows(int8_t y) {
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

/* Public functions */
void GridManager::Grid::convolve(int8_t x_base, int8_t y_base, operation_t operation, uint8_t stride) {
	// Apply mathematical function around area
	for(int8_t x = x_base - 2; x <= x_base + 2; x++) {
		for(int8_t y = y_base - 2; y <= y_base + 2; y++) {
			checkGridBounds(x, y);
			float val = operation(x, x_base, y, y_base);

			ROS_INFO(
				"[grid_service] Coordinate is (%d, %d) with value [%f] and stride [%u]",
				x,
				y,
				val,
				stride
			);

			this->getCell(x, y)->setCellStatistic(stride, val);
		}
	}
}

void GridManager::Grid::addTopic(const std::string& topic) {
	fresh_nodes[topic] = std::list<data_t>();
}

void GridManager::Grid::enqueue(int8_t x, int8_t y, float data, uint8_t stride, const std::string& topic) {
	data_t data_point;
	data_point.x = x;
	data_point.y = y;
	data_point.data = data;

	convolve(
		x,
		y,
		[](int8_t x_f, int8_t x_i, int8_t y_f, int8_t y_i) -> float {
			if((x_f == x_i) && (y_f == y_i))
				return 1.0;
			else
				return 1/(2 * std::sqrt(std::pow(x_f - x_i, 2) + std::pow(y_f - y_i, 2)));
		},
		stride
	);

	std::list<data_t>::iterator it = find_if(fresh_nodes[topic].begin(), fresh_nodes[topic].end(), [x, y](const data_t& node) {
		return (node.x == x) && (node.y == y);
	});

	if(it == fresh_nodes[topic].end())
		fresh_nodes[topic].push_back(data_point);
}

int8_t GridManager::Grid::getMVP_X() {
	return fresh_nodes["update_desirability_index"].begin()->x;
}

int8_t GridManager::Grid::getMVP_Y() {
	return fresh_nodes["update_desirability_index"].begin()->y;
}

void GridManager::Grid::update(const std::string& topic, uint8_t stride) {
	std::list<data_t>::iterator it = std::remove_if(fresh_nodes[topic].begin(), fresh_nodes[topic].end(), [stride, this](const data_t& node) {
		this->convolve(
			node.x,
			node.y,
			[stride, this](int8_t x_f, int8_t x_i, int8_t y_f, int8_t y_i) -> float {
				return std::max(0.0, this->getCell(x_f, y_f)->getCellStatistic(stride) - 0.1);
			},
			stride
		);

		// Check if selected cell's desirability value has decayed severely.
		return this->getCell(node.x, node.y)->getCellStatistic(stride) < 0.01;
	});

	fresh_nodes[topic].erase(it, fresh_nodes[topic].end());

	ROS_INFO(
		"[grid_service] Nodes in list: [%u]",
		fresh_nodes[topic].size()
	);

	fresh_nodes[topic].sort([stride, this] (const data_t& l_node, const data_t& r_node) {
		return this->getCell(l_node.x, l_node.y)->getCellStatistic(stride) > this->getCell(r_node.x, r_node.y)->getCellStatistic(stride);
	});
}

void GridManager::Grid::checkGridBounds(int8_t x, int8_t y) {
	ROS_DEBUG(
		"[grid_service] Grid bounds currently are (%d, %d) (top-left) and (%d, %d) (bottom-right)",
		0 - x_center,
		0 - y_center,
		x_size - x_center,
		y_size - y_center
	);

    //ROS_DEBUG("[grid_service] Attemping to access cell (%d, %d)...", x, y);
	ROS_INFO("[grid_service] Attemping to access cell (%d, %d)...", x, y);

	if(((int8_t) (y_size - y_center) < y) || ((int8_t) (0 - y_center) > y))
		addRows(y);

	if(((int8_t) (x_size - x_center) < x) || ((int8_t) (0 - x_center) > x))
		addColumns(x);
}

Cell * GridManager::Grid::getCell(int8_t x, int8_t y) {
	return grid.at((int8_t) x_center + x).at((int8_t) y_center + y).get();
}
