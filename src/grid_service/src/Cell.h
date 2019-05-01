#ifndef CELL_H
#define CELL_H
#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>

class Cell {
private:
	std::weak_ptr<Cell> most_valuable_neighbor;
public:
	virtual ~Cell() {};
	// Abstract methods
	virtual uint8_t * const getCellAddress() = 0;
	virtual float getCellStatistic(uint8_t) const = 0;
	virtual void setCellStatistic(uint8_t, float) = 0;
	virtual uint8_t getCellStatus(uint8_t) const = 0;
	virtual void setCellStatus(uint8_t, uint8_t) = 0;
	// Implemented methods
	Cell * getBestNeighbor();
	void setBestNeighbor(std::shared_ptr<Cell>, uint8_t, uint8_t);
};

#endif
