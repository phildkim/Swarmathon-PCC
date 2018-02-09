/** 
 *  @file    Cell.h
 *  @author  Eric Tung (Treayn)
 *  @date    2/06/2017
 *  @version 1.0 
 *  
 *  @brief Cell (Abstract Class)
 *
 *  @section DESCRIPTION
 *  
 *  Abstract Class - Do not attempt to instantiate
 *    directly.
 *  
 *  Do not attempt to instantiate any Derived classes
 *    directly either, use Grid::createCell() factory
 *    function to perform Cell allocations as neeed.
 *  
 *  Grid.h contains a 150-by-150 2D deque and allocates
 *    new Cells using the MappedCell() and AllocatedCell()
 *    constructors as needed.
**/

#ifndef CELL_H
#define CELL_H

#include <cstddef>
#include <cstdint>

class Cell {
public:
	virtual ~Cell() {};
	
	virtual Cell * const getCellAddress() = 0;
	
	virtual uint8_t getCellStatus(uint8_t) const = 0;
	virtual void setCellStatus(uint8_t, uint8_t) = 0;
};

#endif
