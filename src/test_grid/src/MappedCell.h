/** 
 *  @file    MappedCell.h
 *  @author  Eric Tung (Treayn)
 *  @date    2/06/2017
 *  @version 1.0 
 *  
 *  @brief Derived class of cell
 *
 *  @section DESCRIPTION
 *  
 *  The MappedCell class creates a memory map for the
 *    grid in Grid.h.
 *  Cell allocations for the initial 150-by-150 by Grid.h
 *    take place using MappedCells.
 *  
 *  MappedCell::initializeMap(), a static class member,
 *    must be called before initializing the grid.
 *  This is best done in the main() function of your node.
 *  
 *  The MappedCell class uses static pointers to keep
 *    track of the mmapped space. When the MappedCell()
 *    constructor is called, the static pointer's address
 *    is copied into the instance of the MappedCell.
 *  
 *  Do not attempt to allocate or create MappedCells
 *    directly using the MappedCell() constructor.
 *  Instead, calls to Grid::createCell will ensure that
 *    cells are initialized properly, and that no memory
 *    leaks occur.
**/

#ifndef MAPPEDCELL_H
#define MAPPEDCELL_H

#include <unistd.h>
#include <sys/types.h>
#include <sys/mman.h>

#include "Cell.h"

class MappedCell : public Cell {
private:
        static MappedCell * mmap_base_addr;
        static MappedCell * mmap_free_addr;
        static size_t mmap_stride;
        static uint32_t page_size;

        MappedCell * const cell_base_addr;
        uint8_t * const data_status_registers;

public:
        MappedCell();
        ~MappedCell();

        static void initializeMap();
        static void cleanupMap();
        static bool isMapFull();
	
	// Test functions
	static MappedCell * const getFreeAddress();
        MappedCell * const getCellAddress() override;

        uint8_t getCellStatus(uint8_t) const override;
        void setCellStatus(uint8_t, uint8_t) override;
};

#endif

