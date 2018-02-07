/** 
 *  @file    AllocatedCell.h
 *  @author  Eric Tung (Treayn)
 *  @date    2/06/2017
 *  @version 1.0 
 *  
 *  @brief Derived class of cell
 *
 *  @section DESCRIPTION
 *  
 *  Allocates additional cells after the grid has
 *    been initialized.
 *  
 *  When the memory-mapped region governed by MappedCell
 *    is full, Grid.h starts allocating AllocatedCells
 *    on the heap.
 *  
 *  This allows the grid to expand to a full size of
 *    220-by-220 as needed.
**/

#ifndef ALLOCATEDCELL_H
#define ALLOCATEDCELL_H

#include <array>

#include "Cell.h"

class AllocatedCell : public Cell {
private:
        std::array<uint8_t, 3> status_registers;

public:
        AllocatedCell();
        ~AllocatedCell();

        AllocatedCell * const getCellAddress() override;

        uint8_t getCellStatus(uint8_t) const override;
        void setCellStatus(uint8_t, uint8_t) override;
};

#endif

