#include <cmath>
#include <sys/types.h>
#include <sys/mman.h>
#include <new>
#include <unistd.h>
#include "MappedCell.h"

// Static Class Variables
const uint8_t MappedCell::STATUS_REGISTER_OFFSET = 0;
const uint8_t MappedCell::STATISTIC_REGISTER_OFFSET = 4;

// Public Member Functions
MappedCell::MappedCell(uint8_t * addr) : 
	cell_base_addr(addr){}

MappedCell::~MappedCell() {}

uint8_t * const MappedCell::getCellAddress() {
	return this->cell_base_addr;
}

float MappedCell::getCellStatistic(uint8_t stride) const {
	return *(((float *) (this->cell_base_addr + MappedCell::STATISTIC_REGISTER_OFFSET)) + stride);
}

void MappedCell::setCellStatistic(uint8_t stride, float data) {
	*(((float *) (this->cell_base_addr + MappedCell::STATISTIC_REGISTER_OFFSET)) + stride) = data;
}

uint8_t MappedCell::getCellStatus(uint8_t stride) const {
	return *(this->cell_base_addr + MappedCell::STATUS_REGISTER_OFFSET + stride);
}

void MappedCell::setCellStatus(uint8_t stride, uint8_t data) {
	*(this->cell_base_addr + MappedCell::STATUS_REGISTER_OFFSET + stride) ^= data;	
}
