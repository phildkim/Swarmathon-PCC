#include <cmath>
#include <sys/types.h>
#include <sys/mman.h>
#include <new>
#include <unistd.h>

#include "MappedCell.h"

// Static Class Variables
const size_t MappedCell::cell_size = sizeof(uint8_t) * 3;
const size_t MappedCell::page_size = std::ceil(150 * 150 * cell_size/(float) getpagesize()) * getpagesize();
const size_t MappedCell::mmap_stride = MappedCell::page_size/MappedCell::cell_size;

uint8_t * MappedCell::mmap_base_addr = (uint8_t *) mmap(
	NULL,
	MappedCell::page_size,
	PROT_READ | PROT_WRITE,
	MAP_ANONYMOUS | MAP_PRIVATE,
	-1,
	0
);
uint8_t * MappedCell::mmap_free_addr = MappedCell::mmap_base_addr;

const uint8_t MappedCell::STATUS_REGISTER_OFFSET = 0;
const uint8_t MappedCell::STATISTIC_REGISTER_OFFSET = 3;

// Public Member Functions
MappedCell::MappedCell() : 
	cell_base_addr(MappedCell::mmap_free_addr)
{
	MappedCell::mmap_free_addr += MappedCell::cell_size;
}

MappedCell::~MappedCell() {

}

void MappedCell::initializeMap() {
        if(MappedCell::mmap_base_addr == (uint8_t *) -1)
                throw std::bad_alloc();
}

void MappedCell::cleanupMap() {
	munmap(MappedCell::mmap_base_addr, MappedCell::page_size);
}

bool MappedCell::isMapFull() {
        return (MappedCell::mmap_free_addr - MappedCell::mmap_base_addr)/MappedCell::cell_size > MappedCell::mmap_stride;
}

uint8_t * const MappedCell::getCellAddress() {
	return this->cell_base_addr;
}


uint8_t MappedCell::getCellStatus(uint8_t stride) const {
	return *(this->cell_base_addr + MappedCell::STATUS_REGISTER_OFFSET + stride);
}

void MappedCell::setCellStatus(uint8_t stride, uint8_t data) {
	*(this->cell_base_addr + MappedCell::STATUS_REGISTER_OFFSET + stride) = data;	
}
