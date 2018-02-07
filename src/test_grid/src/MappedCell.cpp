#include <sys/types.h>
#include <sys/mman.h>
#include <new>
#include <unistd.h>

#include "MappedCell.h"

// Static Class Variables
uint32_t MappedCell::page_size = getpagesize() * 17;
MappedCell * MappedCell::mmap_base_addr = (MappedCell *) mmap(
	NULL,
	MappedCell::page_size,
	PROT_READ | PROT_WRITE,
	MAP_ANONYMOUS | MAP_PRIVATE,
	-1,
	0
);

MappedCell * MappedCell::mmap_free_addr = MappedCell::mmap_base_addr;
size_t MappedCell::mmap_stride = MappedCell::page_size/sizeof(MappedCell);

// Public Member Functions
MappedCell::MappedCell() : 
	cell_base_addr(MappedCell::mmap_free_addr),
	data_status_registers((uint8_t *) MappedCell::mmap_free_addr)
{
	MappedCell::mmap_free_addr++;
}

MappedCell::~MappedCell() {

}

void MappedCell::initializeMap() {
        if(MappedCell::mmap_base_addr == (MappedCell *) -1)
                throw std::bad_alloc();
}

void MappedCell::cleanupMap() {
	munmap(MappedCell::mmap_base_addr, MappedCell::page_size);
}

bool MappedCell::isMapFull() {
        return (MappedCell::mmap_free_addr - MappedCell::mmap_base_addr) >= MappedCell::mmap_stride;
}

MappedCell * const MappedCell::getCellAddress() {
	return this->cell_base_addr;
}

uint8_t MappedCell::getCellStatus(uint8_t stride) const {
	return *(this->data_status_registers + stride);
}

void MappedCell::setCellStatus(uint8_t stride, uint8_t data) {
	*(this->data_status_registers + stride) = data;	
}
