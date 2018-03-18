#include "AllocatedCell.h"

AllocatedCell::AllocatedCell() {
	this->status_registers.fill(0);
}

AllocatedCell::~AllocatedCell() {

}

uint8_t * const AllocatedCell::getCellAddress() {
	return (uint8_t *) this;
}

float AllocatedCell::getCellStatistic(uint8_t stride) const {
	return statistic_registers.at(stride);
}

void AllocatedCell::setCellStatistic(uint8_t stride, float data) {
	statistic_registers.at(stride) = data;
}

uint8_t AllocatedCell::getCellStatus(uint8_t stride) const {
	return status_registers.at(stride);
}

void AllocatedCell::setCellStatus(uint8_t stride, uint8_t data) {
	status_registers.at(stride) ^= data;
}
