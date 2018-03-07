#include "Cell.h"

Cell * Cell::getBestNeighbor() {
    if(this->most_valuable_neighbor.expired())
        return nullptr;
    else
        return this->most_valuable_neighbor.lock().get();
}

void Cell::setBestNeighbor(std::shared_ptr<Cell> neighbor, uint8_t x, uint8_t y) {
    this->most_valuable_neighbor = neighbor;
}