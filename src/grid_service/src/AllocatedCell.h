#ifndef ALLOCATEDCELL_H
#define ALLOCATEDCELL_H
#include <array>
#include "Cell.cpp"

class AllocatedCell : public Cell {
private:
        std::array<uint8_t, 4> status_registers;
        std::array<float, 2> statistic_registers;
public:
        AllocatedCell();
        ~AllocatedCell();
        uint8_t * const getCellAddress() override;
        float getCellStatistic(uint8_t) const override;
        void setCellStatistic(uint8_t, float) override;
        uint8_t getCellStatus(uint8_t) const override;
        void setCellStatus(uint8_t, uint8_t) override;
};

#endif

