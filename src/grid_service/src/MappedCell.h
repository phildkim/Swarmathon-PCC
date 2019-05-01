#ifndef MAPPEDCELL_H
#define MAPPEDCELL_H
#include <unistd.h>
#include <sys/types.h>
#include <sys/mman.h>
#include "Cell.h"

class MappedCell : public Cell {
private:
	static const uint8_t STATUS_REGISTER_OFFSET;
    static const uint8_t STATISTIC_REGISTER_OFFSET;    
	uint8_t * const cell_base_addr;
public:
        MappedCell(uint8_t *);
        ~MappedCell();
        uint8_t * const getCellAddress() override;
        float getCellStatistic(uint8_t) const override;
        void setCellStatistic(uint8_t, float) override;
        uint8_t getCellStatus(uint8_t) const override;
        void setCellStatus(uint8_t, uint8_t) override;
};

#endif

