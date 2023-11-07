#ifndef MAPPER002_HEADER
#define MAPPER002_HEADER

#include "Mapper.hpp"
#include <stdint.h>

class Mapper002 : public Mapper
{
public:
	Mapper002(uint8_t prog_banks, uint8_t char_banks);

	uint8_t ppu_map_read(uint16_t addr, uint32_t *mapped_addr) override;
	uint8_t ppu_map_write(uint16_t addr, uint32_t *mapped_addr) override;

	uint8_t cpu_map_read(uint16_t addr, uint32_t *mapped_addr, uint8_t *data) override;
	uint8_t cpu_map_write(uint16_t addr, uint32_t *mapped_addr, uint8_t data) override;

private:
	uint16_t n_prog_bank_select_lo;
	uint16_t n_prog_bank_select_hi;

};


#endif 