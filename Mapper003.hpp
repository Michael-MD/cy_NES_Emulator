#ifndef MAPPER003_HEADER
#define MAPPER003_HEADER

#include "Mapper.hpp"
#include "Mapper003.hpp"
#include <stdint.h>

class Mapper003 : public Mapper
{
public:
	using Mapper::Mapper;

	uint8_t ppu_map_read(uint16_t addr, uint32_t *mapped_addr) override;
	uint8_t ppu_map_write(uint16_t addr, uint32_t *mapped_addr) override;

	uint8_t cpu_map_read(uint16_t addr, uint32_t *mapped_addr) override;
	uint8_t cpu_map_write(uint16_t addr, uint32_t *mapped_addr, uint8_t data) override;

private:
	uint8_t char_bank_select = 0;

};


#endif 