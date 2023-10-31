#ifndef MAPPER000_HEADER
#define MAPPER000_HEADER

#include "Mapper.hpp"
#include <stdint.h>

class Mapper000 : public Mapper
{
public:
	using Mapper::Mapper;

	uint8_t ppu_map_read(uint16_t addr, uint16_t *mapped_addr) override;
	uint8_t ppu_map_write(uint16_t addr, uint16_t *mapped_addr) override;

	uint8_t cpu_map_read(uint16_t addr, uint16_t *mapped_addr) override;
	uint8_t cpu_map_write(uint16_t addr, uint16_t *mapped_addr) override;

};


#endif 