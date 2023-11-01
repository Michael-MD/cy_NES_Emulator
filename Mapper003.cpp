#include "Mapper003.hpp"
#include <stdint.h>

uint8_t Mapper003::ppu_map_read(uint16_t addr, uint32_t *mapped_addr)
{
	if (addr >= 0x0000 && addr <= 0x1FFF)
	{
		*mapped_addr = char_bank_select*0x2000 + (addr & 0x1FFF);
		return true;
	}

	*mapped_addr = 0x0000;
	return false;
}

uint8_t Mapper003::ppu_map_write(uint16_t addr, uint32_t *mapped_addr)
{
	return false;
}


uint8_t Mapper003::cpu_map_read(uint16_t addr, uint32_t *mapped_addr)
{
	if (addr >= 0x8000 && addr <= 0xFFFF)
	{
		*mapped_addr = addr & (prog_banks > 1 ? 0x7FFF : 0x3FFF);
		return true;
	}

	*mapped_addr = 0x0000;
	return false;
}

uint8_t Mapper003::cpu_map_write(uint16_t addr, uint32_t *mapped_addr, uint8_t data)
{
	if (addr >= 0x8000 && addr <= 0xFFFF)
	{
		char_bank_select = data & 0b11;
		*mapped_addr = addr;
	}

	return false;
}