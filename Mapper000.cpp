#include "Mapper000.hpp"
#include <stdint.h>

uint8_t Mapper000::ppu_map_read(uint16_t addr, uint16_t *mapped_addr)
{
	if (addr >= 0x0000 && addr <= 0x1FFF)
	{
		*mapped_addr = addr;
		return true;
	}

	*mapped_addr = 0x0000;
	return false;
}

uint8_t Mapper000::ppu_map_write(uint16_t addr, uint16_t *mapped_addr)
{
	// The system cannot write to the cartridge so 
	//	this is always rejected.
	*mapped_addr = 0x0000;
	return false;
}


uint8_t Mapper000::cpu_map_read(uint16_t addr, uint16_t *mapped_addr)
{
	if (addr >= 0x8000 && addr <= 0xFFFF)
	{
		*mapped_addr = addr & (prog_banks > 1 ? 0x7FFF : 0x3FFF);
		return true;
	}

	*mapped_addr = 0x0000;
	return false;
}

uint8_t Mapper000::cpu_map_write(uint16_t addr, uint16_t *mapped_addr)
{
	if (addr >= 0x8000 && addr <= 0xFFFF)
	{
		// If address within mapper range, then return mapped address which 
		// is indexed from zero to match ines file.
		// If 32kB then just remove address 0x8000 offset. 
		// If 16kB then mirror address (0x3FFF = 16kB). 
		*mapped_addr = addr & (prog_banks > 1 ? 0x7FFF : 0x3FFF);
		return true;
	}

	*mapped_addr = 0x0000;
	return false;
}