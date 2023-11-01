#include "Mapper002.hpp"
#include <stdint.h>

Mapper002::Mapper002(uint8_t prog_banks, uint8_t char_banks) : Mapper(prog_banks, char_banks)
{
	n_prog_bank_select_lo = 0;
	n_prog_bank_select_hi = prog_banks - 1;
}

uint8_t Mapper002::ppu_map_read(uint16_t addr, uint32_t *mapped_addr)
{
	if (addr < 0x2000)
	{
		*mapped_addr = addr;
		return true;
	}

	*mapped_addr = 0x0000;
	return false;
}

uint8_t Mapper002::ppu_map_write(uint16_t addr, uint32_t *mapped_addr)
{
	if (addr < 0x2000)
	{
		if (char_banks == 0)
		{
			*mapped_addr = addr;
			return true;
		}
	}

	*mapped_addr = 0x0000;
	return false;
}


uint8_t Mapper002::cpu_map_read(uint16_t addr, uint32_t *mapped_addr)
{
	if (addr >= 0x8000 && addr <= 0xBFFF)
	{
		// Offsets into specific bank and mirrors data into some 16kB range
		*mapped_addr = n_prog_bank_select_lo * 0x4000 + (addr & 0x3FFF);
		return true;
	}
	else if (addr >= 0xC000 && addr < 0xFFFF)
	{
		*mapped_addr = n_prog_bank_select_hi * 0x4000 + (addr & 0x3FFF);
		return true;
	}

	*mapped_addr = 0x0000;
	return false;
}

uint8_t Mapper002::cpu_map_write(uint16_t addr, uint32_t *mapped_addr, uint8_t data)
{
	if (addr >= 0x8000 && addr < 0xFFFF)
	{
		n_prog_bank_select_lo = data & 0x0F;
	}

	return false;
}