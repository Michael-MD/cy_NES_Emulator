#include "Mapper001.hpp"
#include <stdint.h>
#include <iostream>

Mapper001::Mapper001(uint8_t prog_banks, uint8_t char_banks) : Mapper(prog_banks, char_banks)
{
	controlReg = 0x1C;
	loadReg = 0x00;
	loadRegCounter = 0x00;

	CHARBankSelectLo = 0;
	CHARBankSelectHi = 0;
	CHARBankSelect8kB = 0;

	PRGBankSelectLo = 0;
	PRGBankSelectHi = prog_banks - 1;
	PRGBankSelect32kB = 0;
}

uint8_t Mapper001::ppu_map_read(uint16_t addr, uint32_t *mapped_addr)
{
	if (addr < 0x2000)
	{
		if (char_banks == 0)	// If cartridge doesn't contain any CHR memory
		{
			*mapped_addr = addr;
			return true;
		}
		else
		{
			if (controlReg & 0x10)
			{
				if (addr >= 0x0000 && addr <= 0x0FFF)
				{
					*mapped_addr = CHARBankSelectLo * 0x1000 + (addr & 0x0FFF);
					return true;
				}

				if (addr >= 0x1000 && addr <= 0x1FFF)
				{
					*mapped_addr = CHARBankSelectHi * 0x1000 + (addr & 0x0FFF);
					return true;
				}
			}
			else
			{
				*mapped_addr = CHARBankSelect8kB * 0x2000 + (addr & 0x1FFF);
				return true;
			}
		}
	}

	return false;
}

uint8_t Mapper001::ppu_map_write(uint16_t addr, uint32_t *mapped_addr)
{
	if (addr < 0x2000)
	{
		if (char_banks == 0)
		{
			*mapped_addr = addr;
		}
		return true;
	}
	
	return false;
	
}


uint8_t Mapper001::cpu_map_read(uint16_t addr, uint32_t *mapped_addr, uint8_t *data)
{
	if (addr >= 0x6000 && addr <= 0x7FFF)
	{
		*mapped_addr = 0xFFFFFFFF;
		*data = onboardRAM[addr & 0x1FFF];
		return true;
	}
	if (addr >= 0x8000)
	{
		if (controlReg & 0b01000)
		{
			// 16kB Mode
			if (addr >= 0x8000 && addr <= 0xBFFF)
			{
				*mapped_addr = PRGBankSelectLo * 0x4000 + (addr & 0x3FFF);
				return true;
			}

			else if (addr >= 0xC000 && addr <= 0xFFFF)
			{
				*mapped_addr = PRGBankSelectHi * 0x4000 + (addr & 0x3FFF);
				return true;
			}
		}
		else
		{
			// 32kB Mode
			*mapped_addr = PRGBankSelect32kB * 0x8000 + (addr & 0x7FFF);
			return true;
		}
	}

	return false;
}



uint8_t Mapper001::cpu_map_write(uint16_t addr, uint32_t *mapped_addr, uint8_t data)
{
	if (addr >= 0x6000 && addr <= 0x7FFF)
	{
		*mapped_addr = 0xFFFFFFFF;
		onboardRAM[addr & 0x1FFF] = data;
		return true;
	}

	if (addr >= 0x8000)
	{
		if (data & 0x80)
		{
			// Reset registers
			loadReg = 0x00;
			loadRegCounter = 0;
			controlReg = controlReg | 0x0C;
		}
		else
		{
			// Serially fill load register starting LSB first
			loadReg >>= 1;
			loadReg |= (data&0x01)<<4;
			loadRegCounter++;

			if (loadRegCounter == 5)
			{
				targetReg = (addr>>13)&0x03;
				if (targetReg == 0)
				{
					// Set control reg
					controlReg = loadReg & 0x1F;

					switch (controlReg & 0x03)
					{
					case 0: mirroring = ONESCREEN_LO ; break;
					case 1: mirroring = ONESCREEN_HI ; break;
					case 2: mirroring = VERTICAL; 	break;
					case 3: mirroring = HORIZONTAL; break;
					}
				}
				else if (targetReg == 1)
				{
					if (controlReg & 0x10)
					{
						CHARBankSelectLo = loadReg & 0x1F;
					}
					else
					{
						CHARBankSelect8kB = loadReg & 0x1E;
					}
				}
				else if (targetReg == 2)
				{
					if (controlReg & 0x10)
					{
						CHARBankSelectHi = loadReg & 0x1F;
					}
				}
				else if (targetReg == 3)
				{
					uint8_t PRGMode = (controlReg>>2)&0x03;
					if (PRGMode == 0 || PRGMode == 1)
					{
						PRGBankSelect32kB = (loadReg&0x0E)>>1;
					}
					else if (PRGMode == 2)
					{
						PRGBankSelectLo = 0;
						PRGBankSelectHi = loadReg&0x0F;
					}
					else if (PRGMode == 3)
					{
						PRGBankSelectLo = loadReg&0x0F;
						PRGBankSelectHi = prog_banks - 1;
					}
				}
				loadReg = 0x00;
				loadRegCounter = 0;
			}
		}

	}

	return false;
}