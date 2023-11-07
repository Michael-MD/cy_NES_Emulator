#ifndef MAPPER001_HEADER
#define MAPPER001_HEADER

#include "Mapper.hpp"
#include <stdint.h>

class Mapper001 : public Mapper
{
public:
	Mapper001(uint8_t prog_banks, uint8_t char_banks);

	uint8_t ppu_map_read(uint16_t addr, uint32_t *mapped_addr) override;
	uint8_t ppu_map_write(uint16_t addr, uint32_t *mapped_addr) override;

	uint8_t cpu_map_read(uint16_t addr, uint32_t *mapped_addr, uint8_t *data) override;
	uint8_t cpu_map_write(uint16_t addr, uint32_t *mapped_addr, uint8_t data) override;

private:
	uint8_t char_bank_select = 0;

	uint8_t controlReg;
	uint8_t loadReg;
	uint8_t loadRegCounter;
	uint8_t targetReg;

	uint8_t CHARBankSelectLo;
	uint8_t CHARBankSelectHi;
	uint8_t CHARBankSelect8kB;

	uint8_t PRGBankSelectLo;
	uint8_t PRGBankSelectHi;
	uint8_t PRGBankSelect32kB;

	uint8_t onboardRAM[1024*32];

};


#endif 