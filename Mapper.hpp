#ifndef MAPPER_HEADER
#define MAPPER_HEADER

#include <stdint.h>

enum mirrormode
{
	HORIZONTAL,
	VERTICAL,
	ONESCREEN_LO,
	ONESCREEN_HI,
};


class Mapper
{
public:
	uint8_t prog_banks;
	uint8_t char_banks;

	Mapper(uint8_t prog_banks, uint8_t char_banks);
	~Mapper();

	virtual uint8_t ppu_map_read(uint16_t addr, uint32_t *mapped_addr) = 0;
	virtual uint8_t ppu_map_write(uint16_t addr, uint32_t *mapped_addr) = 0;

	virtual uint8_t cpu_map_read(uint16_t addr, uint32_t *mapped_addr, uint8_t *data) = 0;
	virtual uint8_t cpu_map_write(uint16_t addr, uint32_t *mapped_addr, uint8_t data) = 0;

	int get_mirroring_mode(void);
	int mirroring = 0;
};


#endif 