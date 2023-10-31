#include "Mapper.hpp"
#include <stdint.h>

Mapper::Mapper(uint8_t prog_banks, uint8_t char_banks)
{
	this->prog_banks = prog_banks;
	this->char_banks = char_banks;
}

Mapper::~Mapper() {};

