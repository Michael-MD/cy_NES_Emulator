cdef extern from "stdint.h":
	ctypedef unsigned char uint8_t
	ctypedef unsigned short uint16_t

from .PPU cimport PPU
from .CPU6502 cimport CPU6502
from .APU cimport APU

cdef class Bus:
	cdef CPU6502 cpu
	cdef PPU ppu
	cdef APU apu
	cdef object cartridge
	cdef uint8_t cpu_ram[2048]
	cdef int n_system_clock_counter
	cdef uint8_t _controller[2]
	cdef uint8_t _controller_state[2]
	cdef int n_mapper_ID

	cdef uint8_t dma_page
	cdef uint8_t dma_addr
	cdef uint8_t dma_transfer
	cdef uint8_t dma_dummy
	cdef uint8_t dma_data
	cdef uint8_t _entry

	cpdef void clock(self, int N)
	cpdef void write(self, uint16_t addr, uint8_t data)
	cpdef uint8_t read(self, uint16_t addr)