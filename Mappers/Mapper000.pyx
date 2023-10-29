# cython: cflags=-O3, boundscheck=False, wraparound=False, cdivision=True, nonecheck=False, initializedcheck=False, overflowcheck=False



from .Mapper cimport Mapper

cdef extern from "stdint.h":
	ctypedef unsigned char uint8_t
	ctypedef unsigned short uint16_t

cdef class Mapper000(Mapper):
	# PPU with mapper communication
	cpdef tuple ppu_map_write(self, uint16_t addr, uint8_t data):
		"""
		The system cannot write to the cartridge so 
		this is always rejected.
		"""

		return (0x00, False)

	cpdef tuple ppu_map_read(self, uint16_t addr):
		cdef uint16_t mapped_addr
		if addr >= 0x0000 and addr <= 0x1FFF:
			mapped_addr = addr
			return (mapped_addr, True)

		return (0x00, False)

	# CPU to mapper communication 
	cpdef tuple cpu_map_write(self, uint16_t addr, uint8_t data):
		cdef uint16_t mapped_addr
		if addr >= 0x8000 and addr <= 0xFFFF:		# Cartridge
			"""
			If address within mapper range, then return mapped address which 
			is indexed from zero to match ines file.
			If 32kB then just remove address 0x8000 offset. 
			If 16kB then mirror address (0x3FFF = 16kB). 
			"""
			mapped_addr = addr & (0x7FFF if self.prog_banks > 1 else 0x3FFF)
			return (mapped_addr, True)

		return (0x00, False)


	cpdef tuple cpu_map_read(self, uint16_t addr):
		cdef uint16_t mapped_addr
		if addr >= 0x8000 and addr <= 0xFFFF:
			mapped_addr = addr & (0x7FFF if self.prog_banks > 1 else 0x3FFF)
			return (mapped_addr, True)

		return (0x0000, False)