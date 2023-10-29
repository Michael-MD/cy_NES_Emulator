cdef extern from "stdint.h":
	ctypedef unsigned char uint8_t
	ctypedef unsigned short uint16_t

cdef class Cartridge:
	"""
	Pattern (char) memory which contains sprite informations is located 
	at address range 0x0000 to 0x1FFF.
	"""
	cdef object mapper

	cdef uint8_t n_prog_chunks
	cdef uint8_t n_char_chunks
	cdef uint8_t[:] _v_prog_memory
	cdef uint8_t[:] _v_char_memory

	cdef uint8_t ppu_write(self, uint16_t addr, uint8_t data)
	cpdef uint8_t cpu_write(self, uint16_t addr, uint8_t data)
	cdef uint8_t ppu_read(self, uint16_t addr, uint8_t* data)