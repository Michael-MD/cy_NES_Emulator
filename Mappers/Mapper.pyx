cdef extern from "stdint.h":
	ctypedef unsigned char uint8_t
	ctypedef unsigned short uint16_t

cdef class Mapper:
	def __init__(self, prog_banks, char_banks):
		self.prog_banks = prog_banks
		self.char_banks = char_banks