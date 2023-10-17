cdef extern from "stdint.h":
	ctypedef unsigned char uint8_t
	ctypedef unsigned short uint16_t

cdef class Mapper:
	cdef uint8_t prog_banks
	cdef uint8_t char_banks