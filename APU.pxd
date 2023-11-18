cdef extern from "stdint.h":
	ctypedef unsigned char uint8_t
	ctypedef unsigned short uint16_t
	ctypedef short int16_t
	ctypedef unsigned int uint32_t

cdef class Channel:
	cdef uint32_t channel_id
	cdef object channel

cdef class PulseWave(Channel):
	cdef uint8_t enable
	cdef uint8_t v
	cdef float freq
	cdef uint16_t timer

	cdef float dc
	cdef uint8_t C


cdef class APU:
	cdef uint32_t fs

	cdef PulseWave pulse_1
	cdef PulseWave pulse_2

	cdef void cpu_write(self, uint16_t addr, uint8_t data)
	cdef uint8_t cpu_read(self, uint16_t addr)
	
	cdef void clock(self)