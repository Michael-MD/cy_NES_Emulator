cdef extern from "stdint.h":
	ctypedef unsigned char uint8_t
	ctypedef unsigned short uint16_t
	ctypedef short int16_t
	ctypedef unsigned int uint32_t

cdef class Channel:
	cdef uint32_t fs
	cdef float[:] buffer
	cdef float[:] wave
	# cdef object stream
	
	cdef uint8_t param_changed

	cdef uint8_t _enable
	cdef float _freq
	cdef uint16_t timer

cdef class PulseWave(Channel):
	cdef uint8_t v
	cdef float _dc
	cdef uint8_t C



cdef class APU:

	cdef PulseWave pulse_1
	cdef PulseWave pulse_2

	cdef void cpu_write(self, uint16_t addr, uint8_t data)
	cdef uint8_t cpu_read(self, uint16_t addr)
	
	cdef void clock(self)