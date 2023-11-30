cdef extern from "stdint.h":
	ctypedef unsigned char uint8_t
	ctypedef unsigned short uint16_t
	ctypedef short int16_t
	ctypedef unsigned int uint32_t

from libc.math cimport round, ceil


cdef class Divider:
	cdef uint16_t period
	cdef uint16_t counter

	cdef uint8_t clock(self)
	cdef void reload(self)

cdef class Envelope:
	cdef uint8_t start
	cdef uint8_t loop
	cdef uint16_t decay_lvl
	cdef Divider divider

	cdef uint8_t clock(self)

cdef class Sweep:
	cdef Divider divider
	cdef uint8_t reload_f
	cdef uint8_t divider_P
	cdef uint8_t enable 
	cdef uint8_t negate 
	cdef uint8_t shift 
	cdef uint32_t _current_period 
	cdef uint32_t target_period 
	cdef uint8_t comp
	cdef uint8_t sweep_mute

	cdef void update_target_period(self)

	cdef uint8_t clock(self)

cdef class Channel:
	cdef uint32_t fs
	cdef float[:] buffer
	cdef float[:] wave
	cdef object stream
	
	cdef uint8_t param_changed
	cdef uint8_t _length_counter


	cdef uint8_t _enable
	cdef float _freq
	cdef uint16_t timer

	cdef void update_wave(self)

cdef class PulseWave(Channel):
	cdef uint8_t _v
	cdef float _volume
	cdef float _dc
	cdef uint8_t C
	cdef uint8_t H

	cdef Envelope envelope
	cdef Sweep sweep

	cdef void update_wave(self)


cdef class TriangleWave(Channel):
	cdef uint8_t C 		# Length counter halt / linear counter control
	cdef uint8_t _linear_counter
	cdef uint8_t linear_counter_reload_f
	cdef uint8_t new_linear_counter


	cdef void update_wave(self)

cdef class Noise(Channel):
	cdef uint8_t period
	cdef uint8_t loop
	cdef uint8_t C
	cdef uint8_t H
	
	cdef uint8_t _v
	cdef float _volume

	cdef Envelope envelope

	cdef void update_wave(self)

cdef class APU:
	cdef uint32_t n_apu_clock_cycles

	cdef PulseWave pulse_1
	cdef PulseWave pulse_2
	cdef TriangleWave triangle
	cdef Noise noise

	# Frame counter
	cdef uint8_t _fc_mode
	cdef uint8_t _fc_irq
	cdef uint8_t _fc_length_counter
	cdef uint8_t _fc_env_lin_counter
	cdef uint32_t _fc_counter
	cdef uint8_t _fc_irq_inhibit

	cdef void cpu_write(self, uint16_t addr, uint8_t data)
	cdef uint8_t cpu_read(self, uint16_t addr)
	
	cdef void clock(self)
	cdef void quarter_frame_clock(self)
	cdef void half_frame_clock(self)