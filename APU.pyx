# cython: boundscheck=False, wraparound=False, initializedcheck=False, cdivision_warnings=False

import pyaudio
import numpy as np

from .length_conter_tbl import *

"""
The implementation of audio used here means buffer size sets 
the limit on the minimal frequency that can be played since
the buffer must be able to fit atleast a single period. The 
minimial frequency is given by
					
					f_min = fs / buffer_size

"""
cdef uint32_t buffer_size = 200

p = pyaudio.PyAudio()


cdef class Divider:
	def __cinit__(self):
		self.period = 0
		self.counter = 0

	cdef uint8_t clock(self):
		if self.counter == 0:
			self.reload()
			return 1
		else:
			self.counter -= 1
			return 0

	cdef void reload(self):
		self.counter = self.period


cdef class Envelope:
	def __cinit__(self):
		self.start = 0
		self.decay_lvl = 0
		self.loop = 0
		self.divider = Divider()

	cdef uint8_t clock(self):
		if self.start == 0:
			if self.divider.clock() == 1:

				if self.decay_lvl == 0:
					if self.loop:
						self.decay_lvl = 15
				else:
					self.decay_lvl -= 1
				
				return 1

		else:
			self.start = 0
			self.decay_lvl = 15
			self.divider.reload()

		return 0

cdef class Sweep:
	def __cinit__(self, comp):
		self.divider = Divider()
		self.reload_f = 0
		self.enable = 0
		self.negate = 0
		self.shift = 0
		self._current_period = 0
		self.target_period = 0
		self.shift = 0
		self.comp = comp

	@property
	def current_period(self):
		return self._current_period

	@current_period.setter
	def current_period(self, v):
		if self._current_period != v:
			self._current_period = v

			self.update_target_period()		

	cdef void update_target_period(self):
		# Calculate target period
		cdef uint16_t deltat
		cdef int tmp

		deltat = self._current_period >> self.shift

		if self.negate:
			tmp = self._current_period - deltat - self.comp
		else:
			tmp = self._current_period + deltat
			

		self.target_period = tmp if tmp > 0 else 0 		# Clamp target period to 0
		self.sweep_mute = self._current_period < 8 or self.target_period > 0x07FF

	cdef uint8_t clock(self):
		self.update_target_period()

		if self.reload_f:
			self.divider.reload()
			self.reload_f = 0
		
		if self.divider.clock() and self.enable and self.shift != 0:
			return 1

		return 0

cdef class Channel:
	def __cinit__(self):
		self.fs = 44100
		self.buffer = np.asarray([], dtype=np.float32)	# Audio buffer
		self.wave = np.zeros(buffer_size, dtype=np.float32)
		self.param_changed = False
		self._freq = 440
		self._enable = False

		# Define and start audio stream (non-blocking)
		self.stream = p.open(format=pyaudio.paFloat32,
				channels=1,
				rate=self.fs,
				output=True,
				frames_per_buffer=buffer_size,
				stream_callback=self.update_buffer)
		self.stream.start_stream()


	def update_buffer(self, in_data, frame_count, time_info, status):
		if self.param_changed and self._enable and self._length_counter > 0:
			self.update_wave()
			self.param_changed = False

		if len(self.buffer) < frame_count:
			self.buffer = np.append(self.buffer, self.wave)

		samples_to_play = np.copy(self.buffer[:frame_count]) * self.enable
		self.buffer = self.buffer[frame_count:]

		return (samples_to_play.tobytes(), pyaudio.paContinue)

	cdef void empty_buffer(self):
		self.buffer = np.asarray([], dtype=np.float32)

	cdef void update_wave(self):	# Intended to be overwritten in derived class
		...


	@property
	def enable(self):
		return self._enable

	@enable.setter
	def enable(self, v):
		if self._enable != v:
			self._enable = v

			if not self._enable:
				# self.buffer = np.asarray([], dtype=np.float32)
				self.wave = np.zeros(buffer_size, dtype=np.float32)
	
	@property
	def length_counter(self):
		return self._length_counter

	@length_counter.setter
	def length_counter(self, v):
		self._length_counter = v
		if self._length_counter == 0:
			# TODO: set current wave to zero rather than make new array
			self.wave = np.zeros(buffer_size, dtype=np.float32)

	@property
	def freq(self):
		return self._freq

	@freq.setter
	def freq(self, v):
		if self._freq != v:
			self._freq = v
			self.param_changed = True


cdef class PulseWave(Channel):
	def __cinit__(self, comp):
		self.envelope = Envelope()
		self.sweep = Sweep(comp)

	@property
	def dc(self):
		return self._dc

	@dc.setter
	def dc(self, v):
		if self._dc != v:
			self._dc = v
			self.param_changed = True

	@property
	def volume(self):
		return self._volume

	@volume.setter
	def volume(self, vv):
		cdef float m, f
		cdef int i
		if self._volume != vv / 15:

			if self.C == 1:
				self._volume = vv / 15
			else:
				if self.envelope.divider.period == 0:
					self._volume = 0
				else:
					self._volume = self.envelope.decay_lvl / 15

			# TODO: modify wave rather than recalc
			self.param_changed = True

	@property
	def v(self):
		return self._v

	@v.setter
	def v(self, vv):
		if self._v != vv:
			self._v = vv
			self.envelope.divider.period = vv
			self.volume = vv


	cdef void update_wave(self):
		cdef int cycles = <int> ceil(buffer_size/self.fs*self._freq)+1
		cdef int num_samples = <int> round(self.fs / self._freq)
		cdef float A = .1 * self._volume
		cdef float[:] signal = np.full(num_samples * cycles, -A/2, dtype=np.float32)
		cdef int hi_ind = int(num_samples * self._dc)
		cdef uint32_t i, s, e

		if self.sweep.sweep_mute == 0:
			for i in range(cycles):
				s = i*num_samples
				e = i*num_samples + hi_ind
				signal[s:e] = A/2

		self.wave = signal		

cdef class TriangleWave(Channel):
	def __cinit__(self):
		self.linear_counter_reload_f = False

	@property
	def linear_counter(self):
		return self._linear_counter

	@linear_counter.setter
	def linear_counter(self, v):
		self._linear_counter = v
		if self._linear_counter == 0:
			self.wave = np.zeros(buffer_size, dtype=np.float32)


	cdef void update_wave(self):
		cdef int cycles = <int> ceil(buffer_size/self.fs*self._freq)+1
		cdef int num_samples = <int> round(self.fs / self._freq)
		
		cdef float A = .1
		cdef float[:] signal
		cdef int hi_ind = int(num_samples * .5)
		cdef uint32_t i, s, e
		cdef float c

		if self._linear_counter > 0 and self._freq < 1500:
			signal = np.full(num_samples * cycles, -A/2, dtype=np.float32)

			for i in range(cycles):
				s = i*num_samples
				e = i*num_samples + hi_ind
				signal[s:e] = A/2

			# Perform cumulative sum
			c = 4 / num_samples
			for i in range(1, num_samples * cycles):
				signal[i] = (signal[i-1] + signal[i]) 

			for i in range(num_samples * cycles):
				signal[i] = signal[i] * c

		else:
			signal = np.zeros(buffer_size, dtype=np.float32)
		
		self.wave = signal


cdef class Noise(Channel):
	def __cinit__(self):
		self.envelope = Envelope()

	@property
	def volume(self):
		return self._volume

	@volume.setter
	def volume(self, vv):
		if self._volume != vv / 15:

			if self.C == 1:
				self._volume = vv / 15
			else:
				self._volume = self.envelope.decay_lvl / 15

			# TODO: modify wave rather than recalc
			self.param_changed = True

	@property
	def v(self):
		return self._v

	@v.setter
	def v(self, vv):
		if self._v != vv:
			self._v = vv
			self.envelope.divider.period = vv
			self.volume = vv


	cdef void update_wave(self):
		A = .1

		self.wave = (np.random.uniform(-A/2, A/2, buffer_size*6)*self.volume).astype(np.float32)


cdef class APU:
	def __cinit__(self):
		self.n_apu_clock_cycles = 0

		self.pulse_1 = PulseWave(1)
		self.pulse_2 = PulseWave(0)
		self.triangle = TriangleWave()
		self.noise = Noise()

	cdef void quarter_frame_clock(self):
		if self.pulse_1.envelope.clock():
			if self.pulse_1.C != 1:
				self.pulse_1.volume = self.pulse_1.envelope.decay_lvl / 15
				self.pulse_1.param_changed = True

		if self.pulse_2.envelope.clock():
			if self.pulse_2.C != 1:
				self.pulse_2.volume = self.pulse_2.envelope.decay_lvl / 15
				self.pulse_2.param_changed = True

		if self.noise.envelope.clock():
			if self.noise.C != 1:
				self.noise.volume = self.noise.envelope.decay_lvl / 15
				self.noise.param_changed = True

		# Load/decrement triangle linear counter
		if self.triangle.linear_counter_reload_f:
			self.triangle.linear_counter = self.triangle.new_linear_counter
			self.triangle.param_changed = True

		elif self.triangle._linear_counter != 0:
			self.triangle.linear_counter -= 1

		if self.triangle.C == 0:
			self.triangle.linear_counter_reload_f = 0

	cdef void half_frame_clock(self):

		if self.pulse_1.sweep.clock() and self.pulse_1.sweep.enable and not self.pulse_1.sweep.sweep_mute:
			self.pulse_1.timer = self.pulse_1.sweep.target_period
			self.pulse_1.sweep.current_period = self.pulse_1.sweep.target_period
			self.pulse_1.freq = 1789773 / (16*(self.pulse_1.timer+1))

		if self.pulse_2.sweep.clock() and self.pulse_2.sweep.enable and not self.pulse_2.sweep.sweep_mute:
			self.pulse_2.timer = self.pulse_2.sweep.target_period
			self.pulse_2.sweep.current_period = self.pulse_2.sweep.target_period
			self.pulse_2.freq = 1789773 / (16*(self.pulse_2.timer+1))

		# Decrement length counters
		if self.pulse_1.H==0 and self.pulse_1._length_counter != 0:
			self.pulse_1.length_counter -= 1

		if self.pulse_2.H==0 and self.pulse_2._length_counter != 0:
			self.pulse_2.length_counter -= 1

		if self.triangle.C==0 and self.triangle._length_counter != 0:
			self.triangle.length_counter -= 1

		if self.noise.H==0 and self.noise._length_counter != 0:
			self.noise.length_counter -= 1

	cdef void clock(self):
		self.n_apu_clock_cycles += 1

		if self.n_apu_clock_cycles%2:		# 1 APU cycle = 2 CPU cycles
			
			# Frame counter
			self._fc_counter += 1

			# Mode 0: 4-step mode
			if self._fc_mode == 0:
				if self._fc_counter == 3728:
					self.quarter_frame_clock()
				elif self._fc_counter == 7456:
					self.quarter_frame_clock()
					self.half_frame_clock()
				elif self._fc_counter == 11185:
					self.quarter_frame_clock()
				elif self._fc_counter == 14916:
					self.quarter_frame_clock()
					self.half_frame_clock()

					if self._fc_irq_inhibit == 0:
						self._fc_irq = 1

					self._fc_counter = 0


			# Mode 1: 5-step mode
			if self._fc_mode == 1:
				if self._fc_counter == 3728:
					self.quarter_frame_clock()
				elif self._fc_counter == 7456:
					self.quarter_frame_clock()
					self.half_frame_clock()
				elif self._fc_counter == 11185:
					self.quarter_frame_clock()
				elif self._fc_counter == 14914:
					...	
				elif self._fc_counter == 18640:
					self.quarter_frame_clock()
					self.half_frame_clock()

					self._fc_counter = 0


	cdef void cpu_write(self, uint16_t addr, uint8_t data):
		if addr == 0x4000:	# Pulse 1 duty cycle
			if (data>>6) == 0:	# 12.5% d.c
				self.pulse_1.dc = .125

			elif (data>>6) == 1:	# 25% d.c
				self.pulse_1.dc = .25

			elif (data>>6) == 2:	# 50% d.c
				self.pulse_1.dc = .5

			elif (data>>6) == 3:	# 25% d.c negated
				self.pulse_1.dc = .75

			self.pulse_1.C = (data>>4)&0x01
			self.pulse_1.envelope.loop = (data>>5)&0x01
			self.pulse_1.H = self.pulse_1.envelope.loop
			self.pulse_1.v = data&0x0F

		elif addr == 0x4001:	# Set sweep unit parameters for pulse wave 1
			self.pulse_1.sweep.enable = (data>>7)
			self.pulse_1.sweep.divider.period = (data>>4)&0b111
			self.pulse_1.sweep.negate = (data>>3)&0x01
			self.pulse_1.sweep.shift = (data>>0)&0b111

			self.pulse_1.sweep.reload_f = 1
			self.pulse_1.sweep.update_target_period()

		elif addr == 0x4002:	# Pulse 1 lo bit t
			self.pulse_1.timer = (self.pulse_1.timer&0x0FF00)|data
			self.pulse_1.sweep.current_period = self.pulse_1.timer
			self.pulse_1.freq = 1789773 / (16*(self.pulse_1.timer+1))

		elif addr == 0x4003:	# Pulse 1 hi bit t
			self.pulse_1.timer = ((data&0b111)<<8) | (self.pulse_1.timer&0x000FF)
			self.pulse_1.sweep.current_period = self.pulse_1.timer
			self.pulse_1.freq = 1789773 / (16*(self.pulse_1.timer+1))

			# Reset envelope period
			self.pulse_1.envelope.decay_lvl = 15

			if self.pulse_1._enable:
				self.pulse_1._length_counter = length_conter_tbl[data>>3]

			self.pulse_1.envelope.start = 1

			self.pulse_1.empty_buffer()

		elif addr == 0x4004:	# Pulse 2 duty cycle
			if (data>>6) == 0:	# 12.5% d.c
				self.pulse_2.dc = .125

			elif (data>>6) == 1:	# 25% d.c
				self.pulse_2.dc = .25

			elif (data>>6) == 2:	# 50% d.c
				self.pulse_2.dc = .5

			elif (data>>6) == 3:	# 25% d.c negated
				self.pulse_2.dc = .75

			self.pulse_2.C = (data>>4)&0x01
			self.pulse_2.envelope.loop = (data>>5)&0x01
			self.pulse_2.H = self.pulse_2.envelope.loop
			self.pulse_2.v = data&0x0F

		elif addr == 0x4005:	# Set sweep unit parameters for pulse wave 2
			self.pulse_2.sweep.enable = (data>>7)
			self.pulse_2.sweep.divider.period = (data>>4)&0b111
			self.pulse_2.sweep.negate = (data>>3)&0x01
			self.pulse_2.sweep.shift = (data>>0)&0b111

			self.pulse_2.sweep.reload_f = 1
			self.pulse_2.sweep.update_target_period()

		elif addr == 0x4006:	# Pulse 2 lo bit t
			self.pulse_2.timer = (self.pulse_2.timer&0x0FF00)|data
			self.pulse_2.sweep.current_period = self.pulse_2.timer
			self.pulse_2.freq = 1789773 / (16*(self.pulse_2.timer+1))
			
		elif addr == 0x4007:	# Pulse 2 hi bit t
			self.pulse_2.timer = ((data&0b111)<<8) | (self.pulse_2.timer&0x000FF)
			self.pulse_2.sweep.current_period = self.pulse_2.timer
			self.pulse_2.freq = 1789773 / (16*(self.pulse_2.timer+1))

			# Reset envelope period
			self.pulse_2.envelope.decay_lvl = 15

			if self.pulse_2.enable:
				self.pulse_2._length_counter = length_conter_tbl[data>>3]

			self.pulse_2.envelope.start = 1

			self.pulse_2.empty_buffer()

		elif addr == 0x4008: 	# Triangle linear counter
			self.triangle.new_linear_counter = data & 0x7F
			self.triangle.C = data>>7

		elif addr == 0x400A:	# Triangle lo bit t
			self.triangle.timer = (self.triangle.timer&0x0FF00)|data
			self.triangle.freq = 1789773 / (32*(self.triangle.timer+1))

		elif addr == 0x400B:	# Triangle hi bit t
			self.triangle.timer = ((data&0b111)<<8) | (self.triangle.timer&0x000FF)
			self.triangle.freq = 1789773 / (32*(self.triangle.timer+1))

			self.triangle._length_counter = length_conter_tbl[data>>3]

			self.triangle.linear_counter_reload_f = True

		elif addr == 0x400C:		# Noise envelope
			self.noise.C = (data>>4)&0x01
			self.noise.envelope.loop = (data>>5)&0x01
			self.noise.H = self.noise.envelope.loop
			self.noise.v = data&0x0F

		elif addr == 0x400E:		# Noise wave properties
			self.noise.period = data&0x0F
			self.noise.loop = data>>7

		elif addr == 0x400F:		# Noise length counter
			self.noise._length_counter = length_conter_tbl[data>>3]

		elif addr == 0x4015:	# Status register Enable/Disable channels
			# self.pulse_1.enable = True if data & 0b01 else False
			# self.pulse_2.enable = True if data & 0b10 else False
			# self.triangle.enable = True if data & 0b100 else False
			self.noise.enable = True if data & 0b1000 else False

			# TODO: use _length_counter here
			if not self.pulse_1._enable:
				self.pulse_1.length_counter = 0

			if not self.pulse_2.enable:
				self.pulse_2.length_counter = 0

			if not self.triangle.enable:
				self.triangle.length_counter = 0

		elif addr == 0x4017:	# Frame counter status
			self._fc_mode = (data&0x80)>>7 	# if mode=0: 4-step, mode=1: 5-step
			self._fc_irq_inhibit = (data&0x40)>>6

			if self._fc_mode == 0: 	# Mode 0
				self._fc_irq = data&0x01
				self._fc_length_counter = ((data&0b100)>>1)|(data&0x01)
				self._fc_env_lin_counter = data&0x0F

			elif self._fc_mode == 1: 	# Mode 1
				self._fc_length_counter = ((data&0b1000)>>2)|(data&0x01)
				self._fc_irq = 0
				self._fc_env_lin_counter = ((data&0b11100)>>1)|(data&0x01)

			self.quarter_frame_clock()
			self.half_frame_clock()


	cdef uint8_t cpu_read(self, uint16_t addr):
		return 0x00
	
