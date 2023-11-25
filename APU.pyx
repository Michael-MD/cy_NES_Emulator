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
buffer_size = 200

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
	def __init__(self):
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
		cdef int deltat, tmp

		deltat = self.current_period >> self.shift
		if self.negate:
			tmp = self.current_period - deltat - self.comp
			self.target_period = tmp if tmp>0 else 0 		# Clamp target period to 0
		else:
			self.target_period = self.current_period + deltat


		self.sweep_mute = self.current_period < 8 or self.target_period > 0x07FF

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
		if self.param_changed:
			self.update_wave()
			self.param_changed = False

		if len(self.buffer) < frame_count:
			self.buffer = np.append(self.buffer, self.wave)

		samples_to_play = np.copy(self.buffer[:frame_count]) * self.enable
		self.buffer = self.buffer[frame_count:]

		return (samples_to_play.tobytes(), pyaudio.paContinue)

	def update_wave(self):	# Intended to be overwritten in derived class
		...


	@property
	def enable(self):
		return self._enable

	@enable.setter
	def enable(self, v):
		if self._enable != v:
			self._enable = v

			if not self._enable:
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
		if self._volume != vv:

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

	def update_wave(self):
		cycles = int(np.ceil(buffer_size/self.fs*self.freq)+1)
		num_samples = int(np.round(self.fs / self.freq))
		
		signal = np.zeros(num_samples)

		if self.sweep.sweep_mute == 0 and self.length_counter > 0:
			t = np.arange(num_samples) / self.fs
			signal[:int(num_samples * self.dc)] = .1 * self.volume

		self.wave = np.tile(signal.astype(np.float32), cycles)


cdef class TriangleWave(Channel):
	def approx_sin(self, t):
		j = t * 0.15915
		j = j - np.floor(j)
		return (20.785 * j * (j-.5) * (j-1))

	def update_wave(self):
		cycles = int(np.ceil(buffer_size/self.fs*self.freq)+1)
		num_samples = int(np.round(self.fs / self.freq))

		signal = np.zeros(num_samples)

		t = np.arange(num_samples) / self.fs
		for n in range(1, 5, 2):
			signal += pow(-1, (n-1)/2) * self.approx_sin(2 * np.pi * n * self.freq * t) / (n**2)

		signal *= .1

		self.wave = np.tile(signal.astype(np.float32), cycles)


cdef class APU:
	def __cinit__(self):
		self.n_apu_clock_cycles = 0

		self.pulse_1 = PulseWave(1)
		self.pulse_2 = PulseWave(0)
		self.triangle = TriangleWave()

	cdef void quarter_frame_clock(self):
		if self.pulse_1.envelope.clock():
			if self.pulse_1.C != 1:
				self.pulse_1.param_changed = True

		if self.pulse_2.envelope.clock():
			if self.pulse_2.C != 1:
				self.pulse_2.param_changed = True

	cdef void half_frame_clock(self):

		if self.pulse_1.sweep.clock():
			self.pulse_1.timer = self.pulse_1.sweep.target_period
			self.pulse_1.sweep.current_period = self.pulse_1.sweep.target_period
			self.pulse_1.freq = 1789773 / (16*(self.pulse_1.timer+1))

		if self.pulse_2.sweep.clock():
			self.pulse_2.timer = self.pulse_2.sweep.target_period
			self.pulse_2.sweep.current_period = self.pulse_2.sweep.target_period
			self.pulse_2.freq = 1789773 / (16*(self.pulse_2.timer+1))

	cdef void clock(self):
		self.n_apu_clock_cycles += 1

		if self.n_apu_clock_cycles%2:		# 1 APU cycle = 2 CPU cycles
			
			# Frame counter
			self._fc_counter += 1

			# Mode 0: 4-step mode
			if self._fc_mode == 0:
				if self._fc_counter%3728 == 0:
					self.quarter_frame_clock()
				elif self._fc_counter%7456 == 0:
					self.quarter_frame_clock()
					self.half_frame_clock()
				elif self._fc_counter%11185 == 0:
					self.quarter_frame_clock()
				elif self._fc_counter%14916 == 0:
					self.quarter_frame_clock()
					self.half_frame_clock()

					if self._fc_irq_inhibit == 0:
						self._fc_irq = 1

					self._fc_counter = 0


			# Mode 1: 5-step mode
			if self._fc_mode == 1:
				if self._fc_counter%3728 == 0:
					self.quarter_frame_clock()
				elif self._fc_counter%7456 == 0:
					self.quarter_frame_clock()
					self.half_frame_clock()
				elif self._fc_counter%11185 == 0:
					self.quarter_frame_clock()
				elif self._fc_counter%14914 == 0:
					...	
				elif self._fc_counter%18641 == 0:
					self.quarter_frame_clock()
					self.half_frame_clock()

					self._fc_counter = 0

			# Increment length counters
			if self._fc_counter == 0:
				if self.pulse_1.H==0 and self.pulse_1.length_counter != 0:
					self.pulse_1.length_counter -= 1

				if self.pulse_2.H==0 and self.pulse_2.length_counter != 0:
					self.pulse_2.length_counter -= 1

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

		elif addr == 0x4003:	# Pulse 1 hi bit t
			self.pulse_1.timer = ((data&0b111)<<8) | (self.pulse_1.timer&0x000FF)
			self.pulse_1.sweep.current_period = self.pulse_1.timer
			self.pulse_1.freq = 1.789773*1e6 / (16*(self.pulse_1.timer+1))

			# Reset envelope period
			self.pulse_1.envelope.decay_lvl = 15

			if self.pulse_1.enable:
				self.pulse_1.length_counter = length_conter_tbl[data>>3]

			self.pulse_1.envelope.start = 1

			# TODO: Reset pulse channel phase

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
			
		elif addr == 0x4007:	# Pulse 2 hi bit t
			self.pulse_2.timer = ((data&0b111)<<8) | (self.pulse_2.timer&0x000FF)
			self.pulse_2.sweep.current_period = self.pulse_2.timer
			self.pulse_2.freq = 1789773 / (16*(self.pulse_2.timer+1))

			# Reset envelope period
			self.pulse_2.envelope.decay_lvl = 15

			if self.pulse_2.enable:
				self.pulse_2.length_counter = length_conter_tbl[data>>3]

			self.pulse_2.envelope.start = 1
		
		elif addr == 0x400A:	# Triangle lo bit t
			self.triangle.timer = (self.triangle.timer&0x0FF00)|data

		elif addr == 0x400B:	# Triangle hi bit t
			self.triangle.timer = ((data&0b111)<<8) | (self.triangle.timer&0x000FF)
			self.triangle.freq = 1789773 / (32*(self.triangle.timer+1))

		elif addr == 0x4015:	# Status register Enable/Disable channels
			self.pulse_1.enable = True if data & 0b01 else False
			self.pulse_2.enable = True if data & 0b10 else False
			self.triangle.enable = True if data & 0b100 else False
			# self.noise_channel.enable = True if (data & 0b1000) else False

			if not self.pulse_1.enable:
				self.pulse_1.length_counter = 0

			if not self.pulse_2.enable:
				self.pulse_2.length_counter = 0

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


	cdef uint8_t cpu_read(self, uint16_t addr):
		return 0x00
	
