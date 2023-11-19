import pyaudio
import numpy as np

"""
The implementation of audio used here means buffer size sets 
the limit on the minimal frequency that can be played since
the buffer must be able to fit atleast a single period. The 
minimial frequency is given by
					
					f_min = fs / buffer_size

"""
buffer_size = 200

p = pyaudio.PyAudio()

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

		if len(self.buffer) < frame_count:
			self.buffer = np.append(self.buffer, self.wave)

		samples_to_play = np.copy(self.buffer[:frame_count]) * self.enable
		self.buffer = self.buffer[frame_count:]

		return (samples_to_play.tobytes(), pyaudio.paContinue)

	def update_wave(self):	# Intended to be overwritten in derived class
		self.param_changed = False

	@property
	def enable(self):
		return self._enable

	@enable.setter
	def enable(self, v):
		if self._enable != v:
			self._enable = v

	@property
	def freq(self):
		return self._freq

	@freq.setter
	def freq(self, v):
		if self._freq != v:
			self._freq = v
			self.param_changed = True

	def update_wave(self):
		self.param_changed = False

cdef class PulseWave(Channel):
	def __cinit__(self):
		...

	@property
	def dc(self):
		return self._dc

	@dc.setter
	def dc(self, v):
		if self._dc != v:
			self._dc = v
			self.param_changed = True

	@property
	def v(self):
		return self._v

	@v.setter
	def v(self, vv):
		if self._v != vv:
			self._v = vv
			self.param_changed = True

	def update_wave(self):
		cycles = int(buffer_size/self.fs*self.freq)+1
		num_samples = int(np.round(self.fs / self.freq))
		t = np.arange(num_samples) / self.fs

		# TODO: Temporary
		signal = np.zeros(num_samples)
		signal[:int(num_samples * self.dc)] = .1 * self.v / 15

		self.wave = np.tile(signal.astype(np.float32), cycles)

		self.param_changed = False

cdef class APU:
	def __cinit__(self):
		self.pulse_1 = PulseWave()
		self.pulse_2 = PulseWave()

	cdef void clock(self):
		...

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
			self.pulse_1.v = data&0x0F

		elif addr == 0x4002:	# Pulse 1 lo bit t
			self.pulse_1.timer = (self.pulse_1.timer&0x0FF00)|data

		elif addr == 0x4003:	# Pulse 1 hi bit t
			self.pulse_1.timer = ((data&0b111)<<8) | (self.pulse_1.timer&0x000FF)
			self.pulse_1.freq = 1.789773*1e6 / (16*(self.pulse_1.timer+1))

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
			self.pulse_2.v = data&0x0F

		elif addr == 0x4006:	# Pulse 2 lo bit t
			self.pulse_2.timer = (self.pulse_2.timer&0x0FF00)|data
			
		elif addr == 0x4007:	# Pulse 2 hi bit t
			self.pulse_2.timer = ((data&0b111)<<8) | (self.pulse_2.timer&0x000FF)
			self.pulse_2.freq = 1789773 / (16*(self.pulse_2.timer+1))
			
		elif addr == 0x4015:	# Status register Enable/Disable channels
			self.pulse_1.enable = True if data & 0b01 else False
			self.pulse_2.enable = True if data & 0b10 else False
			# self.triangle_channel.enable = True if data & 0b100 else False
			# self.noise_channel.enable = True if (data & 0b1000) else False


	cdef uint8_t cpu_read(self, uint16_t addr):
		return 0x00
	
