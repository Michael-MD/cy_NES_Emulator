import pygame


cdef class Channel:
	def __cinit__(self, channel_id):
		self.channel_id = channel_id
		self.channel = pygame.mixer.Channel(channel_id)

cdef class PulseWave(Channel):
	def __cinit__(self, channel_id):
		super().__init__(channel_id)

cdef class APU:
	def __cinit__(self):
		self.fs = 44100

		pygame.mixer.init(self.fs, -16, 1, 512, allowedchanges=pygame.AUDIO_ALLOW_FREQUENCY_CHANGE)
		pygame.mixer.set_num_channels(5)	# Create 5 channels

		self.pulse_1 = PulseWave(0)
		self.pulse_2 = PulseWave(1)

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
	
