import os
import numpy as np
import pygame

from .CPU6502 import CPU6502
from .Bus import Bus
from .Cartridge import Cartridge
from .PPU import PPU
from .Mappers import Mapper000

cdef extern from "stdint.h":
	ctypedef unsigned char uint8_t
	ctypedef unsigned short uint16_t

cdef class NES:
	"""
	Description of iNES file formate: https://www.nesdev.org/wiki/INES#Flags_7
	"""
	cdef object rom_name
	cdef object file_ext

	cdef uint8_t n_mapper_ID

	cdef object cart
	cdef object bus
	cdef object cpu
	cdef object ppu

	cdef object screen
	cdef object _clock

	def __cinit__(self, rom_directory):
		self.rom_name, self.file_ext = os.path.splitext(rom_directory)
		
		# Check file format
		if self.file_ext != '.nes':
			raise ValueError('ines file format only (.nes).')

		# Read .nes file
		with open(rom_directory, "rb") as rom_file:
			rom = np.fromfile(rom_file, dtype=np.uint8)

			n_prog_chunks = rom[4]	# specified in 16kB chunks
			n_char_chunks = rom[5]	# specified in 8kB chunks

			flag_6 = rom[6]
			mirroring = flag_6 & (1<<0) 	# 0: horizontal, 1: vertical
			persistent_memory = flag_6 & (1<<1)
			byte_trainer = flag_6 & (1<<2)
			four_screen_vram = flag_6 & (1<<3)
			n_mapper_ID_lo = flag_6 >> 4

			flag_7 = rom[7]
			vs_unisystem = flag_7 & (1<<0)
			play_choice = flag_7 & (1<<1)
			nes_2_format = flag_7 & (1<<2)
			n_mapper_ID_hi = flag_7 >> 4

			self.n_mapper_ID = n_mapper_ID_hi | n_mapper_ID_lo

			prog_ram_size = flag_8 = rom[8]

			flag_9 = rom[9]
			tv_system = flag_9 & 1	# (0: NTSC; 1: PAL)

		# Instantiate cartridge class and load with program and palette data
		self.cart = Cartridge(n_prog_chunks, n_char_chunks)

		if self.n_mapper_ID == 0:
			mapper = Mapper000
		else:
			raise Exception('Unsupported mapper.')
		
		self.cart.connect_mapper(
			mapper(n_prog_chunks, n_char_chunks)
		)

		offset = 0
		if byte_trainer:
			offset = 512

		v_prog_memory_start = 15 + offset + 1
		v_prog_memory_end = v_prog_memory_start + (n_prog_chunks*16*1024)
		v_char_memory_start = v_prog_memory_end
		v_char_memory_end = v_char_memory_start + (n_char_chunks*8*1024)
		self.cart.v_prog_memory = rom[v_prog_memory_start:v_prog_memory_end]
		self.cart.v_char_memory = rom[v_char_memory_start:v_char_memory_end]

		# Set up NES system components
		self.bus = Bus()
		self.cpu = CPU6502()
		self.ppu = PPU()

		self.ppu.v_mirroring = mirroring

		self.bus.connect_cpu(self.cpu)
		self.bus.connect_ppu(self.ppu)
		self.bus.connect_cartridge(self.cart)
		self.ppu.connect_cartridge(self.cart)

		# Reset everything
		self.reset()
		self.cpu.cycles = 0 	# Override instruction delay

	@property
	def ppu(self):
		return self.ppu

	@property
	def bus(self):
		return self.bus

	def reset(self):
		self.cpu.reset()

	cdef void clock(self):
		self.bus.clock()

	cdef void clock_system(self, N=3):
		# Clock CPU one instruction
		cdef int i
		for i in range(N):
			self.clock()

	cpdef void run(self):
		pygame.init()
		self.screen = pygame.display.set_mode((128, 128), pygame.RESIZABLE)
		self.screen.fill((0,0,0))
		self._clock = pygame.time.Clock()

		while True:
			self.clock_system(100_000)

			image_surface = pygame.surfarray.make_surface(self.ppu.screen)
			self.screen.blit(image_surface, (0, 0))

			self.bus.controller = 0
			for event in pygame.event.get():
				keys = pygame.key.get_pressed()
				if event.type == pygame.QUIT:
					pygame.quit()
					exit()

				elif event.type == pygame.KEYDOWN:
					if event.key == pygame.K_UP:
						self.bus.controller = self.bus.controller | 0x08
					if event.key == pygame.K_LEFT:
						self.bus.controller = self.bus.controller | 0x02
					if event.key == pygame.K_DOWN:
						self.bus.controller = self.bus.controller | 0x04
					if event.key == pygame.K_RIGHT:
						self.bus.controller = self.bus.controller | 0x01
					if event.key == pygame.K_s:
						self.bus.controller = self.bus.controller | 0x10
					if event.key == pygame.K_a:
						self.bus.controller = self.bus.controller | 0x20
					if event.key == pygame.K_z:
						self.bus.controller = self.bus.controller | 0x40
					if event.key == pygame.K_x:
						self.bus.controller = self.bus.controller | 0x80

			self._clock.tick(60)
			pygame.display.update()