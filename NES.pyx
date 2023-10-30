# cython: cflags=-O3, boundscheck=False, wraparound=False, cdivision=True, nonecheck=False, initializedcheck=False, overflowcheck=False

import os
import numpy as np
import pygame

from .CPU6502 cimport CPU6502
from .Cartridge cimport Cartridge
from .Bus cimport Bus
from .PPU cimport PPU
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

	cdef Cartridge cart
	cdef Bus bus
	cdef CPU6502 cpu
	cdef PPU ppu

	cdef object screen
	cdef object _clock

	cdef int width
	cdef int height

	cdef int screen_width
	cdef int screen_height

	cdef int padding_left
	cdef int pading_top

	def __cinit__(self, rom_directory):
		self.rom_name, self.file_ext = os.path.splitext(rom_directory)
		
		# Check file format
		if self.file_ext != '.nes':
			raise ValueError('ines file format only (.nes).')

		# Read .nes file
		cdef uint8_t[:] rom
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
		self.cart.v_prog_memory[:] = rom[v_prog_memory_start:v_prog_memory_end]
		self.cart.v_char_memory[:] = rom[v_char_memory_start:v_char_memory_end]

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

	cdef void clock_system(self, N=3):
		self.bus.clock(N)
	
	def init_dimensions(self, screen_width, screen_height):
		scale_factor = min(screen_width / self.width, screen_height / self.height)

		# Calculate the scaled dimensions
		self.width = int(self.width * scale_factor)
		self.height = int(self.height * scale_factor)

		self.screen_width = screen_width
		self.screen_height = screen_height

		self.padding_left = self.screen_width//2 - self.width//2
		self.pading_top = self.screen_height//2 - self.height//2

	cpdef void run(self):
		pygame.init()
		self.screen = pygame.display.set_mode((255, 239), pygame.RESIZABLE)
		self.screen.fill((0,0,0))
		self._clock = pygame.time.Clock()
		self.width = 256
		self.height = 240

		self.screen_width = 256
		self.screen_height = 240

		self.padding_left = 0
		self.pading_top = 0

		a = np.zeros((256, 240, 3), dtype=np.uint8)
		image_surface = pygame.surfarray.make_surface(a)

		while True:
			self.clock_system(82_190)

			a[:] = self.ppu.screen[:]

			pygame.surfarray.blit_array(image_surface, a)  # Update the Surface
			self.screen.blit(pygame.transform.scale(image_surface, (self.width, self.height)), (self.padding_left, self.pading_top))
			pygame.display.update()

			self.bus.controller = 0
			
			keys = pygame.key.get_pressed()
			if keys[pygame.K_UP]:
				self.bus.controller |= 0x08
			if keys[pygame.K_LEFT]:
				self.bus.controller |= 0x02
			if keys[pygame.K_DOWN]:
				self.bus.controller |= 0x04
			if keys[pygame.K_RIGHT]:
				self.bus.controller |= 0x01
			if keys[pygame.K_s]:
				self.bus.controller |= 0x10
			if keys[pygame.K_a]:
				self.bus.controller |= 0x20
			if keys[pygame.K_z]:
				self.bus.controller |= 0x40
			if keys[pygame.K_x]:
				self.bus.controller |= 0x80

			for event in pygame.event.get():
				if event.type == pygame.QUIT:
					pygame.quit()
					exit()
				elif event.type == pygame.VIDEORESIZE:
					self.init_dimensions(*event.size)

			self._clock.tick(60)

			