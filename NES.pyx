# distutils: language = c++
# cython: cflags=-O3, boundscheck=False, wraparound=False, cdivision=True, nonecheck=False, initializedcheck=False, overflowcheck=False

import os
import numpy as np
import pygame

from .CPU6502 cimport CPU6502
from .Cartridge cimport Cartridge
from .Bus cimport Bus
from .PPU cimport PPU

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

		self.cart.connect_mapper(self.n_mapper_ID, n_prog_chunks, n_char_chunks)

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

		using_joystick = False
		# Joystick buttons, nesdev: https://www.nesdev.org/wiki/Standard_controller#Output_($4016/$4017_read)
		A = 0b1<<7
		B = 0b1<<6
		SELECT = 0b1<<5
		START = 0b1<<4
		UP = 0b1<<3
		DOWN = 0b1<<2
		LEFT = 0b1<<1
		RIGHT = 0b1<<0
		i = 0
		while True:
			self.clock_system(82_190)

			a[:] = self.ppu.screen[:]

			pygame.surfarray.blit_array(image_surface, a)  # Update the Surface
			self.screen.blit(pygame.transform.scale(image_surface, (self.width, self.height)), (self.padding_left, self.pading_top))
			pygame.display.update()

			self.bus.controller = 0
			
			if not using_joystick:
				keys = pygame.key.get_pressed()
				if keys[pygame.K_UP]:
					self.bus.controller |= UP
				if keys[pygame.K_LEFT]:
					self.bus.controller |= LEFT
				if keys[pygame.K_DOWN]:
					self.bus.controller |= DOWN
				if keys[pygame.K_RIGHT]:
					self.bus.controller |= RIGHT
				if keys[pygame.K_s]:
					self.bus.controller |= START
				if keys[pygame.K_a]:
					self.bus.controller |= SELECT
				if keys[pygame.K_z]:
					self.bus.controller |= B
				if keys[pygame.K_x]:
					self.bus.controller |= A

			if using_joystick:
				"""
                Joystick mapping:
                      ======_                               _=====_
                     / _____ \                             / _____ \
                   +.-'_____'-.---------------------------.-'_____'-.+
                  /   |     |  '.                       .'  |  _  |   \
                 / ___| /|\ |___ \                     / ___| /3\ |___ \
                / |     11      | ;  __           _   ; | _         _ | ;
                | | <-13   14-> | | |_6|         |7:> | ||2|       (1)| |
                | |___  12   ___| ;SELECT       START ; |___       ___| ;
                |\    | \|/ |    /  _     ___      _   \    | (0) |    /|
                | \   |_____|  .','" "', |___|  ,'" "', '.  |_____|  .' |
                |  '-.______.-' /       \ANALOG/       \  '-._____.-'   |
                |               |       |------|       |                |
                |              /\       /      \       /\               |
                |             /  '.___.'        '.___.'  \              |
                |            /                            \             |
                 \          /                              \           /
                  \________/                                \_________/
                ASCII art source: https://www.asciiart.eu/computers/game-consoles
				"""

				if joystick.get_button(0):
					self.bus.controller |= A
				if joystick.get_button(1):
					self.bus.controller |= B
				if joystick.get_button(6):
					self.bus.controller |= SELECT
				if joystick.get_button(7):
					self.bus.controller |= START

				hat = joystick.get_hat(0)
				if hat[1] == 1:
					self.bus.controller |= UP
				if hat[1] == -1:
					self.bus.controller |= DOWN
				if hat[0] == -1:
					self.bus.controller |= LEFT
				if hat[0] == 1:
					self.bus.controller |= RIGHT

			for event in pygame.event.get():
				if event.type == pygame.QUIT:
					pygame.quit()
					exit()
				elif event.type == pygame.VIDEORESIZE:
					self.init_dimensions(*event.size)
				elif event.type == pygame.JOYDEVICEADDED:
					pygame.joystick.init()
					joystick = pygame.joystick.Joystick(event.device_index)
					using_joystick = True
				elif event.type == pygame.JOYDEVICEREMOVED:
					joystick.quit()
					using_joystick = False

			self._clock.tick(60)

			i += 1
			if i%100:
				print(self._clock.get_fps())

			