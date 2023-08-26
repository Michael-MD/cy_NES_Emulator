import os
import numpy as np
from .CPU6502 import CPU6502
from .Bus import Bus
from .Cartridge import Cartridge
from .PPU import PPU
from .Mappers import Mapper000

import pdb

class NES:
	def __init__(self, rom_directory):
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
			mirroring = flag_6 & (1<<0)
			persistent_memory = flag_6 & (1<<1)
			byte_trainer = flag_6 & (1<<2)
			four_screen_vram = flag_6 & (1<<3)
			n_mapper_ID_lo = flag_6 >> 4

			flag_7 = rom[7]
			vs_unisystem = flag_7 & (1<<0)
			play_choice = flag_7 & (1<<1)
			nes_2_format = flag_7 & (1<<2)
			four_screen_vram = flag_7 & (1<<3)
			n_mapper_ID_hi = flag_7 & 0xFF00

			self.n_mapper_ID = n_mapper_ID_hi | n_mapper_ID_lo

			prog_ram_size = flag_8 = rom[8]

			flag_9 = rom[9]
			tv_system = flag_9 & 1	# (0: NTSC; 1: PAL)

		# Instantiate cartridge class and load with program and palette data
		self.cart = Cartridge(n_prog_chunks, n_char_chunks)

		if self.n_mapper_ID == 0:
			mapper = Mapper000
		
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
		self.cpu = CPU6502(debugging_mode=True)
		self.ppu = PPU()

		self.bus.connect_cpu(self.cpu)
		self.bus.connect_ppu(self.ppu)
		self.bus.connect_cartridge(self.cart)
		self.ppu.connect_cartridge(self.cart)

		# Reset everything
		self.reset()

	def reset(self):
		self.cpu.reset()

	def clock(self):
		self.bus.clock()


