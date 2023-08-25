import numpy as np
from .Bus import Bus

class Cartridge:
	def __init__(self, n_prog_chunks, n_char_chunks):
		self.bus = None
		self.ppu = None
		self.mapper = None

		self.n_prog_chunks = n_prog_chunks
		self.n_char_chunks = n_char_chunks
		self.v_prog_memory = np.zeros(n_prog_chunks * 16 * 1024, dtype=np.uint8)	# Single chunkchunk 16kB
		self.v_char_memory = np.zeros(n_char_chunks * 8 * 1024, dtype=np.uint8)	# Single bank unk 8kB

	# Connect Components
	def connect_bus(self, bus):
		self.bus = bus

	def connect_ppu(self, ppu):
		self.ppu = ppu

	def connect_mapper(self, mapper):
		self.mapper = mapper

	# PPU with internal bus communication
	def ppu_write(self, addr, data):
		if (mapped_addr:=self.mapper.ppu_map_write(addr, data)):
			self.v_char_memory[mapped_addr] = data
			return True
		return False
		
	def ppu_read(self, addr, bReadOnly: bool = False):
		if (mapped_addr:=self.mapper.ppu_map_read(addr)):
			return self.v_char_memory[mapped_addr]
		return False

	# CPU-PPU registers communication 
	def cpu_write(self, addr, data):
		if (mapped_addr:=self.mapper.cpu_map_write(addr)):
			self.v_prog_memory[mapped_addr] = data
			return True
		return False
		
	def cpu_read(self, addr, bReadOnly: bool = False):
		if (mapped_addr:=self.mapper.cpu_map_read(addr)):
			return self.v_prog_memory[mapped_addr]
		return False
