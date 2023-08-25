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

	def write(self, addr, data):
		...

	def read(self, addr, bReadOnly: bool = False):
		return 0x00