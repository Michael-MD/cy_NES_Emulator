import numpy as np
from .Bus import Bus

class Cartridge:
	"""
	Pattern (char) memory which contains sprite informations is located 
	at address range 0x0000 to 0x1FFF.
	"""
	def __init__(self, n_prog_chunks, n_char_chunks):
		self.bus = None
		self.ppu = None
		self.mapper = None

		self.n_prog_chunks = n_prog_chunks
		self.n_char_chunks = n_char_chunks
		self.v_prog_memory = np.zeros(n_prog_chunks * 16 * 1024, dtype=np.uint8)	# Single chunk 16kB
		self.v_char_memory = np.zeros(n_char_chunks * 8 * 1024, dtype=np.uint8)		# Single chunk 8kB

		"""
		Restructures 1D array to ndarray with the following inputs
			- 8kB chunk
			- 1st or 2nd 4kB chunk
			- tile row
			- tile column
			- lsb/msb i.e. plane
			- pixel values represented as 8 rows of bytes
		"""
		self.v_char_memory_nd = self.v_char_memory.reshape((n_char_chunks, 2, 16, 16, 2, 8))


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
			return self.v_char_memory[mapped_addr], True
		return 0x00, False

	# CPU-Cart registers communication
	def cpu_write(self, addr, data):
		mapped_addr, valid_addr = self.mapper.cpu_map_write(addr, data)
		if valid_addr:
			self.v_prog_memory[mapped_addr] = data
			return True
		return False
		
	def cpu_read(self, addr, bReadOnly: bool = False):
		mapped_addr, valid_addr = self.mapper.cpu_map_read(addr)
		if valid_addr:
			return self.v_prog_memory[mapped_addr], True
		return 0x00, False
