# cython: cflags=-O3, boundscheck=False, wraparound=False, cdivision=True, nonecheck=False, initializedcheck=False, overflowcheck=False

import numpy as np
from .Bus import Bus

cdef class Cartridge:
	def __cinit__(self, uint8_t n_prog_chunks, uint8_t n_char_chunks):
		self.mapper = None

		self.n_prog_chunks = n_prog_chunks
		self.n_char_chunks = n_char_chunks
		self._v_prog_memory = np.zeros(n_prog_chunks * 16 * 1024, dtype=np.uint8)	# Single chunk 16kB
		self._v_char_memory = np.zeros(n_char_chunks * 8 * 1024, dtype=np.uint8)		# Single chunk 8kB

		"""
		Restructures 1D array to ndarray with the following inputs
			- 8kB chunk
			- 1st or 2nd 4kB chunk
			- tile row
			- tile column
			- lsb/msb i.e. plane
			- pixel values represented as 8 rows of bytes
		"""
		# self._v_char_memory_nd = self._v_char_memory.reshape((n_char_chunks, 2, 16, 16, 2, 8))

	@property
	def v_prog_memory(self):
		return self._v_prog_memory

	@property
	def v_char_memory(self):
		return self._v_char_memory

	# Connect Components
	def connect_mapper(self, mapper):
		self.mapper = mapper

	# PPU with internal bus communication
	cdef uint8_t ppu_write(self, uint16_t addr, uint8_t data):
		mapped_addr, valid_addr = self.mapper.ppu_map_write(addr, data)
		if valid_addr:	
			self._v_char_memory[mapped_addr] = data
			return True
		return False
		
	cdef uint8_t ppu_read(self, uint16_t addr, uint8_t* data):
		mapped_addr, valid_addr = self.mapper.ppu_map_read(addr)
		if valid_addr:
			data[0] = self._v_char_memory[mapped_addr]	
			return True

		data[0] = 0x00
		return False

	# CPU-Cart registers communication
	cpdef uint8_t cpu_write(self, uint16_t addr, uint8_t data):
		mapped_addr, valid_addr = self.mapper.cpu_map_write(addr, data)
		if valid_addr:
			self._v_prog_memory[mapped_addr] = data
			return True
		return False
		
	def cpu_read(self, addr):
		mapped_addr, valid_addr = self.mapper.cpu_map_read(addr)
		if valid_addr:
			return self._v_prog_memory[mapped_addr], True
		return 0x00, False
