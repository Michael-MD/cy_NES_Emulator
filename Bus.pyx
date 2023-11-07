# distutils: language = c++
# cython: cflags=-O3, boundscheck=False, wraparound=False, cdivision=True, nonecheck=False, initializedcheck=False, overflowcheck=False

cdef extern from "stdint.h":
	ctypedef unsigned char uint8_t
	ctypedef unsigned short uint16_t


import numpy as np

cdef class Bus:
	def __cinit__(self, int n_mapper_ID):
		self.cpu = None
		self.ppu = None
		self.cartridge = None
		self.cpu_ram = np.zeros(2 * 1024, dtype=np.uint8)	# 2kB CPU RAM
		self.n_mapper_ID = n_mapper_ID
		self.n_system_clock_counter = 0

		# Controller state
		self._controller = np.zeros(2, dtype=np.uint8)	# Keeps track of controller
		self._controller_state = np.zeros(2, dtype=np.uint8)	# Latches controlller state after CPU write

		self.dma_page = 0x00
		self.dma_addr = 0x00
		self.dma_data = 0x00
		self.dma_transfer = False
		self.dma_dummy = True

	@property
	def controller(self):
		return self._controller[0]

	@controller.setter
	def controller(self, v):
		self._controller[0] = v

	cpdef void clock(self, int N):
		cdef int i
		for i in range(N):
			self.ppu.clock()
			if self.n_system_clock_counter % 3 == 0:	# PPU clocked 3x faster than CPU
				if self.dma_transfer:
					if self.dma_dummy:
						if self.n_system_clock_counter % 2 == 1:
							self.dma_dummy = False
					else:
						if self.n_system_clock_counter % 2 == 0:
							self.dma_data = self.read((self.dma_page<<8)|self.dma_addr)
						else:
							self.ppu.OAM[self.dma_addr] = self.dma_data
							
							self.dma_addr+=1

							if self.dma_addr == 0x00:
								self.dma_transfer = False
								self.dma_dummy = True

				else:
					self.cpu.clock()

			if self.ppu.nmi:
				self.ppu.nmi = False
				self.cpu.set_nmi(1)

			self.n_system_clock_counter+=1

	# Connect Components
	def connect_cpu(self, cpu):
		self.cpu = cpu
		self.cpu.connect_bus(self)

	def connect_ppu(self, ppu):
		self.ppu = ppu
		# self.ppu.connect_bus(self)

	def connect_cartridge(self, cartridge):
		self.cartridge = cartridge

	cpdef void write(self, uint16_t addr, uint8_t data):
		if self.cartridge.cpu_write(addr, data):		# Write to cartridge
			"""
			Since different mappers may accept and reject different 
			memmory ranges. We have the read and write return if the
			value specified is in the cartridge addressable range.
			"""
		elif addr >= 0x0000 and addr <= 0x1FFF:		# Write to CPU RAM
			self.cpu_ram[addr & 0x07FF] = data		# Mirroring 2kB range to fill 8kB
		elif addr >= 0x2000 and addr <= 0x3FFF:		# Write to PPU
			self.ppu.cpu_write(addr & 0x0007, data)		# PPU has 8 registers
		elif addr == 0x4016 or addr == 0x4017:		# Controller Memory mapped IO
			self._controller_state[addr&0x0001] = self._controller[addr&0x0001]&0xFF
		elif addr == 0x4014:
			self.dma_page = data
			self.dma_addr = 0x00
			self.dma_transfer = True
			self.dma_dummy = True

	cpdef uint8_t read(self, uint16_t addr):
		data, valid_addr = self.cartridge.cpu_read(addr)
		if valid_addr:	# Read from cartridge
			return data
		elif addr >= 0x0000 and addr <= 0x1FFF:		# Read from CPU RAM
			return self.cpu_ram[addr & 0x07FF]	# Maps 8kB addressable memory to 2kB of physical memory (mirroring)
		elif addr >= 0x2000 and addr <= 0x3FFF:		# Read from PPU
			return self.ppu.cpu_read(addr & 0x0007)	# PPU has 8 registers
		elif addr == 0x4016 or addr == 0x4017:		# Controller Memory mapped IO
			data = (self._controller_state[addr&0x0001] & 0x80) > 0
			self._controller_state[addr&0x0001] <<= 1
			return data


		return 0x00