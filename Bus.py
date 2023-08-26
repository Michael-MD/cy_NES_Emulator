import numpy as np

class Bus:
	def __init__(self):
		self.cpu = None
		self.ppu = None
		self.cartridge = None
		self.cpu_ram = np.zeros(2 * 1024, dtype=np.uint8)	# 2kB CPU RAM

		self.n_system_clock_counter = 0

	def clock(self):
		if self.n_system_clock_counter % 3 == 0:	# PPU clock 3x faster than CPU
			self.cpu.clock()
		self.ppu.clock()

		self.n_system_clock_counter+=1

	# Connect Components
	def connect_cpu(self, cpu):
		self.cpu = cpu
		self.cpu.connect_bus(self)

	def connect_ppu(self, ppu):
		self.ppu = ppu
		self.ppu.connect_bus(self)

	def connect_cartridge(self, cartridge):
		self.cartridge = cartridge
		self.cartridge.connect_bus(self)

	def write(self, addr: np.uint16, data: np.uint8):
		if self.cartridge.cpu_write(addr, data):		# Write to cartridge
			"""
			Since different mappers may accept and regect different 
			memmory ranges. We have the read and write return if the
			value specified is in the cartridge addressable range.
			"""
			...
		elif addr >= 0x0000 and addr <= 0x1FFF:		# Write to CPU RAM
			self.cpu_ram[addr & 0x1FFF] = data
		elif addr >= 0x2000 and addr <= 0x3FFF:		# Write to PPU
			self.ppu.cpu_write(addr & 0x0007, data)		# PPU has 8 registers

	def read(self, addr: np.uint16, bReadOnly: bool = False):
		data, valid_addr = self.cartridge.cpu_read(addr, bReadOnly)
		if valid_addr:	# Read from cartridge
			return data
		elif addr >= 0x0000 and addr <= 0x1FFF:		# Read from CPU RAM
			return self.cpu_ram[addr & 0x07FF]	# Maps 8kB addressable memory to 2kB of physical memory (mirroring)
		elif addr >= 0x2000 and addr <= 0x3FFF:		# Read from PPU
			return self.ppu.cpu_read(addr & 0x0007, bReadOnly)	# PPU has 8 registers
		return 0x00