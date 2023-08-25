import numpy as np

class Bus:
	def __init__(self):
		self.cpu = None
		self.ppu = None
		self.cartridge = None
		self.cpu_ram = np.zeros(2 * 1024, dtype=np.uint8)	# 2kB CPU RAM

	def reset():
		cpu.reset()

	def clock(): ...

	# Connect Components
	def connect_cpu(self, cpu):
		self.cpu = cpu
		cpu.connect_bus(self)

	def connect_ppu(self, ppu):
		self.ppu = ppu
		ppu.connect_bus(self)

	def connect_cartridge(self, cartridge):
		self.cartridge = cartridge
		cartridge.connect_bus(self)

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
			ppu.cpu_write(addr & 0x0007, data)		# PPU has 8 registers

	def read(self, addr: np.uint16, bReadOnly: bool = False):
		if self.cartridge.cpu_read(addr, bReadOnly):	# Read from cartridge
			...
		elif addr >= 0x0000 and addr <= 0x1FFF:		# Read from CPU RAM
			return self.cpu_ram[addr & 0x1FFF]	# Maps 8kB addressable memory to 2kB of physical memory (mirroring)
		elif addr >= 0x2000 and addr <= 0x3FFF:		# Read from PPU
			return ppu.cpu_read(addr & 0x0007, bReadOnly)	# PPU has 8 registers
		return 0x00