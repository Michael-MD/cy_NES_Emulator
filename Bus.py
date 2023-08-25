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
		if addr >= 0x0000 and addr <= 0x1FFF:		# Write to CPU RAM
			self.cpu_ram[addr & 0x1FFF] = data
		elif addr >= 0x2000 and addr <= 0x3FFF:		# Write to PPU
			ppu.write(addr, data)

	def read(self, addr: np.uint16, bReadOnly: bool = False):
		if addr >= 0x0000 and addr <= 0x1FFF:		# Read from CPU RAM
			return self.cpu_ram[addr & 0x1FFF]	# Maps 8kB addressable memory to 2kB of physical memory (mirroring)
		elif addr >= 0x2000 and addr <= 0x3FFF:		# Read from PPU
			return ppu.read(addr, data, bReadOnly)
		return 0x00