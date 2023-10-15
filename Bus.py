import numpy as np

class Bus:
	def __init__(self):
		self.cpu = None
		self.ppu = None
		self.cartridge = None
		self.cpu_ram = np.zeros(2 * 1024, dtype=np.uint8)	# 2kB CPU RAM

		self.n_system_clock_counter = 0

		# Controller state
		self.controller = np.zeros(2, dtype=np.uint8)	# Keeps track of controller
		self.controller_state = np.zeros(2, dtype=np.uint8)	# Latches controlller state after CPU write

	def clock(self):
		self.ppu.clock()
		if self.n_system_clock_counter % 3 == 0:	# PPU clock 3x faster than CPU
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
			self.cpu_ram[addr & 0x07FF] = data		# Mirroring 2kB range to fill 8kB
		elif addr >= 0x2000 and addr <= 0x3FFF:		# Write to PPU
			self.ppu.cpu_write(addr & 0x0007, data)		# PPU has 8 registers
		elif addr == 0x4016 or addr == 0x4017:		# Controller Memory mapped IO
			self.controller_state[addr&0x0001] = self.controller[addr&0x0001]&0xFF

	def read(self, addr: np.uint16, bReadOnly: bool = False):
		data, valid_addr = self.cartridge.cpu_read(addr, bReadOnly)
		if valid_addr:	# Read from cartridge
			return data
		elif addr >= 0x0000 and addr <= 0x1FFF:		# Read from CPU RAM
			return self.cpu_ram[addr & 0x07FF]	# Maps 8kB addressable memory to 2kB of physical memory (mirroring)
		elif addr >= 0x2000 and addr <= 0x3FFF:		# Read from PPU
			return self.ppu.cpu_read(addr & 0x0007)	# PPU has 8 registers
		elif addr == 0x4016 or addr == 0x4017:		# Controller Memory mapped IO
			data = (self.controller_state[addr&0x0001] & 0x80) > 0
			self.controller_state[addr&0x0001] <<= 1
			return data


		return 0x00