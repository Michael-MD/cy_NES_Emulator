import numpy as np

class PPU:
	def __init__(self):
		self.bus = None
		self.cartridge = None

		# Components attached to PPU bus
		self.name_table = np.zeros([2,1024], dtype=np.uint8)	# 2 1kB name tables
		self.palletes = np.zeros(32, dtype=np.uint8)	# 32 byte pallette

	def clock(): ...

	# Connect Components
	def connect_cartridge(self, cartridge):
		self.cartridge = cartridge
		cartridge.connect_ppu(self)

	def connect_bus(self, bus):
		self.bus = bus

	# PPU with internal bus communication
	def ppu_write(self, addr, data):
		addr &= 0x3FFF	# Ensure in addressable range
		if self.cartridge.ppu_write(addr, data):		# Write to cartridge
			...

	def ppu_read(self, addr, bReadOnly: bool = False):
		addr &= 0x3FFF	# Ensure in addressable range
		if (data:=self.cartridge.ppu_read(addr, bReadOnly)):	# Read from cartridge
			return data

	# CPU to PPU registers communication 
	def cpu_write(self, addr, data):
		...

	def cpu_read(self, addr, bReadOnly: bool = False):
		...