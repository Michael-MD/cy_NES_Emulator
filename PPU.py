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

	# PPU bus communication
	def ppu_write(self, addr, data):
		...

	def ppu_read(self, addr, bReadOnly: bool = False):
		return 0x00

	# CPU bus communication
	def cpu_write(self, addr, data):
		...

	def cpu_read(self, addr, bReadOnly: bool = False):
		return 0x00