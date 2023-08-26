import numpy as np

class PPU:
	"""
	The PPU has its own internal bus with three memories attached.
	The pattern (char) memory which is a part of the cartridge located at 
	0x0000 to 0x1xFFFF.
	Name table memory which stores layout is located in range 0x2000 to 0x3EFF.
	Palette memory which stores color information located in range 0x3F00 to 
	0x3FFF.
	"""
	def __init__(self):
		self.bus = None
		self.cartridge = None

		# Populating screen
		self.cycle = 0 	# Column
		self.scan_line = 0


		# Components attached to PPU bus
		self.name_table = np.zeros([2,1024], dtype=np.uint8)	# 2 1kB name tables
		self.palletes = np.zeros(32, dtype=np.uint8)	# 32 byte pallette

	def clock():
		self.cycle+=1
		if self.cycle >= 341:	# Finished column
			self.cycle = 0
			self.scan_line+=1

			if self.scan_line >= 261:	# Finished row
				self.scan_line = -1


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