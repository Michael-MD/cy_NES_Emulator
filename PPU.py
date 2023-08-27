import numpy as np
import ctypes
c_uint8 = ctypes.c_uint8

class StatusBits(ctypes.LittleEndianStructure):
    _fields_ = [
            ("unused", c_uint8, 5),
            ("sprite_overflow", c_uint8, 1),
            ("sprite_zero_hit", c_uint8, 1),
            ("vertical_blank", c_uint8, 1),
        ]

class MaskBits(ctypes.LittleEndianStructure):
    _fields_ = [
            ("grayscale", c_uint8, 1),
            ("render_background_left", c_uint8, 1),
            ("render_sprites_left", c_uint8, 1),
            ("render_background", c_uint8, 1),
            ("render_sprites", c_uint8, 1),
            ("enhance_red", c_uint8, 1),
            ("enhance_green", c_uint8, 1),
            ("enhance_blue", c_uint8, 1),
        ]

class CtrlBits(ctypes.LittleEndianStructure):
    _fields_ = [
            ("nametable_x", c_uint8, 1),
            ("nametable_y", c_uint8, 1),
            ("increment_mode", c_uint8, 1),
            ("pattern_sprite", c_uint8, 1),
            ("pattern_background", c_uint8, 1),
            ("sprite_size", c_uint8, 1),
            ("slave_mode", c_uint8, 1),		# Unused
            ("enable_nmi", c_uint8, 1),
        ]

class Status(ctypes.Union):
    _fields_ = [("b", StatusBits),
                ("reg", c_uint8)]

class Mask(ctypes.Union):
    _fields_ = [("b", MaskBits),
                ("reg", c_uint8)]

class Ctrl(ctypes.Union):
    _fields_ = [("b", CtrlBits),
                ("reg", c_uint8)]

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

		# Internal registers
		self.ctrl = Ctrl()
		self.mask = Mask()
		self.status = Status()
		self.scroll = 0x00
		self.ppu_addr = 0x0000	# 14 bits
		self.data = 0x00

		self._address_latch = 0x00

		# Components attached to PPU bus
		self.name_table = np.zeros([2,1024], dtype=np.uint8)	# 2 1kB name tables
		self.palettes = np.zeros(32, dtype=np.uint8)	# 32 bytes
		self.colours = np.zeros([64,3], dtype=np.uint8)	# 64 bytes of available colours

		# Stores sprite pattern information
		self._sprite = None

		# Available colour RGB values
		self.colours[0x00] = [84, 84, 84]
		self.colours[0x01] = [0, 30, 116]
		self.colours[0x02] = [8, 16, 144]
		self.colours[0x03] = [48, 0, 136]
		self.colours[0x04] = [68, 0, 100]
		self.colours[0x05] = [92, 0, 48]
		self.colours[0x06] = [84, 4, 0]
		self.colours[0x07] = [60, 24, 0]
		self.colours[0x08] = [32, 42, 0]
		self.colours[0x09] = [8, 58, 0]
		self.colours[0x0A] = [0, 64, 0]
		self.colours[0x0B] = [0, 60, 0]
		self.colours[0x0C] = [0, 50, 6]

		self.colours[0x10] = [152, 150, 152]
		self.colours[0x11] = [8, 76, 196]
		self.colours[0x12] = [48, 50, 236]
		self.colours[0x13] = [92, 30, 228]
		self.colours[0x14] = [136, 20, 176]
		self.colours[0x15] = [160, 20, 100]
		self.colours[0x16] = [152, 34, 32]
		self.colours[0x17] = [120, 60, 0]
		self.colours[0x18] = [84, 90, 0]
		self.colours[0x19] = [40, 114, 0]
		self.colours[0x1A] = [8, 124, 0]
		self.colours[0x1B] = [0, 118, 40]
		self.colours[0x1C] = [0, 102, 120]

		self.colours[0x20] = [236, 238, 236]
		self.colours[0x21] = [76, 154, 236]
		self.colours[0x22] = [120, 124, 236]
		self.colours[0x23] = [176, 98, 236]
		self.colours[0x24] = [228, 84, 236]
		self.colours[0x25] = [236, 88, 180]
		self.colours[0x26] = [236, 106, 100]
		self.colours[0x27] = [212, 136, 32]
		self.colours[0x28] = [160, 170, 0]
		self.colours[0x29] = [116, 196, 0]
		self.colours[0x2A] = [76, 208, 32]
		self.colours[0x2B] = [56, 204, 108]
		self.colours[0x2C] = [56, 180, 204]
		self.colours[0x2D] = [60, 60, 60]

		self.colours[0x30] = [236, 238, 236]
		self.colours[0x31] = [168, 204, 236]
		self.colours[0x32] = [188, 188, 236]
		self.colours[0x33] = [212, 178, 236]
		self.colours[0x34] = [236, 174, 236]
		self.colours[0x35] = [236, 174, 212]
		self.colours[0x36] = [236, 180, 176]
		self.colours[0x37] = [228, 196, 144]
		self.colours[0x38] = [204, 210, 120]
		self.colours[0x39] = [180, 222, 120]
		self.colours[0x3A] = [168, 226, 144]
		self.colours[0x3B] = [152, 226, 180]
		self.colours[0x3C] = [160, 214, 228]
		self.colours[0x3D] = [160, 162, 160]

	def clock(self):
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
		elif (addr >= 0x2000) and (addr <= 0x3EFF):		# Name table memory
			...
		elif (addr >= 0x3F00) and (addr <= 0x3FFF):		# Palette memory
			addr &= 0x1F	# address mod 32bytes
			# Mirrors fg and bg colour palettes
			if addr == 0x10: addr = 0x0000
			if addr == 0x14: addr = 0x0004
			if addr == 0x18: addr = 0x0008
			if addr == 0x1C: addr = 0x000C
			self.palettes[addr] = data

	def ppu_read(self, addr, bReadOnly: bool = False):
		addr &= 0x3FFF	# Ensure in addressable range
		data = 0x00
		if (data:=self.cartridge.ppu_read(addr, bReadOnly)):	# Read from cartridge (Pattern Memory)
			...
		elif (addr >= 0x2000) and (addr <= 0x3EFF):		# Name table memory
			...
		elif (addr >= 0x3F00) and (addr <= 0x3FFF):		# Palette memory
			addr &= 0x1F	# address mod 32bytes
			# Mirrors fg and bg colour palettes
			if addr == 0x10: addr = 0x0000
			if addr == 0x14: addr = 0x0004
			if addr == 0x18: addr = 0x0008
			if addr == 0x1C: addr = 0x000C
			data = self.palettes[addr]

		return data

	# CPU to PPU registers communication 
	def cpu_write(self, addr, data):
		if addr == 0x0000:			# Control
			control.reg = data
		elif addr == 0x0001:		# Mask
			mask.reg = data
		elif addr == 0x0002:		# Status
			...
		elif addr == 0x0003:		# OAM Address
			...
		elif addr == 0x0004:		# OAM Data
			...
		elif addr == 0x0005:		# Scroll
			...
		elif addr == 0x0006:		# PPU Address
			if self._address_latch == 0:
				self.ppu_addr = (self.ppu_addr&0xFF00) | data	# Writing low byte
				self._address_latch == 1
			else
				self.ppu_addr = (self.ppu_addr&0x00FF) | (data<<8)	# Writing low byte
				self._address_latch == 9
		elif addr == 0x0007:		# PPU Data
			self.ppu_write(addr, data)

	def cpu_read(self, addr, bReadOnly: bool = False):
		if addr == 0x0000:			# Control
			...
		elif addr == 0x0001:		# Mask
			...
		elif addr == 0x0002:		# Status
			...
		elif addr == 0x0003:		# OAM Address
			...
		elif addr == 0x0004:		# OAM Data
			...
		elif addr == 0x0005:		# Scroll
			...
		elif addr == 0x0006:		# PPU Address
			return 0x00
		elif addr == 0x0007:		# PPU Data
			...

	# Gets 2-bit color for single tile from pattern (char) memory
	def get_tile(self, i, palette):
		"""
		Pattern memory is split conceptually into two 4kB regions to make the full 8kB.
		It stores a 2 bit number specifying the colour from some palette.
		The first 4kB is usually sprites while the second is backgrounds.

		A single tile is 8x16 bits or 16 bytes and two tiles next to each other specify the two bit number.
		The first tile specifies the lower bit and the second specifies the upper bit.
		Each 4kB range contains 16x16 tiles, each 16 bytes = 4kB.

		i: the first or second 4kB region 
		palette: the row from the palettes table

		To be proper we should make calls to the PPU. However I will ignore this since using numpy arrays 
		to implement a half adder is clearer and very quick.
		"""
		# Pixel colors relative to some palette
		lo, hi = self.cartridge.v_char_memory_nd[0, i,..., 0], self.cartridge.v_char_memory_nd[0, i,..., 1]
		s = np.bitwise_xor(lo, hi)	# Sum
		c = np.bitwise_and(lo, hi)	# Carry

		self._sprite = np.empty([*s.shape, 8], dtype=np.uint8)
		# Isolates individual bit and shift to left before adding sum to carry
		self._sprite[...,0] = ((s&0x01)>>0) + ((c&0x01)>>0)
		self._sprite[...,1] = ((s&0x02)>>1) + ((c&0x02)>>1)
		self._sprite[...,2] = ((s&0x04)>>2) + ((c&0x04)>>2)
		self._sprite[...,3] = ((s&0x08)>>3) + ((c&0x08)>>3)
		self._sprite[...,4] = ((s&0x10)>>4) + ((c&0x10)>>4)
		self._sprite[...,5] = ((s&0x20)>>5) + ((c&0x20)>>5)
		self._sprite[...,6] = ((s&0x40)>>6) + ((c&0x40)>>6)
		self._sprite[...,7] = ((s&0x80)>>7) + ((c&0x80)>>7)

		"""
		Sprite with proper colours in corresponding slots
		Takes sprite pixel info and return same dimensional array with 
		colours.
		Palette addressable range starts from 0x3FFF, then we offset to the correct 
		palette by multiplying by 4, then we select the colour using pixel. 
		"""
		for ind, pixel in np.ndenumerate(self._sprite):
			self._sprite[ind] = self.colours[self.ppu_read(0x3FFF + (palette<<2) + pixel)]