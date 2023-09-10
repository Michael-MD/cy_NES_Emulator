import numpy as np
import ctypes
c_uint8 = ctypes.c_uint8
from .available_colours import pal_screen

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

		self._address_latch = 0x00		# Keeps track of which address byte is being written
		self._ppu_data_buffer = 0x00	# Stores data for clock cycle delay before it is returned to cpu
		self.nmi = False

		# Components attached to PPU bus (pattern memory is implicit since it is contained in the cartridge)
		self.name_table = np.zeros([2,1024], dtype=np.uint8)	# 2 1kB name tables
		self.palettes = np.zeros(32, dtype=np.uint8)	# 32 bytes
		self.colours = pal_screen

		# Stores sprite pattern information
		self._sprites_pixels = None
		self._sprites_colours = None

	def clock(self):
		if self.scan_line == -1 and self.cycle == 1:	# Unset vertical blank at top of page
			self.status.b.vertical_blank = 0

		if self.scan_line == 241 and self.cycle == 1:	# Set vertical blank at scan line below bottom of page and set nmi
			self.status.b.vertical_blank = 1
			if self.ctrl.b.enable_nmi:
				self.nmi = True

		self.cycle+=1 									# Scan across screen
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
		addr &= 0x3FFF	# Ensure in addressable range of PPU bus
		data, valid_addr = self.cartridge.ppu_read(addr, bReadOnly)
		if valid_addr:	# Read from cartridge (Pattern Memory)
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
			self.ctrl.reg = data
		elif addr == 0x0001:		# Mask
			self.mask.reg = data
		elif addr == 0x0002:		# Status
			...	# Cannot be written to
		elif addr == 0x0003:		# OAM Address
			...
		elif addr == 0x0004:		# OAM Data
			...
		elif addr == 0x0005:		# Scroll
			...
		elif addr == 0x0006:		# PPU Address
			if self._address_latch == 0:
				self.ppu_addr = (self.ppu_addr&0x00FF) | (data<<8)	# Writing high byte
				self._address_latch = 1
			else:
				self.ppu_addr = (self.ppu_addr&0xFF00) | data	# Writing low byte
				self._address_latch = 0
		elif addr == 0x0007:		# PPU Data
			self.ppu_write(self.ppu_addr, data)
			self.ppu_addr+=1

	def cpu_read(self, addr, bReadOnly: bool = False):
		data = 0x00
		if addr == 0x0000:			# Control
			...	# Not readable
		elif addr == 0x0001:		# Mask
			... # Not readable
		elif addr == 0x0002:		# Status
			# self.status.b.vertical_blank = 1 	# Uncomment for quick testing
			# First 5 status registers unused usually and will contain noise or buffer data 
			data = (self.status.reg&0xE0) | (self._ppu_data_buffer&0x1F)
			self.status.b.vertical_blank = 0
			self._address_latch = 0
		elif addr == 0x0003:		# OAM Address
			...
		elif addr == 0x0004:		# OAM Data
			...
		elif addr == 0x0005:		# Scroll
			...
		elif addr == 0x0006:		# PPU Address
			return 0x00 	# Cannot read address register
		elif addr == 0x0007:		# PPU Data
			data = self._ppu_data_buffer
			self._ppu_data_buffer = self.ppu_read(self.ppu_addr)

			if self.ppu_addr > 0x3F00:					# Palette uses combinatorial logic which can output data in same clock cycle
				data = self._ppu_data_buffer

			self.ppu_addr+=1
		return data


	# Gets 2-bit color for single tile from pattern (char) memory
	def make_sprite_pixels(self):
		"""
		Pattern memory is split conceptually into two 4kB regions to make the full 8kB.
		It stores a 2 bit number specifying the colour from some palette.
		The first 4kB is usually sprites while the second is backgrounds.
		
		Description:
		There are 16x16 tiles, each tile contains 8x8 pixels. Each pixel is 2 bits => each tile
		is 8x8x2 = 128b = 16bytes. Each 4kB range contains 16x16 tiles, each 16 bytes = 4kB.
		The two bits are used to index a colour in the palette.

		Physical Storage:
		The LSB and MSB (B for stands for bit here, not byte) are stored in "planes". The LSB plane
		is stored first follows by MSB plane. Each row is one byte and the tile is unrapped in bytes.
	
		i: the first or second 4kB region 
		palette: the row from the palettes table

		To be proper we should make calls from the PPU. However I will ignore this since using numpy arrays 
		to implement a half adder is clearer and very quick. 

		This will need changing when we implement more complex mappers other than 000.
		"""
		# Populate palette memory
		# if self.cartridge.mapper == Mapper_000:
		lo, hi = self.cartridge.v_char_memory_nd[...,0,:], self.cartridge.v_char_memory_nd[...,1,:]
		# Combines lower and upper bit to make pixel value
		self._sprites_pixels = np.empty([*lo.shape, 8], dtype=np.uint8)	# NumPy doesn't have 2bit so 1byte is used i.e. 1byte atomicity

		"""
		Make table of palette offsets (pixels)
		1. Isolates ith bit
		2. Shifts high bit to the left by 1 further
		3. Ors the low and high bits
		4. Shifts result so that low bit is in the 0th position
		"""
		for i in range(7):
			self._sprites_pixels[...,(7-i)] = np.bitwise_or(((hi&(1<<i))<<1), (lo&(1<<i)))>>i
		self._sprites_pixels[...,0] = np.bitwise_or(((hi&(1<<7))), (lo&(1<<7))>>1)>>6

	def make_sprite_colours(self, palette):
		"""
		We query the palette table to convert the 2-bit pixel values to 
		RBG colours to be shown on the display.

		The palette memory range is 0x3FFF to 0x3FFF. The palette contains the index of the colour in the display colours
		array.
		The first byte is the background.
		Each 4-bytes thereafter are a palette. The pixel value determines which of the 4-bytes to use.
		A colour is obtained by: 4*(palette #) + pixel.
		The first 4 palettes are usually for bg and the next 4 are fg

		The palette table is arranged as follows:
		Palette #		0x3F00		bkgd
		0				0x3F01		C1		C2		C3		bkgd
		1				0x3F05		C1		C2		C3		bkgd
		2				0x3F09		C1		C2		C3		bkgd
		3				0x3F0D		C1		C2		C3		bkgd
		4				0x3F11		C1		C2		C3		bkgd
		5				0x3F15		C1		C2		C3		bkgd
		6				0x3F19		C1		C2		C3		bkgd
		7				0x3F1D		C1		C2		C3		bkgd
		"""

		if self._sprites_pixels is None:
			self.make_sprite_pixels()

		# Make table of colours
		self._sprites_colours = np.empty([*self._sprites_pixels.shape, 3], dtype=np.uint8)

		for ind, pixel in np.ndenumerate(self._sprites_pixels):
			self._sprites_colours[ind] = self.colours[self.ppu_read(0x3F00 + (palette<<2) + pixel)]