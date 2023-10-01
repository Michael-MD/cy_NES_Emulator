import numpy as np
import ctypes
c_uint8 = ctypes.c_uint8
c_uint16 = ctypes.c_uint16
from .available_colours import pal_screen
from .Screen import *

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

class LoopyBits(ctypes.LittleEndianStructure):
    _fields_ = [
            ("coarse_x", c_uint8, 5),
            ("coarse_y", c_uint8, 5),
            ("nametable_x", c_uint8, 1),
            ("nametable_y", c_uint8, 1),
            ("fine_y", c_uint8, 3),
            ("unused", c_uint8, 1),
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

class Loopy(ctypes.Union):
    _fields_ = [("b", LoopyBits),
                ("reg", c_uint16)]

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
		self.screen = Screen()

		# Populating screen
		self.cycle = 0 	# Column
		self.scan_line = 0

		# Internal registers
		self.ctrl = Ctrl()
		self.mask = Mask()
		self.status = Status()
		self.loopy_t = Loopy()
		self.loopy_v = Loopy()
		self.fine_x = 0x00
		self.scroll = 0x00
		# self.ppu_addr = 0x0000	# 14 bits
		self.data = 0x00

		self._address_latch = 0x00		# Keeps track of which address byte is being written
		self._ppu_data_buffer = 0x00	# Stores data for clock cycle delay before it is returned to cpu
		self.nmi = False

		# Components attached to PPU bus (pattern memory is implicit since it is contained in the cartridge)
		self.nametable = np.zeros([2,1024], dtype=np.uint8)	# 2 1kB name tables
		self.palettes = np.zeros(32, dtype=np.uint8)	# 32 bytes
		self.colours = pal_screen

		# Stores sprite pattern information
		self._sprites_pixels = None
		self._sprites_colours = None

		self.v_mirroring = None

		# Temporarily store information about next namespace tile to be rendered.
		self._bg_next_tile_id = 0x00
		self._bg_next_tile_attr = 0x00
		self._bg_next_tile_lsb = 0
		self._bg_next_tile_msb = 0

		# Shift registers which store pixels to be rendered
		self._bg_shifter_pattern_lo = 0x0000	# Low bit of pallette offset i.e. pixel color
		self._bg_shifter_pattern_hi = 0x0000	# High bit of pallette offset i.e. pixel color
		self._bg_shifter_attr_lo = 0x0000		# Low bit of pallette to use
		self._bg_shifter_attr_hi = 0x0000		# High bit of pallette to use


	def clock(self):
		# https://www.nesdev.org/w/images/default/4/4f/Ppu.svg
		
		if self.scan_line >= -1 and self.scan_line < 240:	# Visible region of screen
			if self.scan_line == -1 and self.cycle == 1:	# Unset vertical blank at top of page
				self.status.b.vertical_blank = 0

			# Prepare next sprite loop, first condition is while screen is being renderd, second is before jumping to top of screen
			if (self.cycle >=2 and self.cycle < 258) or (self.cycle >=321 and self.cycle < 338):
				# Shift registers every cycle
				self._bg_shifter_pattern_lo <<= 1
				self._bg_shifter_pattern_hi <<= 1
				self._bg_shifter_attr_lo <<= 1
				self._bg_shifter_attr_hi <<= 1

				if (self.cycle-1)%8 == 0:	# Read background tile byte from nametable
					# Place tile data into shift registers for rendering onto screen
					self._bg_shifter_pattern_lo = (self._bg_shifter_pattern_lo&0xFF00) | self._bg_next_tile_lsb
					self._bg_shifter_pattern_hi = (self._bg_shifter_pattern_hi&0xFF00) | self._bg_next_tile_msb
					self._bg_shifter_attr_lo = (self._bg_shifter_attr_lo&0xFF00) | (0xFF if self._bg_next_tile_attr&0b01 else 0x00)
					self._bg_shifter_attr_hi = (self._bg_shifter_attr_hi&0xFF00) | (0xFF if self._bg_next_tile_attr&0b10 else 0x00)

					"""
					The loop v register keeps track of which part of the nametable is being shown on screen
					and the pixel currently being rendered. This is stored in reference coarse y and x and fine y,
					fine x is stored seperately. To find the offset into the name we:
					 - offset by the nametable and then
					 - offset to current tile.
					 0x0FFF is 12 bits which is what is required to offset into one of 4 nametables (2bits) and within
					 a 32x30 grid (10bits). Total: 12bits. The remaining bits are unused.
					 We then offset into the range the nametables are located on the bus. 
					"""
					self._bg_next_tile_id = self.ppu_read(0x2000|self.loopy_v.reg&0x0FFF)	

				elif (self.cycle-1)%8 == 2:	# Prepare attribute
					"""
					To get the correct attribute byte we must specify
						- Which nametable we are located. Since we have 4 conceptual namestables we require 2
						bits to specify this. Each nametable is 32x32=1kB which requires 10bits to represent.
						So the 11th and 12th bytes will specify which 1kB region we reside i.e. which nametable.

						- Each nametable is divided into an 8x8 grid, since coase x and y cound nametable tiles i.e. 32x30,
						if we ground the tiles into groups of 8x8, then the index can be obtained from the upper 3 bits of
						coarse x and y.

						- The nametables start at address 0x2000 on the bus so we offset the address by 0x2000.
						Additionally, the attribute memory starts at 0x03C0 so we offset by that amount also within 
						the selected nametable.

					The final address should look as follows:
					_ _ _ _   _ _ _ _   _ _ _ _   _ _ _ _
					|         | |           | |   | | | |
					|         | |           | |   | +-+-+------- coarse x
					|         | |           +-+---+------------- coarse y
					|         | +------------------------------- nametable x
					|         +--------------------------------- nametable y     
					+------------------------------------------- Beginning of nametable space on bus
					The offset for the beginning of the attribute memory uses the remaining spots in three least
					significant nibbles.
					"""
					self._bg_next_tile_attr = self.ppu_read((0x2000 + 0x03C0)	
																| self.loopy_v.b.nametable_y << 11
																| self.loopy_v.b.nametable_x << 10
																| (self.loopy_v.b.coarse_y>>2) << 3
																| (self.loopy_v.b.coarse_x>>2) << 0
															)

					"""
					Each byte in the attribute memory specifies the palette for a group of 2x2 tiles in the 4x4 group,
					of which recall there are 8x8. Since a palette only requires 2bits. The 8 bytes are divided into four parts,
					each is designated 2x2 tiles. The least significant bits of coarse x and y we neglected before can now be used
					to find out which 2x2 tile region of the 4x4 tile region we are in.
					The bits are allocates as follows.
					Top left: 0x03
					Top right: 0x0C
					Bottom left: 0x30
					Bottom right: 0xC0

					The code below selects the palette by ensuring the bottom 2 bits are the desired palette.
					The first if shifts to ensure the correct row is in the bottom 4 bits and the second if statement
					selects the correct column.

					TODO: elaborate in comment about tile attribute
					"""

					if self.loopy_v.b.coarse_y&0x2: # Bottom row if > 0
						self._bg_next_tile_attr >>= 4
					if self.loopy_v.b.coarse_x&0x2: # Right column if > 0
						self._bg_next_tile_attr >>= 2
					self._bg_next_tile_attr&=0x03

				elif (self.cycle-1)%8 == 4:	# Prepare pixel lsb
					"""
					Up to here we should have the address of the next sprite and the pallete to render it with.
					All we have left to do is get the pixel value which is the offset into the chosen palette.
					Usually during these two cycles and the next, the ppu will query the bus to 
					get the sprite lsb and msb from the patternn table to obtain the appropriate
					offset into the chosen palette. However, we take a high level approach here
					where we directly obtain the lsb and msb using the data structure we have made.
					"""
					self._bg_next_tile_lsb = self.cartridge.v_char_memory_nd[	
														0, 
														self.ctrl.b.pattern_background,
														(self._bg_next_tile_id&0x38)>>3,
														self._bg_next_tile_id&0x07,
														0,
														self.loopy_v.b.fine_y 
													]
					
				elif (self.cycle-1)%8 == 6:	# Prepare pixel msb
					self._bg_next_tile_msb = self.cartridge.v_char_memory_nd[	
														0, 
														self.ctrl.b.pattern_background,
														(self._bg_next_tile_id&0x38)>>3,
														self._bg_next_tile_id&0x07,
														1,
														self.loopy_v.b.fine_y 
													]

				elif (self.cycle-1)%8 == 7:	# Increment tile pointer
					"""
					Information about sprite has been loaded so we can increment the course x to begin working on the 
					next sprite.
					"""
					if self.mask.b.render_background or self.mask.b.render_sprites:
						if self.loopy_v.b.coarse_x == 31:	# Boundary of nametable reached
							self.loopy_v.b.coarse_x = 0
							self.loopy_v.b.nametable_x^=1 # Negate nametable bit to increment nametable
						else:
							self.loopy_v.b.coarse_x+=1


		# Set vertical blank at scan line below bottom of page and set nmi i.e. emit interrupt
		if self.scan_line == 241 and self.cycle == 1:	
			self.status.b.vertical_blank = 1
			if self.ctrl.b.enable_nmi:
				self.nmi = True

		
		if self.cycle == 256: 	# Increment Y to next row
			"""
			A row of spites has been rendered to the screen we need to begin a new line. We collect sprite information
			in chunks of bytes so we increment course X. However only one row of pixels of the sprite are collected.
			In other words, we must increment fine y.
			"""
			if self.mask.b.render_background or self.mask.b.render_sprites:
				if self.loopy_v.b.fine_y < 7:
					self.loopy_v.b.fine_y+=1
				else:
					# We have reached the end of a tile, we may need to perform a wrap around
					self.loopy_v.b.fine_y = 0

					if self.loopy_v.b.coarse_y == 29:
						"""
						Boundary of nametable reached, recall y direction only 
						has 30 tiles, 2 for attribute memory.
						"""
						self.loopy_v.b.coarse_y = 0
						self.loopy_v.b.nametable_y^=1 # Negate nametable bit to increment nametable
					elif self.loopy_v.b.coarse_y == 31: 	# If in attribute memory					
						self.loopy_v.b.coarse_y = 0
					else:	# No wrap arounds so just increment normally
						self.loopy_v.b.coarse_y+=1

		if self.cycle == 257:
			"""
			If we reach edge of screen we need to update the vram register with what is in the tram
			"""
			if self.mask.b.render_background or self.mask.b.render_sprites:
				self.loopy_v.b.nametable_x = self.loopy_t.b.nametable_x
				self.loopy_v.b.coarse_x = self.loopy_t.b.coarse_x

		if self.scan_line == -1 and self.cycle >= 280 and self.cycle < 305:
			if self.mask.b.render_background or self.mask.b.render_sprites:
				self.loopy_v.b.nametable_y = self.loopy_t.b.nametable_y
				self.loopy_v.b.coarse_y = self.loopy_t.b.coarse_y
				self.loopy_v.b.fine_y = self.loopy_t.b.fine_y

		self.cycle+=1 									# Scan across screen

		if self.cycle >= 341:	# Finished columns
			self.cycle = 0
			self.scan_line+=1

			if self.scan_line >= 261:	# Finished rows
				self.scan_line = -1

		if self.mask.b.render_background:
			bit_mux = 0x8000 >> self.fine_x

			p0_pixel = (self._bg_shifter_pattern_lo&bit_mux) > 0
			p1_pixel = (self._bg_shifter_pattern_hi&bit_mux) > 0
			bg_pixel = (p1_pixel<<1) | p0_pixel

			bg_pal0 = (self._bg_shifter_attr_lo&bit_mux) > 0
			bg_pal1 = (self._bg_shifter_attr_hi&bit_mux) > 0
			bg_palette = (bg_pal1<<1) | bg_pal0

			# TODO: Make functon which returns rgb colour and renders on screen
			c = self.colours[self.ppu_read(0x3F00 + (bg_palette<<2) + bg_pixel)]
			self.screen(self.scan_line, self.cycle-1, c)


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
			"""
			Each name table is 1kB divided into 32x32 8bit entries.
			Each entry will point to a sprite in the pattern memory. There are 16x16 sprites which 
			requires 256 bits.
			The nametable uses mirroring to have 4 addressable nametables although only 2 physically exist.
			The range on the bus is 7kB so we mirror the 4 name tables.
			"""
			addr &= 0x0FFF	# mirror 4kB
			if self.v_mirroring:	# Vertical mirroring
				if addr >= 0x0000 and addr <= 0x03FF:
					self.nametable[0, addr & 0x3FF] = data
				elif addr >= 0x0400 and addr <= 0x07FF:
					self.nametable[1, addr & 0x3FF] = data
				elif addr >= 0x0800 and addr <= 0x0BFF:
					self.nametable[0, addr & 0x3FF] = data
				elif addr >= 0x0C00 and addr <= 0x0FFF:
					self.nametable[1, addr & 0x3FF] = data
			else:	# Horizontal mirroring
				if addr >= 0x0000 and addr <= 0x03FF:
					self.nametable[0, addr & 0x3FF] = data
				elif addr >= 0x0400 and addr <= 0x07FF:
					self.nametable[0, addr & 0x3FF] = data
				elif addr >= 0x0800 and addr <= 0x0BFF:
					self.nametable[1, addr & 0x3FF] = data
				elif addr >= 0x0C00 and addr <= 0x0FFF:
					self.nametable[1, addr & 0x3FF] = data

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
			"""
			Each name table is 1kB divided into 32x32 8bit entries.
			Each entry will point to a sprite in the pattern memory. There are 16x16 sprites which 
			requires 256 bits.
			The nametable uses mirroring to have 4 addressable nametables although only 2 physically exist.
			The range on the bus is 7kB so we mirror the 4 name tables.
			"""
			addr &= 0x0FFF	# mirror 4kB
			if self.v_mirroring:	# Vertical mirroring
				if addr >= 0x0000 and addr <= 0x03FF:
					data = self.nametable[0, addr & 0x3FF]
				elif addr >= 0x0400 and addr <= 0x07FF:
					data = self.nametable[1, addr & 0x3FF]
				elif addr >= 0x0800 and addr <= 0x0BFF:
					data = self.nametable[0, addr & 0x3FF]
				elif addr >= 0x0C00 and addr <= 0x0FFF:
					data = self.nametable[1, addr & 0x3FF]
			else:	# Horizontal mirroring
				if addr >= 0x0000 and addr <= 0x03FF:
					data = self.nametable[0, addr & 0x3FF]
				elif addr >= 0x0400 and addr <= 0x07FF:
					data = self.nametable[0, addr & 0x3FF]
				elif addr >= 0x0800 and addr <= 0x0BFF:
					data = self.nametable[1, addr & 0x3FF]
				elif addr >= 0x0C00 and addr <= 0x0FFF:
					data = self.nametable[1, addr & 0x3FF]

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
			self.loopy_t.b.nametable_x = self.ctrl.b.nametable_x
			self.loopy_t.b.nametable_y = self.ctrl.b.nametable_y
		elif addr == 0x0001:		# Mask
			self.mask.reg = data
		elif addr == 0x0002:		# Status
			...	# Cannot be written to
		elif addr == 0x0003:		# OAM Address
			...
		elif addr == 0x0004:		# OAM Data
			...
		elif addr == 0x0005:		# Scroll
			if self._address_latch == 0:
				self.fine_x = data&0x7	# Lower 3 bits specify pixel
				self.loopy_t.b.coarse_x = data>>3 	# Upper 5 bits specify tile in nametable, (32x30)
				self._address_latch = 1
			else:
				self.loopy_t.b.fine_y = data&0x7	# Lower 3 bits specify pixel
				self.loopy_t.b.coarse_y = data>>3 	# Upper 5 bits specify y-coord of tile in nametable (30)
				self._address_latch = 0
			
		elif addr == 0x0006:		# PPU Address
			if self._address_latch == 0:
				self.loopy_t.reg = (self.loopy_t.reg&0x00FF) | ((data&0x3F)<<8)	# Writing high byte
				self._address_latch = 1
			else:
				self.loopy_t.reg = (self.loopy_t.reg&0xFF00) | data	# Writing low byte
				self.loopy_v.reg = self.loopy_t.reg
				self._address_latch = 0

		elif addr == 0x0007:		# PPU Data
			self.ppu_write(self.loopy_v.reg, data)
			self.loopy_v.reg += (32 if self.ctrl.b.increment_mode else 1)

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
			self._ppu_data_buffer = self.ppu_read(self.loopy_v.reg)

			if self.loopy_v.reg > 0x3F00:					# Palette uses combinatorial logic which can output data in same clock cycle
				data = self._ppu_data_buffer

			self.loopy_v.reg += (32 if self.ctrl.b.increment_mode else 1)
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

		The palette memory range is 0x3F00 to 0x3FFF. The palette contains the index of the colour in the display colours
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