# cython: boundscheck=False
# cython: wraparound=False
# cython: cdivision=True
# cython: nonecheck=False
# cython: initializedcheck=False
# cython: overflowcheck=False
# cython: cflags=-O3

import numpy as np
import ctypes
c_uint8 = ctypes.c_uint8
c_uint16 = ctypes.c_uint16
from .available_colours import pal_screen

cimport numpy as np

import ctypes

cdef extern from "stdint.h":
	ctypedef unsigned char uint8_t
	ctypedef unsigned short uint16_t
	ctypedef short int16_t


from libcpp cimport bool

cdef class Ctrl:
	cdef uint8_t nametable_x
	cdef uint8_t nametable_y
	cdef uint8_t increment_mode
	cdef uint8_t pattern_sprite
	cdef uint8_t pattern_background
	cdef uint8_t sprite_size
	cdef uint8_t slave_mode
	cdef uint8_t enable_nmi

	cdef uint16_t _reg

	def __cinit__(self):
		self.nametable_x = 0
		self.nametable_y = 0
		self.increment_mode = 0
		self.pattern_sprite = 0
		self.pattern_background = 0
		self.sprite_size = 0
		self.slave_mode = 0
		self.enable_nmi = 0

		self._reg = 0

	@property
	def reg(self):
		return self._reg 

	@reg.setter
	def reg(self, v):
		self._reg = v & 0xFFFF
		self.nametable_x = (self._reg>>0) & 1
		self.nametable_y = (self._reg>>1) & 1
		self.increment_mode = (self._reg>>2) & 1
		self.pattern_sprite = (self._reg>>3) & 1
		self.pattern_background = (self._reg>>4) & 1
		self.sprite_size = (self._reg>>5) & 1
		self.slave_mode = (self._reg>>6) & 1
		self.enable_nmi = (self._reg>>7) & 1

	cdef void update_reg(self):
		self._reg = (
			(self.nametable_x << 0)
			| (self.nametable_y << 1)
			| (self.increment_mode << 2)
			| (self.pattern_sprite << 3)
			| (self.pattern_background << 4)
			| (self.sprite_size << 5)
			| (self.slave_mode << 6)
			| (self.enable_nmi << 7)
		)


cdef class Mask:
	cdef uint8_t grayscale
	cdef uint8_t render_background_left
	cdef uint8_t render_sprites_left
	cdef uint8_t render_background
	cdef uint8_t render_sprites
	cdef uint8_t enhance_red
	cdef uint8_t enhance_green
	cdef uint8_t enhance_blue

	cdef uint16_t _reg

	def __cinit__(self):
		self.grayscale = 0
		self.render_background_left = 0
		self.render_sprites_left = 0
		self.render_background = 0
		self.render_sprites = 0
		self.enhance_red = 0
		self.enhance_green = 0
		self.enhance_blue = 0

		self._reg = 0

	@property
	def reg(self):
		return self._reg 

	@reg.setter
	def reg(self, v):
		self._reg = v & 0xFFFF
		self.grayscale = self._reg & (1 << 0)
		self.render_background_left = self._reg & (1 << 1)
		self.render_sprites_left = self._reg & (1 << 2)
		self.render_background = self._reg & (1 << 3)
		self.render_sprites = self._reg & (1 << 4)
		self.enhance_red = self._reg & (1 << 5)
		self.enhance_green = self._reg & (1 << 6)
		self.enhance_blue = self._reg & (1 << 7)

	cdef void update_reg(self):
		self._reg = (
			(self.grayscale << 0)
			| (self.render_background_left << 1)
			| (self.render_sprites_left << 2)
			| (self.render_background << 3)
			| (self.render_sprites << 4)
			| (self.enhance_red << 5)
			| (self.enhance_green << 6)
			| (self.enhance_blue << 7)
		)

cdef class Status:
	cdef uint8_t unused
	cdef uint8_t sprite_overflow
	cdef uint8_t sprite_zero_hit
	cdef uint8_t vertical_blank

	cdef uint16_t _reg

	def __cinit__(self):
		self.unused = 0
		self.sprite_overflow = 0
		self.sprite_zero_hit = 0
		self.vertical_blank = 0

		self._reg = 0

	@property
	def reg(self):
		return self._reg 

	@reg.setter
	def reg(self, v):
		self._reg = v & 0xFFFF
		self.unused = self._reg & 0x1F
		self.sprite_overflow = (self._reg>>5) & 0b01
		self.sprite_zero_hit = (self._reg>>6) & 0b01
		self.vertical_blank = (self._reg>>7) & 0b01

	cdef void update_reg(self):
		self._reg = (
			(self.unused << 0)
			| (self.sprite_overflow << 5)
			| (self.sprite_zero_hit << 6)
			| (self.vertical_blank << 7)
		)

cdef class Loopy:
	cdef uint8_t coarse_x
	cdef uint8_t coarse_y
	cdef uint8_t nametable_x
	cdef uint8_t nametable_y
	cdef uint8_t fine_y
	cdef uint8_t unused

	cdef uint16_t _reg

	def __cinit__(self):
		self.coarse_x = 0
		self.coarse_y = 0
		self.nametable_x = 0
		self.nametable_y = 0
		self.fine_y = 0
		self.unused = 0

		self._reg = 0

	@property
	def reg(self):
		return self._reg 

	@reg.setter
	def reg(self, v):
		self._reg = v & 0xFFFF
		self.coarse_x = self._reg & 0x1F
		self.coarse_y = (self._reg>>5) & 0x1F
		self.nametable_x = (self._reg>>10) & 0b01
		self.nametable_y = (self._reg>>11) & 0b01
		self.fine_y = (self._reg>>12) & 0b111
		self.unused = (self._reg>>15) & 0b01

	cdef void update_reg(self):
		self._reg = (
			(self.unused << 15)
			| (self.fine_y << 12)
			| (self.nametable_y << 11)
			| (self.nametable_x << 10)
			| (self.coarse_y << 5)
			| self.coarse_x
		)


cdef class PPU:
	"""
	The PPU has its own internal bus with three memories attached.
	The pattern (char) memory which is a part of the cartridge located at 
	0x0000 to 0x1xFFFF.
	Name table memory which stores layout is located in range 0x2000 to 0x3EFF.
	Palette memory which stores color information located in range 0x3F00 to 
	0x3FFF.
	"""
	cdef object cartridge
	cdef object screen

	cdef int cycle
	cdef int scan_line

	cdef Ctrl ctrl
	cdef Mask mask
	cdef Status status
	cdef Loopy loopy_t
	cdef Loopy loopy_v

	cdef uint8_t fine_x
	cdef uint8_t data
	cdef uint8_t _address_latch
	cdef uint8_t _ppu_data_buffer
	cdef uint8_t nmi

	cdef object nametable
	cdef uint8_t[:,:] nametable_a
	cdef uint8_t[:,:] nametable_b
	cdef uint8_t[:] palettes

	cdef uint8_t[:,:] colours

	cdef uint8_t _v_mirroring

	cdef uint8_t _bg_next_tile_id
	cdef uint8_t _bg_next_tile_attr
	cdef uint8_t _bg_next_tile_lsb
	cdef uint8_t _bg_next_tile_msb

	cdef uint16_t _bg_shifter_pattern_lo
	cdef uint16_t _bg_shifter_pattern_hi
	cdef uint16_t _bg_shifter_attr_lo
	cdef uint16_t _bg_shifter_attr_hi

	cdef uint8_t end_of_frame

	cdef uint8_t[:] OAM
	cdef uint8_t OAM_addr

	cdef uint8_t[:] OAM_scanline
	cdef uint8_t sprite_count

	cdef uint8_t _n_OAM_entry
	cdef int16_t _diff

	cdef uint8_t[:] sprite_shifter_pattern_lo
	cdef uint8_t[:] sprite_shifter_pattern_hi

	cdef uint8_t sprite_shifter_pattern_lo_bits
	cdef uint8_t sprite_shifter_pattern_hi_bits
	cdef uint16_t sprite_shifter_pattern_lo_addr
	cdef uint16_t sprite_shifter_pattern_hi_addr

	cdef uint8_t sprite_zero_hit_possible
	cdef uint8_t rendering_sprite_zero


	def __cinit__(self):
		self.cartridge = None
		self.screen = np.zeros((256, 240, 3), dtype=np.uint8)

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
		self.data = 0x00

		self._address_latch = 0x00		# Keeps track of which address byte is being written
		self._ppu_data_buffer = 0x00	# Stores data for clock cycle delay before it is returned to cpu
		self.nmi = False

	# 	# Components attached to PPU bus (pattern memory is implicit since it is contained in the cartridge)
		self.nametable = None	# Defined when mirroring mode is specified
		self.nametable_a = np.zeros((32,32), dtype=np.uint8)
		self.nametable_b = np.zeros((32,32), dtype=np.uint8)

		self.palettes = np.zeros(32, dtype=np.uint8)	# 32 bytes
		self.colours = pal_screen

	# 	# Stores sprite pattern information
	# 	self._sprites_pixels = None
	# 	self._sprites_colours = None

		self._v_mirroring = False

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

		self.end_of_frame = False

		self.OAM = np.zeros(64*4, dtype=np.uint8)
		self.OAM_addr = 0x00

		self.OAM_scanline = np.zeros(8*4, dtype=np.uint8)
		self.sprite_count = 0

		self._n_OAM_entry = 0x00
		self._diff = 0x0000

		self.sprite_shifter_pattern_lo = np.zeros(8, dtype=np.uint8)
		self.sprite_shifter_pattern_hi = np.zeros(8, dtype=np.uint8)

		self.sprite_shifter_pattern_lo_bits = 0
		self.sprite_shifter_pattern_lo_addr = 0
		self.sprite_shifter_pattern_hi_bits = 0
		self.sprite_shifter_pattern_hi_addr = 0

		self.sprite_zero_hit_possible = False
		self.rendering_sprite_zero = False

	@property
	def OAM(self):
		return self.OAM

	@property
	def screen(self):
		return self.screen

	@property
	def nmi(self):
		return self.nmi

	@nmi.setter
	def nmi(self, v):
		self.nmi = v

	@property
	def v_mirroring(self):
		return self._v_mirroring

	@v_mirroring.setter
	def v_mirroring(self, v):
		self._v_mirroring = v

		# Once the mirroring mode has been set we set up the nametable to match
		if self.v_mirroring:
			self.nametable = [
								[self.nametable_a, self.nametable_b],
								[self.nametable_a, self.nametable_b]
							]
		else:
			self.nametable = [
								[self.nametable_a, self.nametable_a],
								[self.nametable_b, self.nametable_b]
							]


	cdef void _load_bg_shifters(self):
		# Place tile data into shift registers for rendering onto screen
		self._bg_shifter_pattern_lo = (self._bg_shifter_pattern_lo&0xFF00) | self._bg_next_tile_lsb
		self._bg_shifter_pattern_hi = (self._bg_shifter_pattern_hi&0xFF00) | self._bg_next_tile_msb
		self._bg_shifter_attr_lo = (self._bg_shifter_attr_lo&0xFF00) | (0xFF if (self._bg_next_tile_attr&0b01) else 0x00)
		self._bg_shifter_attr_hi = (self._bg_shifter_attr_hi&0xFF00) | (0xFF if (self._bg_next_tile_attr&0b10) else 0x00)

	cdef uint8_t _flip_byte(self, uint8_t b):
		b = ((b & 0xF0) >> 4) | ((b & 0x0F) << 4)
		b = ((b & 0xCC) >> 2) | ((b & 0x33) << 2)
		b = ((b & 0xAA) >> 1) | ((b & 0x55) << 1)

		return b

	cpdef clock(self):
		# https://www.nesdev.org/w/images/default/4/4f/Ppu.svg
		cdef int i
		if self.scan_line >= -1 and self.scan_line < 240:	# Visible region of screen
			if self.scan_line == 0 and self.cycle == 0:
				self.cycle = 1

			if self.scan_line == -1 and self.cycle == 1:	# Unset vertical blank at top of page
				self.status.vertical_blank = 0
				self.status.sprite_overflow = 0
				self.status.sprite_zero_hit = 0
				self.status.update_reg()

				self.sprite_shifter_pattern_lo[:] = 0
				self.sprite_shifter_pattern_hi[:] = 0

			# Prepare next sprite loop, first condition is while screen is being renderd, second is before jumping to top of screen
			if (self.cycle >= 2 and self.cycle < 258) or (self.cycle >= 321 and self.cycle < 338):
				if self.mask.render_background:
					# Shift registers every cycle
					self._bg_shifter_pattern_lo <<= 1
					self._bg_shifter_pattern_hi <<= 1
					self._bg_shifter_attr_lo <<= 1
					self._bg_shifter_attr_hi <<= 1

				if self.mask.render_sprites and self.cycle >= 1 and self.cycle < 258:
					for i in range(self.sprite_count):
						if self.OAM_scanline[(i<<2)+3] > 0:
							self.OAM_scanline[(i<<2)+3]-=1
						else:
							self.sprite_shifter_pattern_lo[i] <<= 1
							self.sprite_shifter_pattern_hi[i] <<= 1

				if (self.cycle-1)%8 == 0:	# Read background tile byte from nametable
					self._load_bg_shifters()

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
					self._bg_next_tile_id = self.ppu_read(0x2000|(self.loopy_v._reg&0x0FFF))

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
																| self.loopy_v.nametable_y << 11
																| self.loopy_v.nametable_x << 10
																| (self.loopy_v.coarse_y>>2) << 3
																| (self.loopy_v.coarse_x>>2) << 0
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

					if self.loopy_v.coarse_y&0x2: # Bottom row if > 0
						self._bg_next_tile_attr >>= 4
					if self.loopy_v.coarse_x&0x2: # Right column if > 0
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
					# self._bg_next_tile_lsb = self.cartridge.v_char_memory_nd[	
					# 									0, 
					# 									self.ctrl.pattern_background,
					# 									(self._bg_next_tile_id&0xF0)>>4,
					# 									self._bg_next_tile_id&0x0F,
					# 									0,
					# 									self.loopy_v.fine_y
					# 								]
					self._bg_next_tile_lsb = self.ppu_read(
							(self.ctrl.pattern_background<<12)
							+ (self._bg_next_tile_id << 4)
							+ self.loopy_v.fine_y
						)
					
				elif (self.cycle-1)%8 == 6:	# Prepare pixel msb
					# self._bg_next_tile_msb = self.cartridge.v_char_memory_nd[	
					# 									0, 
					# 									self.ctrl.pattern_background,
					# 									(self._bg_next_tile_id&0xF0)>>4,
					# 									self._bg_next_tile_id&0x0F,
					# 									1,
					# 									self.loopy_v.fine_y 
					# 								]

					self._bg_next_tile_msb = self.ppu_read(
							(self.ctrl.pattern_background<<12)
							+ (self._bg_next_tile_id << 4)
							+ self.loopy_v.fine_y + 8
						)

				elif (self.cycle-1)%8 == 7:	# Increment tile pointer
					"""
					Information about sprite has been loaded so we can increment the course x to begin working on the 
					next sprite.
					"""
					if self.mask.render_background or self.mask.render_sprites:
						if self.loopy_v.coarse_x == 31:	# Boundary of nametable reached
							self.loopy_v.coarse_x = 0
							self.loopy_v.nametable_x^=1 # Negate nametable bit to increment nametable
						else:
							self.loopy_v.coarse_x+=1
						self.loopy_v.update_reg()

			if self.cycle == 256: 	# Increment Y to next row
				"""
				A row of spites has been rendered to the screen we need to begin a new line. We collect sprite information
				in chunks of bytes so we increment course X. However only one row of pixels of the sprite are collected.
				In other words, we must increment fine y.
				"""
				if self.mask.render_background or self.mask.render_sprites:
					if self.loopy_v.fine_y < 7:
						self.loopy_v.fine_y+=1
					else:
						# We have reached the end of a tile, we may need to perform a wrap around
						self.loopy_v.fine_y = 0

						if self.loopy_v.coarse_y == 29:
							"""
							Boundary of nametable reached, recall y direction only 
							has 30 tiles, 2 for attribute memory.
							"""
							self.loopy_v.coarse_y = 0
							self.loopy_v.nametable_y^=1 # Negate nametable bit to increment nametable
						elif self.loopy_v.coarse_y > 29: 	# If in attribute memory					
							self.loopy_v.coarse_y = 0
						else:	# No wrap arounds so just increment normally
							self.loopy_v.coarse_y+=1

					self.loopy_v.update_reg()

			if self.cycle == 257:
				"""
				If we reach edge of screen we need to update the vram register with what is in the tram
				"""
				self._load_bg_shifters()
				if self.mask.render_background or self.mask.render_sprites:
					self.loopy_v.nametable_x = self.loopy_t.nametable_x
					self.loopy_v.coarse_x = self.loopy_t.coarse_x

					self.loopy_v.update_reg()

			if self.cycle == 338 or self.cycle == 340:
				self._bg_next_tile_id = self.ppu_read(0x2000 | (self.loopy_v._reg&0x0FFF))

			if self.scan_line == -1 and self.cycle >= 280 and self.cycle < 305:
				if self.mask.render_background or self.mask.render_sprites:
					self.loopy_v.nametable_y = self.loopy_t.nametable_y
					self.loopy_v.coarse_y = self.loopy_t.coarse_y
					self.loopy_v.fine_y = self.loopy_t.fine_y

					self.loopy_v.update_reg()


			# Foreground Rendering

			# Sprite evaluation phase: Get sprites to be rendered on next scanline during blanking period
			if self.cycle == 257 and self.scan_line >= 0:
				# At beginning of scanline reset all sprite memory and count
				self.sprite_count = 0
				self.OAM_scanline[:] = 0xFF

				self.sprite_shifter_pattern_lo[:] = 0
				self.sprite_shifter_pattern_hi[:] = 0

				self._n_OAM_entry = 0
				self.sprite_zero_hit_possible = False

				while self._n_OAM_entry < 64 and self.sprite_count < 9:
					self._diff = self.scan_line - self.OAM[self._n_OAM_entry<<2 + 0]

					if self._diff >= 0 and self._diff < (16 if self.ctrl.sprite_size else 8):
						if self.sprite_count < 8:
							if self._n_OAM_entry == 0:
								self.sprite_zero_hit_possible = True

							self.OAM_scanline[(self.sprite_count<<2):((self.sprite_count<<2)+4)] = self.OAM[(self._n_OAM_entry<<2):(self._n_OAM_entry<<2)+4]
							self.sprite_count+=1

					self._n_OAM_entry+=1

				self.status.sprite_overflow = self.sprite_count > 8

			# Get data from pattern memory for sprites to be drawn on next scan line
			if self.cycle == 340:
				for i in range(0,self.sprite_count*4, 4):
					if not self.ctrl.sprite_size: # 8x8 sprite
						if not (self.OAM_scanline[i+2]&0x80):	# Check if sprite flipped vertically
							# Sprite is NOT flipped vertically
							self.sprite_shifter_pattern_lo_addr = (
																(self.ctrl.pattern_sprite<<12)
																| (self.OAM_scanline[i+1] << 4)
																| (self.scan_line - self.OAM_scanline[i+0])
																)
						else:
							# Sprite is flipped vertically
							self.sprite_shifter_pattern_lo_addr = ((self.ctrl.pattern_sprite<<12)
																+ (self.OAM_scanline[i+1] << 4)
																+ (7 - (self.scan_line - self.OAM_scanline[i+0])))

					else: # 8x16 sprite
						if not (self.OAM_scanline[i+2]&0x80):	# Check if sprite flipped vertically
							# Sprite Not flipped verically
							if self.scan_line - self.OAM_scanline[i] < 8: # Reading top half of tile
								self.sprite_shifter_pattern_lo_addr = (((self.OAM_scanline[i+1]&1)<<12)
																		| ((self.OAM_scanline[i+1]&0xFE) << 4)
																		| (self.scan_line - self.OAM_scanline[i+0])&0x07)
							else: # Reading bottom half of tile
								self.sprite_shifter_pattern_lo_addr = (((self.OAM_scanline[i+1]&1)<<12)
																		| (((self.OAM_scanline[i+1]&0xFE)+1) << 4)
																		| (self.scan_line - self.OAM_scanline[i+0])&0x07)
						else: # Sprite flipped vertically
							if self.scan_line - self.OAM_scanline[i] < 8: # Reading top half of tile
								self.sprite_shifter_pattern_lo_addr = (((self.OAM_scanline[i+1]&1)<<12)
																		| (((self.OAM_scanline[i+1]&0xFE) + 1) << 4)
																		| (7 - (self.scan_line - self.OAM_scanline[i+0])&0x07))
							else: # Reading bottom half of tile
								self.sprite_shifter_pattern_lo_addr = (((self.OAM_scanline[i+1]&1)<<12)
																		| (((self.OAM_scanline[i+1]&0xFE)) << 4)
																		| (7 - (self.scan_line - self.OAM_scanline[i+0])&0x07))

					# Get pixel address from high bit plane
					self.sprite_shifter_pattern_hi_addr = self.sprite_shifter_pattern_lo_addr + 8

					# Get pixels for row of sprite data
					self.sprite_shifter_pattern_lo_bits = self.ppu_read(self.sprite_shifter_pattern_lo_addr)	
					self.sprite_shifter_pattern_hi_bits = self.ppu_read(self.sprite_shifter_pattern_hi_addr)	

					# Flip sprites horizonally if necessary
					if self.OAM_scanline[i + 2] & 0x40:
						self.sprite_shifter_pattern_lo_bits = self._flip_byte(self.sprite_shifter_pattern_lo_bits)
						self.sprite_shifter_pattern_hi_bits = self._flip_byte(self.sprite_shifter_pattern_hi_bits)

					# Load tiles into shift registers
					self.sprite_shifter_pattern_lo[i>>2] = self.sprite_shifter_pattern_lo_bits
					self.sprite_shifter_pattern_hi[i>>2] = self.sprite_shifter_pattern_hi_bits

		if self.scan_line >= 241 and self.scan_line < 261:
			# Set vertical blank at scan line below bottom of page and set nmi i.e. emit interrupt
			if self.scan_line == 241 and self.cycle == 1:	
				self.status.vertical_blank = 1
				self.status.update_reg()

				self.end_of_frame = True
				if self.ctrl.enable_nmi:
					self.nmi = True

		# Combine background rendering information
		cdef uint8_t bg_pixel, bg_palette
		bg_pixel, bg_palette = 0, 0

		cdef uint16_t bit_mux
		cdef uint8_t p0_pixel, p1_pixel, bg_pal0, bg_pal1

		if self.mask.render_background:
			bit_mux = 0x8000 >> self.fine_x

			p0_pixel = (self._bg_shifter_pattern_lo&bit_mux) > 0
			p1_pixel = (self._bg_shifter_pattern_hi&bit_mux) > 0
			bg_pixel = (p1_pixel<<1) | p0_pixel

			bg_pal0 = (self._bg_shifter_attr_lo&bit_mux) > 0
			bg_pal1 = (self._bg_shifter_attr_hi&bit_mux) > 0
			bg_palette = (bg_pal1<<1) | bg_pal0


		# Combine foreground rendering information
		cdef uint8_t fg_pixel, fg_palette, fg_priority
		fg_pixel, fg_palette, fg_priority = 0, 0, 0

		cdef uint8_t fg_p0_pixel, fg_p1_pixel

		if self.mask.render_sprites:
			self.rendering_sprite_zero = False

			for i in range(self.sprite_count):
				if self.OAM_scanline[(i<<2)+3] == 0:
					fg_p0_pixel = (self.sprite_shifter_pattern_lo[i]&0x80) > 0
					fg_p1_pixel = (self.sprite_shifter_pattern_hi[i]&0x80) > 0
					fg_pixel = (fg_p1_pixel<<1) | fg_p0_pixel

					fg_palette = (self.OAM_scanline[(i<<2)+2] & 0x03) + 0x04
					fg_priority = (self.OAM_scanline[(i<<2)+2] & 0x20) == 0 	# Priority of sprites with background

					# If we have found a sprite which will render infront of the background then we can stop since everything else is lower priority
					if fg_pixel != 0:
						if i == 0:
							self.rendering_sprite_zero = True
						break

		# Find which pixel is to be chosen between background and foreground
		cdef uint8_t pixel, palette
		if bg_pixel == 0 and fg_pixel == 0:
			pixel = 0
			palette = 0
		elif bg_pixel == 0 and fg_pixel > 0:
			pixel = fg_pixel
			palette = fg_palette
		elif bg_pixel > 0 and fg_pixel == 0:
			pixel = bg_pixel
			palette = bg_palette
		else:
			if fg_priority:
				pixel = fg_pixel
				palette = fg_palette
			else:
				pixel = bg_pixel
				palette = bg_palette

			if self.sprite_zero_hit_possible and self.rendering_sprite_zero:
				if self.mask.render_background and self.mask.render_sprites:
					if not (self.mask.render_background_left | self.mask.render_sprites_left):
						if self.cycle >= 9 and self.cycle < 258:
							self.status.sprite_zero_hit = 1
							self.status.update_reg()
					else:
						if self.cycle >= 1 and self.cycle < 258:
							self.status.sprite_zero_hit = 1
							self.status.update_reg()


		c = self.colours[self.ppu_read(0x3F00 + (palette<<2) + pixel)]
		if self.cycle <= 255 and self.scan_line <= 239:
			self.screen[self.cycle-1, self.scan_line] = c

		self.cycle+=1 	# Scan across screen
		if self.cycle >= 341:	# Finished columns
			self.cycle = 0
			self.scan_line+=1

			if self.scan_line >= 261:	# Finished rows
				self.scan_line = -1


	# Connect Components
	def connect_cartridge(self, cartridge):
		self.cartridge = cartridge
		cartridge.connect_ppu(self)

	# PPU with internal bus communication
	cpdef void ppu_write(self, uint16_t addr, uint8_t data):
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
			NT_y = (addr >> 11) & 0xb1
			NT_x = (addr >> 10) & 0xb1
			coarse_y = (addr >> 5) & 0x1F
			coarse_x = (addr >> 0) & 0x1F
			self.nametable[NT_y][NT_x][coarse_y, coarse_x] = data

		elif (addr >= 0x3F00) and (addr <= 0x3FFF):		# Palette memory
			addr &= 0x1F	# address mod 32bytes
			# Mirrors fg and bg colour palettes
			if addr == 0x10: addr = 0x0000
			if addr == 0x14: addr = 0x0004
			if addr == 0x18: addr = 0x0008
			if addr == 0x1C: addr = 0x000C
			self.palettes[addr] = data

	cpdef uint8_t ppu_read(self, uint16_t addr):
		addr &= 0x3FFF	# Ensure in addressable range of PPU bus
		data, valid_addr = self.cartridge.ppu_read(addr)
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
			NT_y = (addr >> 11) & 0xb1
			NT_x = (addr >> 10) & 0xb1
			coarse_y = (addr >> 5) & 0x1F
			coarse_x = (addr >> 0) & 0x1F
			data = self.nametable[NT_y][NT_x][coarse_y, coarse_x]

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
	cpdef void cpu_write(self, uint16_t addr, uint8_t data):
		if addr == 0x0000:			# Control
			self.ctrl.reg = data
			self.loopy_t.nametable_x = self.ctrl.nametable_x
			self.loopy_t.nametable_y = self.ctrl.nametable_y
		elif addr == 0x0001:		# Mask
			self.mask.reg = data
		elif addr == 0x0002:		# Status
			...	# Cannot be written to
		elif addr == 0x0003:		# OAM Address
			self.OAM_addr = data
		elif addr == 0x0004:		# OAM Data
			self.OAM[self.OAM_addr] = data

		elif addr == 0x0005:		# Scroll
			if self._address_latch == 0:
				self.fine_x = data&0x7	# Lower 3 bits specify pixel
				self.loopy_t.coarse_x = data>>3 	# Upper 5 bits specify tile in nametable, (32x30)
				self._address_latch = 1
			else:
				self.loopy_t.fine_y = data&0x7	# Lower 3 bits specify pixel
				self.loopy_t.coarse_y = data>>3 	# Upper 5 bits specify y-coord of tile in nametable (30)
				self._address_latch = 0

			self.loopy_v.update_reg()
			
		elif addr == 0x0006:		# PPU Address
			if self._address_latch == 0:
				self.loopy_t._reg = ((data&0x3F)<<8) | (self.loopy_t._reg&0x00FF)	# Writing high byte
				self._address_latch = 1
			else:
				self.loopy_t.reg = (self.loopy_t._reg&0xFF00) | data	# Writing low byte
				self.loopy_v.reg = self.loopy_t._reg
				self._address_latch = 0

		elif addr == 0x0007:		# PPU Data
			self.ppu_write(self.loopy_v._reg, data)
			self.loopy_v.reg += (32 if self.ctrl.increment_mode else 1)

	cpdef uint8_t cpu_read(self, uint16_t addr):
		data = 0x00
		if addr == 0x0000:			# Control
			...	# Not readable
		elif addr == 0x0001:		# Mask
			... # Not readable
		elif addr == 0x0002:		# Status
			# self.status.vertical_blank = 1 	# Uncomment for quick testing
			# First 5 status registers unused usually and will contain noise or buffer data 
			data = (self.status.reg&0xE0) | (self._ppu_data_buffer&0x1F)
			self.status.vertical_blank = 0
			self.status.update_reg()
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
			self._ppu_data_buffer = self.ppu_read(self.loopy_v._reg)

			if self.loopy_v._reg >= 0x3F00:					# Palette uses combinatorial logic which can output data in same clock cycle
				data = self._ppu_data_buffer

			self.loopy_v.reg += (32 if self.ctrl.increment_mode else 1)
		return data


	# # Gets 2-bit color for single tile from pattern (char) memory
	# def make_sprite_pixels(self):
	# 	"""
	# 	Pattern memory is split conceptually into two 4kB regions to make the full 8kB.
	# 	It stores a 2 bit number specifying the colour from some palette.
	# 	The first 4kB is usually sprites while the second is backgrounds.
		
	# 	Description:
	# 	There are 16x16 tiles, each tile contains 8x8 pixels. Each pixel is 2 bits => each tile
	# 	is 8x8x2 = 128b = 16bytes. Each 4kB range contains 16x16 tiles, each 16 bytes = 4kB.
	# 	The two bits are used to index a colour in the palette.

	# 	Physical Storage:
	# 	The LSB and MSB (B for stands for bit here, not byte) are stored in "planes". The LSB plane
	# 	is stored first follows by MSB plane. Each row is one byte and the tile is unrapped in bytes.
	
	# 	i: the first or second 4kB region 
	# 	palette: the row from the palettes table

	# 	To be proper we should make calls from the PPU. However I will ignore this since using numpy arrays 
	# 	to implement a half adder is clearer and very quick. 

	# 	This will need changing when we implement more complex mappers other than 000.
	# 	"""
	# 	# Populate palette memory
	# 	# if self.cartridge.mapper == Mapper_000:
	# 	lo, hi = self.cartridge.v_char_memory_nd[...,0,:], self.cartridge.v_char_memory_nd[...,1,:]
	# 	# Combines lower and upper bit to make pixel value
	# 	self._sprites_pixels = np.empty([*lo.shape, 8], dtype=np.uint8)	# NumPy doesn't have 2bit so 1byte is used i.e. 1byte atomicity

	# 	"""
	# 	Make table of palette offsets (pixels)
	# 	1. Isolates ith bit
	# 	2. Shifts high bit to the left by 1 further
	# 	3. Ors the low and high bits
	# 	4. Shifts result so that low bit is in the 0th position
	# 	"""
	# 	for i in range(7):
	# 		self._sprites_pixels[...,(7-i)] = np.bitwise_or(((hi&(1<<i))<<1), (lo&(1<<i)))>>i
	# 	self._sprites_pixels[...,0] = np.bitwise_or(((hi&(1<<7))), (lo&(1<<7))>>1)>>6

	# def make_sprite_colours(self, palette):
	# 	"""
	# 	We query the palette table to convert the 2-bit pixel values to 
	# 	RBG colours to be shown on the display.

	# 	The palette memory range is 0x3F00 to 0x3FFF. The palette contains the index of the colour in the display colours
	# 	array.
	# 	The first byte is the background.
	# 	Each 4-bytes thereafter are a palette. The pixel value determines which of the 4-bytes to use.
	# 	A colour is obtained by: 4*(palette #) + pixel.
	# 	The first 4 palettes are usually for bg and the next 4 are fg

	# 	The palette table is arranged as follows:
	# 	Palette #		0x3F00		bkgd
	# 	0				0x3F01		C1		C2		C3		bkgd
	# 	1				0x3F05		C1		C2		C3		bkgd
	# 	2				0x3F09		C1		C2		C3		bkgd
	# 	3				0x3F0D		C1		C2		C3		bkgd
	# 	4				0x3F11		C1		C2		C3		bkgd
	# 	5				0x3F15		C1		C2		C3		bkgd
	# 	6				0x3F19		C1		C2		C3		bkgd
	# 	7				0x3F1D		C1		C2		C3		bkgd
	# 	"""

	# 	if self._sprites_pixels is None:
	# 		self.make_sprite_pixels()

	# 	# Make table of colours
	# 	self._sprites_colours = np.empty([*self._sprites_pixels.shape, 3], dtype=np.uint8)

	# 	for ind, pixel in np.ndenumerate(self._sprites_pixels):
	# 		self._sprites_colours[ind] = self.colours[self.ppu_read(0x3F00 + (palette<<2) + pixel)]