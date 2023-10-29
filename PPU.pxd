cdef extern from "stdint.h":
	ctypedef unsigned char uint8_t
	ctypedef unsigned short uint16_t
	ctypedef short int16_t

from .Cartridge cimport Cartridge

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

	cdef void update_reg(self)

	
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

	cdef void update_reg(self)

	
cdef class Status:
	cdef uint8_t unused
	cdef uint8_t sprite_overflow
	cdef uint8_t sprite_zero_hit
	cdef uint8_t vertical_blank

	cdef uint16_t _reg

	cdef void update_reg(self)

cdef class Loopy:
	cdef uint8_t coarse_x
	cdef uint8_t coarse_y
	cdef uint8_t nametable_x
	cdef uint8_t nametable_y
	cdef uint8_t fine_y
	cdef uint8_t unused

	cdef uint16_t _reg

	cdef void update_reg(self)


cdef class PPU:
	"""
	The PPU has its own internal bus with three memories attached.
	The pattern (char) memory which is a part of the cartridge located at 
	0x0000 to 0x1xFFFF.
	Name table memory which stores layout is located in range 0x2000 to 0x3EFF.
	Palette memory which stores color information located in range 0x3F00 to 
	0x3FFF.
	"""
	cdef Cartridge cartridge
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

	cdef void _load_bg_shifters(self)
	cdef inline uint8_t _flip_byte(self, uint8_t b)
	cpdef void clock(self)
	cpdef void ppu_write(self, uint16_t addr, uint8_t data)
	cpdef uint8_t ppu_read(self, uint16_t addr)
	cpdef void cpu_write(self, uint16_t addr, uint8_t data)
	cpdef uint8_t cpu_read(self, uint16_t addr)