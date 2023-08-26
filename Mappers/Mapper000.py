from .Mapper import Mapper

class Mapper000(Mapper):
	# PPU with mapper communication
	def ppu_map_write(self, addr, data):
		"""
		The system cannot write to the cartridge so 
		this is always rejected.
		"""

		return False

	def ppu_map_read(self, addr, bReadOnly: bool = False):
		if addr >= 0x0000 and addr <= 0x1FFF:
			mapped_addr = addr
			return mapped_addr

		return False

	# CPU to mapper communication 
	def cpu_map_write(self, addr, data):
		if addr >= 0x8000 and addr <= 0xFFFF:		# Cartridge
			"""
			If address within mapper range, then return mapped address which 
			is indexed from zero to match ines file.
			If 32kB then just remove address 0x8000 offset. 
			If 16kB then mirror address (0x3FFF = 16kB). 
			"""
			mapped_addr = addr & (0x7FFF if self.prog_banks > 1 else 0x3FFF)
			return mapped_addr

		return False


	def cpu_map_read(self, addr, bReadOnly: bool = False):
		if addr >= 0x8000 and addr <= 0xFFFF:
			mapped_addr = addr & (0x7FFF if self.prog_banks > 1 else 0x3FFF)
			return mapped_addr

		return False