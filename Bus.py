import numpy as np


class Bus:
	def __init__(self, cpu):
		self.cpu = cpu
		self.ram = np.empty(64_000, dtype=np.uint8)

	def write(self, addr: np.uint16, data: np.uint8):
		if addr >= 0x0000 and addr <= 0xFFFF:
			self.ram[addr] = data


	def read(self, addr: np.uint16, bReadOnly: bool = False):
		if addr >= 0x0000 and addr <= 0xFFFF:
			return self.ram[addr]
		return 0x00