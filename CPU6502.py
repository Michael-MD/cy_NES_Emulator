from .Bus import Bus
import numpy as np
from enum import Enum
import pdb

""" 
Masks to be used for easily accessing individual bits of 
the cpu's status register.
"""
class flags(Enum):
	C = (1 << 0)	# Carry
	Z = (1 << 1)	# Zero
	I = (1 << 2)	# Interrupt disable
	D = (1 << 3)	# Decimal mode
	B = (1 << 4)	# Break
	U = (1 << 5)	# Unused
	V = (1 << 6)	# Overflow
	N = (1 << 7)	# Negative

class Instruction:
	def __init__(self, name: str, opcode, addr_mode, cycles: int):
		self.name = name
		self.opcode = opcode
		self.addr_mode = addr_mode
		self.cycles = cycles

class CPU6502:
	"""
	Implements high level 6502 CPU. 
	Datasheet: http://datasheets.chipdb.org/Rockwell/6502.pdf
	Instruction Set: https://www.pagetable.com/c64ref/6502/?tab=0
	Assembler: https://www.masswerk.at/6502/assembler.html
	"""
	def __init__(self, debugging_mode=False):
		self.a = 0x00 		# Accumulator Register
		self.x = 0x00 		# X Register
		self.y = 0x00 		# Y Register
		self.stkp = 0xFD 	# Stack Pointer
		self.pc = 0x0000 	# Program Counter
		self.status = 0x70 	# Status Register

		self._interrupt = False	# Maskable interrupt pin
		self._nm_interrupt = False # Non-maskable interrupt pin

		self.bus = None

		self._fetched = 0x00
		self._addr_abs = 0x0000	# Stores absolute address
		self._addr_rel = 0x0000	# Stores relative address from jump location
		self._opcode = 0x00		# Stores instruction opcode
		self._cycles = 0 		# Stores number of remaining instruction cycles
		self._debugging_mode = debugging_mode

		self.lookup = np.full(16**2, Instruction('???', opcode=self.XXX, addr_mode=self.IMP, cycles=2), dtype=Instruction)

		self.lookup[0x00] = Instruction('BRK', opcode=self.BRK, addr_mode=self.IMM, cycles=7)
		self.lookup[0x01] = Instruction('ORA', opcode=self.ORA, addr_mode=self.IZX, cycles=6)
		self.lookup[0x05] = Instruction('ORA', opcode=self.ORA, addr_mode=self.ZP0, cycles=3)
		self.lookup[0x06] = Instruction('ASL', opcode=self.ASL, addr_mode=self.ZP0, cycles=5)
		self.lookup[0x08] = Instruction('PHP', opcode=self.PHP, addr_mode=self.IMP, cycles=3)
		self.lookup[0x09] = Instruction('ORA', opcode=self.ORA, addr_mode=self.IMM, cycles=2)
		self.lookup[0x0A] = Instruction('ASL', opcode=self.ASL, addr_mode=self.IMP, cycles=2)
		self.lookup[0x0D] = Instruction('ORA', opcode=self.ORA, addr_mode=self.ABS, cycles=4)
		self.lookup[0x0E] = Instruction('ASL', opcode=self.ASL, addr_mode=self.ABS, cycles=6)
		self.lookup[0x10] = Instruction('BPL', opcode=self.BPL, addr_mode=self.REL, cycles=2)
		self.lookup[0x11] = Instruction('ORA', opcode=self.ORA, addr_mode=self.IZY, cycles=5)
		self.lookup[0x15] = Instruction('ORA', opcode=self.ORA, addr_mode=self.ZPX, cycles=4)
		self.lookup[0x16] = Instruction('ASL', opcode=self.ASL, addr_mode=self.ZPX, cycles=6)
		self.lookup[0x18] = Instruction('CLC', opcode=self.CLC, addr_mode=self.IMP, cycles=2)
		self.lookup[0x19] = Instruction('ORA', opcode=self.ORA, addr_mode=self.ABY, cycles=4)
		self.lookup[0x1D] = Instruction('ORA', opcode=self.ORA, addr_mode=self.ABX, cycles=4)
		self.lookup[0x1E] = Instruction('ASL', opcode=self.ASL, addr_mode=self.ABX, cycles=7)
		self.lookup[0x20] = Instruction('JSR', opcode=self.JSR, addr_mode=self.ABS, cycles=6)
		self.lookup[0x21] = Instruction('AND', opcode=self.AND, addr_mode=self.IZX, cycles=6)
		self.lookup[0x24] = Instruction('BIT', opcode=self.BIT, addr_mode=self.ZP0, cycles=3)
		self.lookup[0x25] = Instruction('AND', opcode=self.AND, addr_mode=self.ZP0, cycles=3)
		self.lookup[0x26] = Instruction('ROL', opcode=self.ROL, addr_mode=self.ZP0, cycles=5)
		self.lookup[0x28] = Instruction('PLP', opcode=self.PLP, addr_mode=self.IMP, cycles=4)
		self.lookup[0x29] = Instruction('AND', opcode=self.AND, addr_mode=self.IMM, cycles=2)
		self.lookup[0x2A] = Instruction('ROL', opcode=self.ROL, addr_mode=self.IMP, cycles=2)
		self.lookup[0x2C] = Instruction('BIT', opcode=self.BIT, addr_mode=self.ABS, cycles=4)
		self.lookup[0x2D] = Instruction('AND', opcode=self.AND, addr_mode=self.ABS, cycles=4)
		self.lookup[0x2E] = Instruction('ROL', opcode=self.ROL, addr_mode=self.ABS, cycles=6)
		self.lookup[0x30] = Instruction('BMI', opcode=self.BMI, addr_mode=self.REL, cycles=2)
		self.lookup[0x31] = Instruction('AND', opcode=self.AND, addr_mode=self.IZY, cycles=5)
		self.lookup[0x35] = Instruction('AND', opcode=self.AND, addr_mode=self.ZPX, cycles=4)
		self.lookup[0x36] = Instruction('ROL', opcode=self.ROL, addr_mode=self.ZPX, cycles=6)
		self.lookup[0x38] = Instruction('SEC', opcode=self.SEC, addr_mode=self.IMP, cycles=2)
		self.lookup[0x39] = Instruction('AND', opcode=self.AND, addr_mode=self.ABY, cycles=4)
		self.lookup[0x3D] = Instruction('AND', opcode=self.AND, addr_mode=self.ABX, cycles=4)
		self.lookup[0x3E] = Instruction('ROL', opcode=self.ROL, addr_mode=self.ABX, cycles=7)
		self.lookup[0x40] = Instruction('RTI', opcode=self.RTI, addr_mode=self.ABX, cycles=6)
		self.lookup[0x41] = Instruction('EOR', opcode=self.EOR, addr_mode=self.IZX, cycles=6)
		self.lookup[0x45] = Instruction('EOR', opcode=self.EOR, addr_mode=self.ZP0, cycles=3)
		self.lookup[0x46] = Instruction('LSR', opcode=self.LSR, addr_mode=self.ZP0, cycles=5)
		self.lookup[0x48] = Instruction('PHA', opcode=self.PHA, addr_mode=self.IMP, cycles=3)
		self.lookup[0x49] = Instruction('EOR', opcode=self.EOR, addr_mode=self.IMM, cycles=2)
		self.lookup[0x4A] = Instruction('LSR', opcode=self.LSR, addr_mode=self.IMP, cycles=2)
		self.lookup[0x4B] = Instruction('JMP', opcode=self.JMP, addr_mode=self.ABS, cycles=3)
		self.lookup[0x4D] = Instruction('EOR', opcode=self.EOR, addr_mode=self.ABS, cycles=4)
		self.lookup[0x4E] = Instruction('LSR', opcode=self.LSR, addr_mode=self.ABS, cycles=6)
		self.lookup[0x50] = Instruction('BVC', opcode=self.BVC, addr_mode=self.REL, cycles=2)
		self.lookup[0x51] = Instruction('EOR', opcode=self.EOR, addr_mode=self.IZY, cycles=5)
		self.lookup[0x55] = Instruction('EOR', opcode=self.EOR, addr_mode=self.ZPX, cycles=4)
		self.lookup[0x56] = Instruction('LSR', opcode=self.LSR, addr_mode=self.ZPX, cycles=6)
		self.lookup[0x58] = Instruction('CLI', opcode=self.CLI, addr_mode=self.IMP, cycles=2)
		self.lookup[0x59] = Instruction('EOR', opcode=self.EOR, addr_mode=self.ABY, cycles=4)
		self.lookup[0x5D] = Instruction('EOR', opcode=self.EOR, addr_mode=self.ABX, cycles=4)
		self.lookup[0x5E] = Instruction('LSR', opcode=self.LSR, addr_mode=self.ABX, cycles=7)
		self.lookup[0x60] = Instruction('RTS', opcode=self.RTS, addr_mode=self.IMP, cycles=6)
		self.lookup[0x61] = Instruction('ADC', opcode=self.ADC, addr_mode=self.IZX, cycles=6)
		self.lookup[0x65] = Instruction('ADC', opcode=self.ADC, addr_mode=self.ZP0, cycles=3)
		self.lookup[0x66] = Instruction('ROR', opcode=self.ROR, addr_mode=self.ZP0, cycles=5)
		self.lookup[0x68] = Instruction('PLA', opcode=self.PLA, addr_mode=self.IMP, cycles=4)
		self.lookup[0x69] = Instruction('ADC', opcode=self.ADC, addr_mode=self.IMM, cycles=2)
		self.lookup[0x6A] = Instruction('ROR', opcode=self.ROR, addr_mode=self.IMP, cycles=2)
		self.lookup[0x6C] = Instruction('JMP', opcode=self.JMP, addr_mode=self.IND, cycles=5)
		self.lookup[0x6D] = Instruction('ADC', opcode=self.ADC, addr_mode=self.ABS, cycles=4)
		self.lookup[0x6E] = Instruction('ROR', opcode=self.ROR, addr_mode=self.ABS, cycles=6)
		self.lookup[0x70] = Instruction('BVS', opcode=self.BVS, addr_mode=self.REL, cycles=2)
		self.lookup[0x71] = Instruction('ADC', opcode=self.ADC, addr_mode=self.IZY, cycles=5)
		self.lookup[0x75] = Instruction('ADC', opcode=self.ADC, addr_mode=self.ZPX, cycles=4)
		self.lookup[0x76] = Instruction('ROR', opcode=self.ROR, addr_mode=self.ZPX, cycles=6)
		self.lookup[0x78] = Instruction('SEI', opcode=self.SEI, addr_mode=self.IMP, cycles=2)
		self.lookup[0x79] = Instruction('ADC', opcode=self.ADC, addr_mode=self.ABY, cycles=4)
		self.lookup[0x7D] = Instruction('ADC', opcode=self.ADC, addr_mode=self.ABX, cycles=4)
		self.lookup[0x7E] = Instruction('ROR', opcode=self.ROR, addr_mode=self.ABX, cycles=7)
		self.lookup[0x81] = Instruction('STA', opcode=self.STA, addr_mode=self.IZX, cycles=6)
		self.lookup[0x84] = Instruction('STY', opcode=self.STY, addr_mode=self.ZP0, cycles=3)
		self.lookup[0x85] = Instruction('STA', opcode=self.STA, addr_mode=self.ZP0, cycles=3)
		self.lookup[0x86] = Instruction('STX', opcode=self.STX, addr_mode=self.ZP0, cycles=3)
		self.lookup[0x88] = Instruction('DEY', opcode=self.DEY, addr_mode=self.IMP, cycles=2)
		self.lookup[0x8A] = Instruction('TXA', opcode=self.TXA, addr_mode=self.IMP, cycles=2)
		self.lookup[0x8C] = Instruction('STY', opcode=self.STY, addr_mode=self.ABS, cycles=4)
		self.lookup[0x8D] = Instruction('STA', opcode=self.STA, addr_mode=self.ABS, cycles=4)
		self.lookup[0x8E] = Instruction('STX', opcode=self.STX, addr_mode=self.ABS, cycles=4)
		self.lookup[0x90] = Instruction('BCC', opcode=self.BCC, addr_mode=self.REL, cycles=2)
		self.lookup[0x91] = Instruction('STA', opcode=self.STA, addr_mode=self.IZY, cycles=6)
		self.lookup[0x94] = Instruction('STY', opcode=self.STY, addr_mode=self.ZPX, cycles=4)
		self.lookup[0x95] = Instruction('STA', opcode=self.STA, addr_mode=self.ZPX, cycles=4)
		self.lookup[0x96] = Instruction('STX', opcode=self.STX, addr_mode=self.ZPY, cycles=4)
		self.lookup[0x98] = Instruction('TYA', opcode=self.TYA, addr_mode=self.IMP, cycles=2)
		self.lookup[0x99] = Instruction('STA', opcode=self.STA, addr_mode=self.ABY, cycles=5)
		self.lookup[0x9A] = Instruction('TXS', opcode=self.TXS, addr_mode=self.IMP, cycles=2)
		self.lookup[0x9D] = Instruction('STA', opcode=self.STA, addr_mode=self.ABX, cycles=5)
		self.lookup[0xA0] = Instruction('LDY', opcode=self.LDY, addr_mode=self.IMM, cycles=2)
		self.lookup[0xA1] = Instruction('LDA', opcode=self.LDA, addr_mode=self.IZX, cycles=6)
		self.lookup[0xA2] = Instruction('LDX', opcode=self.LDX, addr_mode=self.IMM, cycles=2)
		self.lookup[0xA4] = Instruction('LDY', opcode=self.LDY, addr_mode=self.ZP0, cycles=3)
		self.lookup[0xA5] = Instruction('LDA', opcode=self.LDA, addr_mode=self.ZP0, cycles=3)
		self.lookup[0xA6] = Instruction('LDX', opcode=self.LDX, addr_mode=self.ZP0, cycles=3)
		self.lookup[0xA8] = Instruction('TAY', opcode=self.TAY, addr_mode=self.IMP, cycles=2)
		self.lookup[0xA9] = Instruction('LDA', opcode=self.LDA, addr_mode=self.IMM, cycles=2)
		self.lookup[0xAA] = Instruction('TAX', opcode=self.TAX, addr_mode=self.IMP, cycles=2)
		self.lookup[0xAC] = Instruction('LDY', opcode=self.LDY, addr_mode=self.ABS, cycles=4)
		self.lookup[0xAD] = Instruction('LDA', opcode=self.LDA, addr_mode=self.ABS, cycles=4)
		self.lookup[0xAE] = Instruction('LDX', opcode=self.LDX, addr_mode=self.ABS, cycles=4)
		self.lookup[0xB0] = Instruction('BCS', opcode=self.BCS, addr_mode=self.REL, cycles=2)
		self.lookup[0xB1] = Instruction('LDA', opcode=self.LDA, addr_mode=self.IZY, cycles=5)
		self.lookup[0xB4] = Instruction('LDY', opcode=self.LDY, addr_mode=self.ZPX, cycles=4)
		self.lookup[0xB5] = Instruction('LDA', opcode=self.LDA, addr_mode=self.ZPX, cycles=4)
		self.lookup[0xB6] = Instruction('LDX', opcode=self.LDX, addr_mode=self.ZPY, cycles=4)
		self.lookup[0xB8] = Instruction('CLV', opcode=self.CLV, addr_mode=self.IMP, cycles=2)
		self.lookup[0xB9] = Instruction('LDA', opcode=self.LDA, addr_mode=self.ABY, cycles=4)
		self.lookup[0xBA] = Instruction('TSX', opcode=self.TSX, addr_mode=self.IMP, cycles=2)
		self.lookup[0xBC] = Instruction('LDY', opcode=self.LDY, addr_mode=self.ABX, cycles=4)
		self.lookup[0xBD] = Instruction('LDA', opcode=self.LDA, addr_mode=self.ABX, cycles=4)
		self.lookup[0xBE] = Instruction('LDX', opcode=self.LDX, addr_mode=self.ABY, cycles=4)
		self.lookup[0xC0] = Instruction('CPY', opcode=self.CPY, addr_mode=self.IMM, cycles=2)
		self.lookup[0xC1] = Instruction('CMP', opcode=self.CMP, addr_mode=self.IZX, cycles=6)
		self.lookup[0xC4] = Instruction('CPY', opcode=self.CPY, addr_mode=self.ZP0, cycles=3)
		self.lookup[0xC5] = Instruction('CMP', opcode=self.CMP, addr_mode=self.ZP0, cycles=3)
		self.lookup[0xC6] = Instruction('DEC', opcode=self.DEC, addr_mode=self.ZP0, cycles=5)
		self.lookup[0xC8] = Instruction('INY', opcode=self.INY, addr_mode=self.IMP, cycles=2)
		self.lookup[0xC9] = Instruction('CMP', opcode=self.CMP, addr_mode=self.IMM, cycles=2)
		self.lookup[0xCA] = Instruction('DEX', opcode=self.DEX, addr_mode=self.IMP, cycles=2)
		self.lookup[0xCC] = Instruction('CPY', opcode=self.CPY, addr_mode=self.ABS, cycles=4)
		self.lookup[0xCD] = Instruction('CMP', opcode=self.CMP, addr_mode=self.ABS, cycles=4)
		self.lookup[0xCE] = Instruction('DEC', opcode=self.DEC, addr_mode=self.ABS, cycles=6)
		self.lookup[0xD0] = Instruction('BNE', opcode=self.BNE, addr_mode=self.REL, cycles=2)
		self.lookup[0xD1] = Instruction('CMP', opcode=self.CMP, addr_mode=self.IZY, cycles=5)
		self.lookup[0xD5] = Instruction('CMP', opcode=self.CMP, addr_mode=self.ZPX, cycles=4)
		self.lookup[0xD6] = Instruction('DEC', opcode=self.DEC, addr_mode=self.ZPX, cycles=6)
		self.lookup[0xD8] = Instruction('CLD', opcode=self.CLD, addr_mode=self.IMP, cycles=2)
		self.lookup[0xD9] = Instruction('CMP', opcode=self.CMP, addr_mode=self.ABY, cycles=4)
		self.lookup[0xDD] = Instruction('CMP', opcode=self.CMP, addr_mode=self.ABX, cycles=4)
		self.lookup[0xDE] = Instruction('DEC', opcode=self.DEC, addr_mode=self.ABX, cycles=7)
		self.lookup[0xE0] = Instruction('CPX', opcode=self.CPX, addr_mode=self.IMM, cycles=2)
		self.lookup[0xE1] = Instruction('SBC', opcode=self.SBC, addr_mode=self.IZX, cycles=6)
		self.lookup[0xE4] = Instruction('CPX', opcode=self.CPX, addr_mode=self.ZP0, cycles=3)
		self.lookup[0xE5] = Instruction('SBC', opcode=self.SBC, addr_mode=self.ZP0, cycles=3)
		self.lookup[0xE6] = Instruction('INC', opcode=self.INC, addr_mode=self.ZP0, cycles=5)
		self.lookup[0xE8] = Instruction('INX', opcode=self.INX, addr_mode=self.IMP, cycles=2)
		self.lookup[0xE9] = Instruction('SBC', opcode=self.SBC, addr_mode=self.IMM, cycles=2)
		self.lookup[0xEA] = Instruction('NOP', opcode=self.NOP, addr_mode=self.IMP, cycles=2)
		self.lookup[0xEC] = Instruction('CPX', opcode=self.CPX, addr_mode=self.ABS, cycles=4)
		self.lookup[0xED] = Instruction('SBC', opcode=self.SBC, addr_mode=self.ABS, cycles=4)
		self.lookup[0xEE] = Instruction('INC', opcode=self.INC, addr_mode=self.ABS, cycles=6)
		self.lookup[0xF0] = Instruction('BEQ', opcode=self.BEQ, addr_mode=self.REL, cycles=2)
		self.lookup[0xF1] = Instruction('SBC', opcode=self.SBC, addr_mode=self.IZY, cycles=5)
		self.lookup[0xF5] = Instruction('SBC', opcode=self.SBC, addr_mode=self.ZPX, cycles=4)
		self.lookup[0xF6] = Instruction('INC', opcode=self.INC, addr_mode=self.ZPX, cycles=6)
		self.lookup[0xF8] = Instruction('SED', opcode=self.SED, addr_mode=self.IMP, cycles=2)
		self.lookup[0xF9] = Instruction('SBC', opcode=self.SBC, addr_mode=self.ABY, cycles=4)
		self.lookup[0xFD] = Instruction('SBC', opcode=self.SBC, addr_mode=self.ABX, cycles=4)
		self.lookup[0xFE] = Instruction('INC', opcode=self.INC, addr_mode=self.ABX, cycles=7)

	def connect_bus(self, bus: Bus):
		self.bus = bus

	def write(self, addr: np.uint16, data: np.uint8):
		self.bus.write(addr, data)

	def read(self, addr: np.uint16):
		return self.bus.read(addr)

	def get_flag(self, f: flags):
		return self.status & f.value

	def set_flag(self, f: flags, v: bool):
		if v:
			self.status |= f.value
		else:
			self.status &= ~f.value

	def set_irq(self, v):
		self._interrupt = True if v else False

	def set_nmi(self, v):
		self._nm_interrupt = True if v else False

	# Reset
	def reset(self):
		self.x = 0x00
		self.y = 0x00
		self.a = 0x00
		self.stkp = 0xFD
		self.status = 0x70

		# Obtain pc address from location 0xFFFC
		self._addr_abs = 0xFFFC
		lo = self.read(self._addr_abs)
		hi = self.read(self._addr_abs+1)

		self.pc = (hi<<8) | lo

		self._cycles = 8

	# Servicing maskable interrupts
	def irq(self):
		if self.get_flag(flags.I) == 0:	# Interrupts allowed
			self.write(0x100 + self.stkp, (self.pc>>8) & 0xFF)	# Store high byte
			self.stkp-=1
			self.write(0x100 + self.stkp, self.pc & 0xFF)		# Store low byte
			self.stkp-=1

			self.set_flag(self.B, 1)
			self.set_flag(self.U, 1)
			self.set_flag(self.I, 1)	# Cannot interrupt while servicing interrupt

			# Store status registor in stack
			self.write(0x100 + self.stkp, self.status & 0xFF)		# Store low byte
			self.stkp-=1

			self._addr_abs = 0xFFFE
			hi = self.read(self._addr_abs)
			lo = self.read(self._addr_abs + 1)
			self.pc = (hi<<8) | lo

			self._cycles = 7

	# Servicing non-maskable interrupts
	def nmi(self):
		self.write(0x100 + self.stkp, (self.pc>>8) & 0xFF)	# Store high byte
		self.stkp-=1
		self.write(0x100 + self.stkp, self.pc & 0xFF)		# Store low byte
		self.stkp-=1

		self.set_flag(self.B, 1)
		self.set_flag(self.U, 1)
		self.set_flag(self.I, 1)	# Cannot interrupt while servicing interrupt

		# Store status registor in stack
		self.write(0x100 + self.stkp, self.status & 0xFF)		# Store low byte
		self.stkp-=1

		self._addr_abs = 0xFFFA
		hi = self.read(self._addr_abs)
		lo = self.read(self._addr_abs + 1)
		self.pc = (hi<<8) | lo

		self._cycles = 8

	# Triggers clock cycle
	def clock(self):
		if self._cycles == 0: 	# Next instruction ready
			# Service interrupts first
			if self._nm_interrupt:
				self.nmi()
				self._nm_interrupt = False

			if self._interrupt:
				self.irq()
				self._interrupt = False

			# Read opcode from memory
			# pdb.set_trace()
			self.opcode = self.read(self.pc)
			self.opcode&=0xFF
			self.pc+=1

			# Get appropriate instruction
			instruction = self.lookup[self.opcode]
			
			if self._debugging_mode:
				print(hex(self.opcode), instruction.name)
				print(hex(self.pc-1))
				print('\n')

			if self.pc == 0x800a+1:
				pdb.set_trace()
			# Get absolute address
			additional_clock_cycle_1 = (instruction.addr_mode)()

			# Run instruction 
			additional_clock_cycle_2 = (instruction.opcode)()

			# Store number of required clock cycles
			self._cycles = instruction.cycles
			# self._cycles = instruction.cycles + (additional_clock_cycle_1 & additional_clock_cycle_2)

			# Always set unused flag
			self.set_flag(flags.U, 1)

		else:
			self._cycles-=1

	# Print CPU internal state
	def __str__(self):
		return 	f'a: 		{hex(self.a)} \n' \
				+ f'x: 		{hex(self.x)} \n' \
				+ f'y: 		{hex(self.y)} \n' \
				+ f'stkp: 	{hex(self.stkp)} \n' \
				+ f'pc: 	{hex(self.pc)} \n' \
				+ f'status: {hex(self.status)} \n'

	# Instruction addressing modes
	"""
	Implied - The location of the data to be manipulated is implicit. For instance, 
	set carry flag.
	"""
	def IMP(self):
		self._fetched = self.a

	"""
	Immediate - The data is contained within the second byte of the instruction.
	Hence, the address is set to the next byte in program memory.
	"""
	def IMM(self):
		self._addr_abs = self.pc
		self.pc+=1
		return 0

	"""
	Zero Page Adressing (Zero Page) - Assumes data is in first page of memory. Hence, only the 
	second byte is fetched from the second byte of the instruction.
	"""
	def ZP0(self):
		self._addr_abs = self.read(self.pc)
		self.pc+=1
		self._addr_abs &= 0x00FF
		return 0

	"""
	Zero Page Adressing with x regisiter offset - Assumes data is located on first page with
	the second byte of the data given by the second byte in the instruction. The data address is
	offset by the amount in the x register.
	"""
	def ZPX(self):
		self._addr_abs = self.read(self.pc) + self.x
		self.pc+=1
		self._addr_abs &= 0x00FF
		return 0

	"""
	Zero Page Adressing with y regisiter offset - Assumes data is located on first page with
	the second byte of the data given by the second byte in the instruction. The data address is
	offset by the amount in the y register.
	"""
	def ZPY(self):
		self._addr_abs = self.read(self.pc) + self.y
		self.pc+=1
		self._addr_abs &= 0x00FF
		return 0

	"""
	Relative Adressing - Second byte of instruction branch specifies the pc offset. Value 
	must remain within -128 to +127 bytes of branch instruction.
	"""
	def REL(self):
		self._addr_rel = self.read(self.pc)
		self.pc+=1

		if self._addr_rel > 127:
			self._addr_rel -= 256

		return 0
	
	"""
	Absolute Adressing - Second byte of instruction specifies low-order bytes of address byte. 
	Third byte of instruction specifies high-order address byte.
	"""
	def ABS(self):
		self._addr_abs = self.read(self.pc) & 0x00FF	# Low byte
		self.pc+=1
		self._addr_abs |= (self.read(self.pc)<<8)	# High byte
		self.pc+=1
		return 0


	"""
	Absolute X Indexed Adressing - Same as absolute addressing except the contents of register x 
	are added to the absolute address. If page changes, the addition clock cycle is required.
	"""
	def ABX(self):
		print(hex(self.opcode))
		self._addr_abs = self.read(self.pc) & 0x00FF	# Low byte
		self.pc+=1
		hi = (self.read(self.pc)<<8) 	# High byte
		self._addr_abs |= hi
		self.pc+=1
		self._addr_abs += self.x
		if (self.addr_mode & 0xFF00) != hi:
			return 1 	# Additional cycle required
		return 0

	"""
	Absolute Y Indexed Adressing - Same as absolute addressing except the contents of register y 
	are added to the absolute address. If page changes, the addition clock cycle is required.
	"""
	def ABY(self):
		self._addr_abs = read(self.pc) & 0x00FF	# Low byte
		self.pc+=1
		hi = (read(self.pc)<<8) 	# High byte
		self._addr_abs |= hi
		self.pc+=1
		self._addr_abs += self.y
		if (self.addr_mode & 0xFF00) != hi:
			return 1 	# Additional cycle required
		return 0

	"""
	Indirect Adressing - Next two bytes containt the address of the locaiton in memory which 
	contains the actual data. The first byte is the low byte, the second is the high.
	"""
	def IND(self):
		ptr_lo = self.read(self.pc)
		self.pc+=1
		ptr_hi = self.read(self.pc)
		self.pc+=1
		
		ptr = (ptr_hi<<8) | ptr_lo

		if ptr_lo == 0x00FF:
			self._addr_abs = (read(ptr & 0xFF00)<<8) + read(ptr)
		else:
			self._addr_abs = (read(ptr+1)<<8) + read(ptr)

		return 0

	"""
	Indirect Adressing X - Next byte of instruction specifies address low byte on first page.
	The address of the high byte is the next byte over on the first page.
	"""
	def IZX(self):
		t = self.read(self.pc)
		self.pc+=1
		lo = read((t + self.x) & 0x00FF) 		# And discards carry
		hi = read((t + self.x + 1) & 0x00FF) 	# And discards carry
		self._addr_abs = (hi<<8) | lo
		return 0

	"""
	Indirect Adressing Y - Same as previous instruction except contents of y register are added
	after the absolute address is constructed from the first page. If page boundary is crossed, an
	additional clock cycle is required.
	"""
	def IZY(self):
		t = self.read(self.pc)
		self.pc+=1
		lo = read(t & 0x00FF)
		hi = read((t+1) & 0x00FF)

		self._addr_abs = (hi<<8) | lo
		self._addr_abs += y
		
		if self._addr_abs & 0xFF00 != (hi<<8):	# Crossed page boundary
			return 1
		return 0
	
	def fetch(self):
		if self.lookup[self.opcode] != self.IMP:
			self._fetched = self.read(self._addr_abs)

	# Instruction opcodes
	"""
	ADC - Add memory to accumulator with carry.
	A + M + C -> A, C
	Falgs: N, V, Z, C
	"""
	def ADC(self):
		self.fetch()
		temp = self.a + self._fetched + self.get_flag(flags.C)
		self.set_flag(flags.C, temp & 0xFF00)
		self.set_flag(flags.Z, (temp&0xFF) == 0x00)
		self.set_flag(flags.N, temp & 0x80)
		self.set_flag(flags.V, (temp > 127) | (temp < -128) )
		self.a = temp&0xFF

	"""
	AND - Second byte is data, "AND" with data in memory with accumulator.
	A and M -> A
	Flags: N, Z
	"""
	def AND(self):
		self.fetch()
		self.a &= self._fetched
		self.set_flag(flags.Z, a == 0x00)	# Set zero flag
		self.set_flag(flags.N, a & 0x80)	# Set negative flag
		# return 1

	"""
	ASL - Arithmetic shift left. 
	C <- /M7...M0/ <- 0
	Flags: C, Z, N
	"""
	def ASL(self):
		self.fetch()
		temp = self._fetched << 1
		self.set_flag(flags.C, temp & 0xFF00)
		self.set_flag(flags.Z, (temp&0xFF) == 0x00)
		self.set_flag(flags.N, temp&0x80)

		# If addressing mode implicit then accumulator is changed, otherwise the result is written to memory
		if self.lookup[self.opcode].addr_mode == self.IMP:
			self.a = temp & 0xFF
		else:
			self.write(self._addr_abs, temp & 0xFF)

		# return 0

	"""
	BCC - Branches if carry bit not set. Asboliute branching address set by relative distance from pc.
	Only has relative addressing mode.
	Requires addition clock cycle if branch taken. 
	Require additional clock cycle if page boundary crossed.
	Flags: -
	"""
	def BCC(self):
		if self.get_flag(flags.C) == 0:
			self._cycles+=1
			self._addr_abs = self.pc + self._addr_rel

			if (self._addr_abs&0xFF00) != (self.pc&0xFF00):	# If page boundary crossed
				self._cycles+=1

			self.pc = self._addr_abs

		# return 0

	"""
	BCS - Branches if carry bit set. Asboliute branching address set by relative distance from pc.
	Only has relative addressing mode.
	Requires addition clock cycle if branch taken. 
	Require additional clock cycle if page boundary crossed.
	Flags: -
	"""
	def BCS(self):
		if self.get_flag(flags.C) == 1:
			self._cycles+=1
			self._addr_abs = self.pc + self._addr_rel

			if (self._addr_abs&0xFF00) != (self.pc&0xFF00):	# If page boundary crossed
				self._cycles+=1

			self.pc = self._addr_abs

		# return 0


	"""
	BEQ - Branches if zero flag set (Branch on Equal Zero). 
	Asbolute branching address set by relative distance from pc.
	Requires addition clock cycle if branch occurs. 
	Require additional clock cycle if page boundary crossed.
	Flags: -
	"""
	def BEQ(self):
		if self.get_flag(flags.Z) == 1:
			self._cycles+=1
			self._addr_abs = self.pc + self._addr_rel

			if (self._addr_abs&0xFF00) != (self.pc&0xFF00):	# If page boundary crossed
				self._cycles+=1

			self.pc = self._addr_abs


	"""
	BIT - Test bits in memory with accumulator.
	A and M
	Flags: N, V
	"""
	def BIT(self):
		self.fetch()
		temp = self.a & self._fetched
		self.set_flag(flags.Z, temp == 0x00)	# Set zero flag
		self.set_flag(flags.N, temp & 0x80)		# Set negative flag

	"""
	BMI - Branch on result minus.
	Flags: -
	"""
	def BMI(self):
		if self.get_flag(flags.N) == 1:
			self._cycles+=1
			self._addr_abs = self.pc + self._addr_rel

			if (self._addr_abs&0xFF00) != (self.pc&0xFF00):	# If page boundary crossed
				self._cycles+=1

			self.pc = self._addr_abs

	"""
	BNE - Branch on result not zero (Branch on Not Equal)
	Flags: -
	"""
	def BNE(self):
		if self.get_flag(flags.Z) == 0:
			self._cycles+=1
			self._addr_abs = self.pc + self._addr_rel

			if (self._addr_abs&0xFF00) != (self.pc&0xFF00):	# If page boundary crossed
				self._cycles+=1

			self.pc = self._addr_abs

	"""
	BPL - Branch on Result Plus
	Flags: -
	"""
	def BPL(self):
		if self.get_flag(flags.N) == 0:
			self._cycles+=1
			self._addr_abs = self.pc + self._addr_rel

			if (self._addr_abs&0xFF00) != (self.pc&0xFF00):	# If page boundary crossed
				self._cycles+=1

			self.pc = self._addr_abs

	"""
	BRK - Break Command
	Pushes program counter and status register on stack in preparation for interript vector.
	Flags: I <- 1
	"""
	def BRK(self):
		self.write(0x100 + self.stkp, (self.pc>>8) & 0xFF)	# Store high byte
		self.stkp-=1
		self.write(0x100 + self.stkp, self.pc & 0xFF)		# Store low byte
		self.stkp-=1

		self.set_flag(flags.I, 1)

		# Store status registor in stack
		self.write(0x100 + self.stkp, self.status & 0xFF)		# Store low byte
		self.stkp-=1

		self._addr_abs = 0xFFFE
		hi = self.read(self._addr_abs)
		lo = self.read(self._addr_abs + 1)
		self.pc = (hi<<8) | lo

	"""
	BVC - Branch on Overflow Clear
	Flags: -
	"""
	def BVC(self):
		if self.get_flag(flags.V) == 0:
			self._cycles+=1
			self._addr_abs = self.pc + self._addr_rel

			if (self._addr_abs&0xFF00) != (self.pc&0xFF00):	# If page boundary crossed
				self._cycles+=1

			self.pc = self._addr_abs

	"""
	BVS - Branch on Overflow Set
	Flags: -
	"""
	def BVS(self):
		if self.get_flag(flags.V) == 1:
			self._cycles+=1
			self._addr_abs = self.pc + self._addr_rel

			if (self._addr_abs&0xFF00) != (self.pc&0xFF00):	# If page boundary crossed
				self._cycles+=1

			self.pc = self._addr_abs
	
	"""
	CLC - Clears carry flag
	C <- 0
	Flags: C <- 0
	"""
	def CLC(self):
		self.set_flag(flags.C, 0)

	"""
	CLD - Clear decimal mode
	D <- 0
	Flags: D <- 0
	"""
	def CLD(self):
		self.set_flag(flags.D, 0)

	"""
	CID - Clear interrupt disable bit
	I <- 0
	Flags: I <- 0
	"""
	def CLI(self):
		self.set_flag(flags.I, 0)

	"""
	CLV - Clear overflow flag
	V <- 0
	Flags: V <- 0
	"""
	def CLV(self):
		self.set_flag(flags.V, 0)

	"""
	CMP - Compares accumulator with memory. 
	A-M 
	Flags: C, Z, N
	"""
	def CMP(self):
		self.fetch()
		temp = self.a - self._fetched
		self.set_flag(flags.C, self.a >= self._fetched)
		self.set_flag(flags.Z, (temp&0xFF) == 0x00)
		self.set_flag(flags.N, temp & 0x80)

	"""
	CPX  - Compares X register with memory location
	X-M 
	Flags: C, Z, N
	"""
	def CPX(self):
		self.fetch()
		temp = self.x - self._fetched
		self.set_flag(flags.C, self.x >= self._fetched)
		self.set_flag(flags.Z, (temp&0xFF) == 0x00)
		self.set_flag(flags.N, temp & 0x80)
	
	"""
	CPY  - Compares Y register with memory location
	Y-M 
	Flags: C, Z, N
	"""	
	def CPY(self):
		self.fetch()
		temp = self.y - self._fetched
		self.set_flag(flags.C, self.y >= self._fetched)
		self.set_flag(flags.Z, (temp&0xFF) == 0x00)
		self.set_flag(flags.N, temp & 0x80)
	
	"""
	DEC - Decrement Memory By One
	M <- M-1
	Flags: N, Z
	"""
	def DEC(self):
		self.fetch()
		temp = self._fetched - 1
		self.set_flag(flags.Z, (temp&0xFF) == 0)
		self.set_flag(flags.N, temp & 0x80)
		self.write(self._addr_abs, temp&0xFF)

	"""
	DEX - Decrement Index Register X By One
	X <- X-1
	Flags: N, Z
	"""
	def DEX(self):
		self.x-=1
		self.x&=0xFF
		self.set_flag(flags.Z, self.x == 0)
		self.set_flag(flags.N, self.x & 0x80)

	"""
	DEY - Decrement Index Register Y By One
	Y <- Y-1
	Flags: N, Z
	"""
	def DEY(self):
		self.y-=1
		self.set_flag(flags.Z, self.y == 0)
		self.set_flag(flags.N, self.y & 0x80)

	"""
	EOR - "Exclusive OR" Memory with Accumulator
	A <- A ^ M
	Flags: N, Z
	"""
	def EOR(self):
		self.fetch()
		temp = self._fetched ^ self.a
		self.a = 0xFF & temp
		self.set_flag(flags.Z, self.a == 0)
		self.set_flag(flags.N, self.a & 0x80)

	"""
	INC - Increment Memory By One
	M <- M+1
	Flags: N, Z
	"""
	def INC(self):
		self.fetch()
		temp = self._fetched + 1
		temp&=0xFF
		self.write(self._addr_abs, temp)
		self.set_flag(flags.Z, temp == 0)
		self.set_flag(flags.N, temp&0x80)

	"""
	INX - Increment Index Register X By One
	X <- X+1
	Flags: N, Z
	"""
	def INX(self):
		self.x+=1
		self.x&=0xFF
		self.set_flag(flags.Z, self.x == 0)
		self.set_flag(flags.N, self.x&0x80)

	"""
	INY - Increment Index Register Y By One
	Y <- Y+1
	Flags: N, Z
	"""
	def INY(self):
		self.y+=1
		self.y&=0xFF
		self.set_flag(flags.Z, self.y == 0)
		self.set_flag(flags.N, self.y&0x80)

	"""
	JMP - JMP Indirect
	Jumps to new location without pushing old pc to stack
	[PC + 1] → PCL, [PC + 2] → PCH
	Flags: -
	"""
	def JMP(self):
		self.pc = self._addr_abs
	
	"""
	JSR - Jump To Subroutine
	Jumps to new location after pushing old pc to stack
	PC + 2↓, [PC + 1] → PCL, [PC + 2] → PCH
	Flags: -
	"""
	def JSR(self):
		self.pc-=1
		self.write(0x100 + self.stkp, (self.pc>>8) & 0xFF)	# Store high byte
		self.stkp-=1
		self.write(0x100 + self.stkp, self.pc & 0xFF)		# Store low byte
		self.stkp-=1

		self.pc = self._addr_abs

	"""
	LDA - Load Accumulator with Memory
	A <- M
	Flags: N, Z
	"""
	def LDA(self):
		self.fetch()
		self.a = self._fetched & 0xFF
		self.set_flag(flags.Z, self.a == 0)
		self.set_flag(flags.N, self.a & 0x80)

	"""
	LDX - Load Accumulator with Memory
	X <- M
	Flags: N, Z
	"""
	def LDX(self):
		self.fetch()
		self.x = self._fetched & 0xFF
		self.set_flag(flags.Z, self.x == 0)
		self.set_flag(flags.N, self.x & 0x80)
		return 1

	"""
	LDY - Load Accumulator with Memory
	Y <- M
	Flags: N, Z
	"""
	def LDY(self):
		self.fetch()
		self.y = self._fetched & 0xFF
		self.set_flag(flags.Z, self.y == 0)
		self.set_flag(flags.N, self.y & 0x80)
		return 1

	"""
	LSR - Logical Shift Right
	0 → /M7...M0/ → C
	Flags: N, Z, C
	"""
	def LSR(self):
		self.fetch()
		self.set_flag(flags.C, self._fetched & 0x01)
		self._fetched >>= 1

		if self.lookup[self.opcode].addr_mode == self.IMP:
			self.a = self._fetched
		else:
			self.write(self._addr_abs, self._fetched)
			
		self.set_flag(flags.Z, self._fetched == 0)
		self.set_flag(flags.N, self._fetched & 0x80)


	"""
	NOP - No Operation
	"""
	def NOP(self): ...

	"""
	ORA - "OR" Memory with Accumulator
	A ∨ M → A
	Flags: -
	"""
	def ORA(self):
		self.fetch()
		self.a = (self.a&0xFF) | (self._fetched&0xFF)
		self.set_flag(flags.N, self.a & 0x80)
		self.set_flag(flags.Z, self.a == 0)

	"""
	PHA - Push Accumulator On Stack
	A↓
	Flags: -
	"""
	def PHA(self):
		self.write(0x100 + self.stkp, self.a & 0xFF)		# Store low byte
		self.stkp-=1

	"""
	PHP - Push Processor Status On Stack
	P↓
	Flags: -
	"""
	def PHP(self):
		self.write(0x100 + self.stkp, self.status & 0xFF)		# Store low byte
		self.stkp-=1

	"""
	PLA - Pull Accumulator From Stack
	A↑
	Flags: N, Z
	"""
	def PLA(self):
		self.stkp+=1
		self.a = self.read(0x100 + self.stkp)		# Store low byte
		self.set_flag(flags.Z, self.a==0)
		self.set_flag(flags.N, self.a&0x80)

	"""
	PLP - Pull Processor Status From Stack
	P↑
	Flags: N, Z
	"""
	def PLP(self):
		self.stkp+=1
		self.status = self.read(0x100 + self.stkp)		# Store low byte
		self.set_flag(flags.Z, self.a==0)
		self.set_flag(flags.N, self.a&0x80)

	"""
	ROL - Rotate Left
	C ← /M7...M0/ ← C
	Flags: N, Z, C
	"""
	def ROL(self):
		self.fetch()
		self._fetched <<= 1
		self._fetched |= flags.C

		self.set_flag(flags.C, self._fetched & 0x100)
		self._fetched &= 0xFF
		self.set_flag(flags.Z, self._fetched == 0)
		self.set_flag(flags.N, self._fetched & 0x80)

		if lookup[self.opcode].addr_mode == self.IMP:
			self.a = self._fetched
		else:
			self.write(self._addr_abs, self._fetched)

	"""
	ROR - Rotate Right
	C → /M7...M0/ → C
	Flags: N, Z, C
	"""
	def ROR(self):
		self.fetch()
		self.set_flag(flags.C, self._fetched & 0x01)
		self._fetched >>= 1
		self._fetched |= (flags.C<<7)

		self._fetched &= 0xFF
		self.set_flag(flags.Z, self._fetched == 0)
		self.set_flag(flags.N, self._fetched & 0x80)

		if lookup[self.opcode].addr_mode == self.IMP:
			self.a = self._fetched
		else:
			self.write(self._addr_abs, self._fetched)

	"""
	RTI - Return From Interrupt
	Called at the end of interrupt vector to restore cpu state
	P↑ PC↑
	Flags: N, V, D, I, Z, C
	"""
	def RTI(self):
		self.stkp+=1
		self.status = self.write(0x100 + self.stkp)

		self.stkp+=1
		lo = self.read(0x100 + self.stkp)
		self.stkp+=1
		hi = self.read(0x100 + self.stkp, self.pc & 0xFF)
		self.pc = (hi<<8)|lo


	"""
	RTS - Return From Subroutme
	Restor pc from JSR instruction
	PC↑, PC + 1 → PC
	Flags: -
	"""
	def RTS(self):
		self.stkp+=1
		lo = self.read(0x100 + self.stkp)
		self.stkp+=1
		hi = self.read(0x100 + self.stkp, self.pc & 0xFF)
		self.pc = (hi<<8)|lo
		self.pc+=1

	"""
	SBC - Subtract Memory from Accumulator with Borrow
	A - M - ~C → A
	Flags: N, V, Z, C
	"""
	def SBC(self):
		self.fetch()
		temp = self.a - self._fetched - self.get_flag(flags.C)
		self.set_flag(flags.C, temp & 0xFF00)
		self.set_flag(flags.Z, (temp&0xFF) == 0x00)
		self.set_flag(flags.N, temp & 0x80)
		self.a = temp&0xFF
		self.set_flag(flags.V, (temp > 127) | (temp < -128) )

	"""
	SEC - Set Carry Flag
	1 → C
	Flags: C <- 1
	"""
	def SEC(self):
		self.set_flag(flags.C, 1)

	"""
	SEC - Set Decimal Mode
	1 → D
	Flags: D <- 1
	"""
	def SED(self):
		self.set_flag(flags.D, 1)

	"""
	SEC - Set Interrupt Disable
	1 → I
	Flags: I <- 1
	"""
	def SEI(self):
		self.set_flag(flags.I, 1)

	"""
	STA - Store Accumulator in Memory
	A → M
	Flags: -
	"""
	def STA(self):
		self.write(self._addr_abs, self.a)

	"""
	STX - Store Index Register X In Memory
	X → M
	Flags: -
	"""
	def STX(self):
		self.write(self._addr_abs, self.x)

	"""
	STY - Store Index Register Y In Memory
	Y → M
	Flags: -
	"""
	def STY(self):
		self.write(self._addr_abs, self.y)

	"""
	STZ - Store Zero Flag in Memory
	Z → M
	Flags: -
	"""
	def STZ(self):
		self.write(self._addr_abs, flags.Z)

	"""
	TAX - Transfer Accumulator To Index X
	A → X
	Flags: N, Z
	"""
	def TAX(self):
		self.x = self.a
		self.set_flag(flags.N, self.x & 0x80)
		self.set_flag(flags.Z, self.x == 0)

	"""
	TAX - Transfer Accumulator To Index Y
	A → Y
	Flags: N, Z
	"""
	def TAY(self):
		self.y = self.a
		self.set_flag(flags.N, self.y & 0x80)
		self.set_flag(flags.Z, self.y == 0)

	"""
	TSX - Transfer Stack Pointer To Index X
	S → X
	Flags: N, Z
	"""
	def TSX(self):
		self.x = self.stkp
		self.set_flag(flags.N, self.x & 0x80)
		self.set_flag(flags.Z, self.x == 0)

	"""
	TXA - Transfer Index X To Accumulator
	X → A
	Flags: N, Z
	"""
	def TXA(self):
		self.a = self.x
		self.set_flag(flags.N, self.a & 0x80)
		self.set_flag(flags.Z, self.a == 0)

	"""
	TXS - Transfer Index X To Stack Pointer
	X → S
	Flags: N, Z
	"""
	def TXS(self):
		self.stkp = self.x
		self.set_flag(flags.N, self.stkp & 0x80)
		self.set_flag(flags.Z, self.stkp == 0)

	"""
	TYA - Transfer Index Y To Accumulator
	Y → A
	Flags: N, Z
	"""
	def TYA(self):
		self.a = self.y
		self.set_flag(flags.N, self.a & 0x80)
		self.set_flag(flags.Z, self.a == 0)

	# Invalid instruction
	def XXX(self):
		print('???')
