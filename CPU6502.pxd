cdef extern from "stdint.h":
	ctypedef unsigned char uint8_t
	ctypedef unsigned short uint16_t

from .Bus cimport Bus


cdef class CPU6502:
	"""
	Implements high level 6502 CPU. 
	Datasheet: http://datasheets.chipdb.org/Rockwell/6502.pdf
	Instruction Set: https://www.pagetable.com/c64ref/6502/?tab=0
	Assembler: https://www.masswerk.at/6502/assembler.html
	"""
	cdef uint8_t a
	cdef uint8_t x
	cdef uint8_t y
	cdef uint8_t stkp
	cdef uint16_t pc
	cdef uint8_t status
	
	cdef uint8_t _interrupt
	cdef uint8_t _nm_interrupt
	
	cdef Bus bus
	
	cdef uint16_t _fetched
	cdef uint16_t _addr_abs
	cdef uint16_t _addr_rel
	cdef uint8_t _opcode
	cdef uint8_t _cycles
	cdef object lookup
	cdef object instruction

	cdef uint8_t C
	cdef uint8_t Z
	cdef uint8_t I
	cdef uint8_t D
	cdef uint8_t B
	cdef uint8_t U
	cdef uint8_t V
	cdef uint8_t N

	cdef void fetch(self)
	cdef void set_nmi(self, uint8_t v)
	
	cdef void write(self, uint16_t addr, uint8_t data)
	cdef uint8_t read(self, uint16_t addr)
	cdef void set_flag(self, uint16_t f, uint16_t v)
	cpdef void clock(self)

	# Adressing modes
	cdef uint8_t IMM(self)
	cdef uint8_t ZP0(self)
	cdef uint8_t ZPX(self)
	cdef uint8_t ZPY(self)
	cdef uint8_t REL(self)
	cdef uint8_t ABS(self)
	cdef uint8_t ABX(self)
	cdef uint8_t ABY(self)
	cdef uint8_t IND(self)
	cdef uint8_t IZX(self)
	cdef uint8_t IZY(self)

	# Opcodes
	cdef uint8_t ADC(self)
	cdef uint8_t AND(self)
	cdef uint8_t ASL(self)
	cdef uint8_t BCC(self)
	cdef uint8_t BCS(self)
	cdef uint8_t BEQ(self)
	cdef uint8_t BIT(self)
	cdef uint8_t BMI(self)
	cdef uint8_t BNE(self)
	cdef uint8_t BPL(self)
	cdef uint8_t BRK(self)
	cdef uint8_t BVC(self)
	cdef uint8_t BVS(self)
	cdef uint8_t CLC(self)
	cdef uint8_t CLD(self)
	cdef uint8_t CLI(self)
	cdef uint8_t CLV(self)
	cdef uint8_t CMP(self)
	cdef uint8_t CPX(self)
	cdef uint8_t CPY(self)
	cdef uint8_t DEC(self)
	cdef uint8_t DEX(self)
	cdef uint8_t DEY(self)
	cdef uint8_t EOR(self)
	cdef uint8_t INC(self)
	cdef uint8_t INX(self)
	cdef uint8_t INY(self)
	cdef uint8_t JMP(self)
	cdef uint8_t JSR(self)
	cdef uint8_t LDA(self)
	cdef uint8_t LDX(self)
	cdef uint8_t LDY(self)
	cdef uint8_t LSR(self)
	cdef uint8_t NOP(self)
	cdef uint8_t ORA(self)
	cdef uint8_t PHA(self)
	cdef uint8_t PHP(self)
	cdef uint8_t PLA(self)
	cdef uint8_t PLP(self)
	cdef uint8_t ROL(self)
	cdef uint8_t ROR(self)
	cdef uint8_t RTI(self)
	cdef uint8_t RTS(self)
	cdef uint8_t SBC(self)
	cdef uint8_t SEC(self)
	cdef uint8_t SED(self)
	cdef uint8_t SEI(self)
	cdef uint8_t STA(self)
	cdef uint8_t STX(self)
	cdef uint8_t STY(self)
	cdef uint8_t STZ(self)
	cdef uint8_t TAX(self)
	cdef uint8_t TAY(self)
	cdef uint8_t TSX(self)
	cdef uint8_t TXA(self)
	cdef uint8_t TXS(self)
	cdef uint8_t TYA(self)
	cdef uint8_t XXX(self)