cdef extern from "Mapper000.hpp":
    cdef cppclass Mapper000:
        unsigned char prog_banks
        unsigned char char_banks
        Mapper000(unsigned char prog_banks, unsigned char char_banks) except +

        unsigned char ppu_map_read(unsigned short addr, unsigned short *mapped_addr)
        unsigned char ppu_map_write(unsigned short addr, unsigned short *mapped_addr)

        unsigned char cpu_map_read(unsigned short addr, unsigned short *mapped_addr)
        unsigned char cpu_map_write(unsigned short addr, unsigned short *mapped_addr, unsigned char data)