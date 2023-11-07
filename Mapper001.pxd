cdef extern from "Mapper001.hpp":
    cdef cppclass Mapper001:
        unsigned char prog_banks
        unsigned char char_banks
        Mapper001(unsigned char prog_banks, unsigned char char_banks) except +

        unsigned char ppu_map_read(unsigned short addr, unsigned int *mapped_addr)
        unsigned char ppu_map_write(unsigned short addr, unsigned int *mapped_addr)

        unsigned char cpu_map_read(unsigned short addr, unsigned int *mapped_addr, unsigned char *data)
        unsigned char cpu_map_write(unsigned short addr, unsigned int *mapped_addr, unsigned char data)
