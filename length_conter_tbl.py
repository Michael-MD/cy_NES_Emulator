import numpy as np

length_conter_tbl = np.zeros(32, dtype=np.uint8)

length_conter_tbl[0x00] = 10
length_conter_tbl[0x01] = 254
length_conter_tbl[0x02] = 20
length_conter_tbl[0x03] = 2
length_conter_tbl[0x04] = 40
length_conter_tbl[0x05] = 4
length_conter_tbl[0x06] = 80
length_conter_tbl[0x07] = 6
length_conter_tbl[0x08] = 160
length_conter_tbl[0x09] = 8
length_conter_tbl[0x0A] = 60
length_conter_tbl[0x0B] = 10
length_conter_tbl[0x0C] = 14
length_conter_tbl[0x0D] = 12
length_conter_tbl[0x0E] = 26
length_conter_tbl[0x0F] = 14
length_conter_tbl[0x10] = 12
length_conter_tbl[0x11] = 16
length_conter_tbl[0x12] = 24
length_conter_tbl[0x13] = 18
length_conter_tbl[0x14] = 48
length_conter_tbl[0x15] = 20
length_conter_tbl[0x16] = 96
length_conter_tbl[0x17] = 22
length_conter_tbl[0x18] = 192
length_conter_tbl[0x19] = 24
length_conter_tbl[0x1A] = 72
length_conter_tbl[0x1B] = 26
length_conter_tbl[0x1C] = 16
length_conter_tbl[0x1D] = 28
length_conter_tbl[0x1E] = 32
length_conter_tbl[0x1F] = 30