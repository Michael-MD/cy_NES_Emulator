from setuptools import setup
from Cython.Build import cythonize
import numpy

setup(
	name='NES Emulator',
	package_dir={'cy_NES_Emulator': ''},
	include_dirs=[numpy.get_include()],
    ext_modules=cythonize([
    	'CPU6502.pyx', 
    	'Bus.pyx',
    	'PPU.pyx',
    	'Cartridge.pyx',
    	# 'Screen.pyx',
    	'NES.pyx',
    	'Mappers/Mapper.pyx',
    	'Mappers/Mapper000.pyx',
    	# 'EmulationDebugger.pyx',
    	# 'available_colours.pyx',
    ])
)
