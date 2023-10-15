from setuptools import setup
from Cython.Build import cythonize

setup(
	name='NES Emulator',
	package_dir={'cy_NES_Emulator': ''},
    ext_modules=cythonize([
    	'CPU6502.pyx', 
    	# 'Bus.pyx',
    	'PPU.pyx',
    	# 'Cartridge.pyx',
    	# 'Screen.pyx',
    	# 'NES.pyx',
    	# 'EmulationDebugger.pyx',
    	# 'available_colours.pyx',
    ])
)
