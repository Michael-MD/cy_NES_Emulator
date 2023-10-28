from setuptools import setup, Extension
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


# from setuptools import setup
# from Cython.Build import cythonize
# import numpy
# from setuptools.extension import Extension
# from Cython.Compiler.Options import get_directive_defaults

# directive_defaults = get_directive_defaults()
# directive_defaults['linetrace'] = True
# directive_defaults['binding'] = True

# # Create Extension objects for each Cython module
# extensions = [
#     Extension(name="CPU6502", sources=["CPU6502.pyx"], define_macros=[('CYTHON_TRACE', '1')]),
#     Extension(name="Bus", sources=["Bus.pyx"], define_macros=[('CYTHON_TRACE', '1')]),
#     Extension(name="PPU", sources=["PPU.pyx"], define_macros=[('CYTHON_TRACE', '1')]),
#     Extension(name="Cartridge", sources=["Cartridge.pyx"], define_macros=[('CYTHON_TRACE', '1')]),
#     Extension(name="NES", sources=["NES.pyx"], define_macros=[('CYTHON_TRACE', '1')]),
#     # Extension(name="Mapper", sources=["Mappers/Mapper.pyx"], define_macros=[('CYTHON_TRACE', '1')]),
#     # Extension(name="Mapper000", sources=["Mappers/Mapper000.pyx"], define_macros=[('CYTHON_TRACE', '1')]),
#     'Mappers/Mapper.pyx',
# 	'Mappers/Mapper000.pyx',
# ]

# setup(
#     name='NES Emulator',
#     package_dir={'cy_NES_Emulator': ''},
#     include_dirs=[numpy.get_include()],
#     ext_modules=cythonize(extensions)
# )
