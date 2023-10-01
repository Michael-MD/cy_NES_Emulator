import numpy as np

class Screen(object):
	"""
	Keeps track of pixels on screen.
	"""
	def __init__(self):
		self.display = np.zeros([240, 256, 3])

	def __call__(self, scan_line, cycle, c):
		if cycle > 255 or scan_line > 239:	# If beyond visible region
			return
		self.display[scan_line, cycle] = c