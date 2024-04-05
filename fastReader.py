# https://matplotlib.org/stable/gallery/animation/strip_chart.html
#
#
#

from matplotlib.figure import Figure
import numpy as np
from matplotlib.lines import Line2D
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib as mpl
import matplotlib.colors as col

import serial
import serial.tools.list_ports as sertools

# pip install wxpython
# pip install pyside6 # (alternative if wxpython fails to build)

devices = sertools.comports()
device_reciever = 1
device_sender = 0
serialStream = None

receiveBuffer: str = ""

MAX_Y =  50000 # 4000

SERVO_MIN = 500
SERVO_MAX = 1833


class Scope:
	def __init__(self, ax, scopeChannels = list(range(16)), maxt=2, dt=0.02, maxv=MAX_Y):
		self.ax = ax
		self.scopeChannels = scopeChannels
		self.dt = dt
		self.maxt = maxt
		self.maxv = maxv

		self.tdata = [0.0]
		self.yData = [[0] for x in range(0, len(scopeChannels))]

		self.lines: list[Line2D] = []
		for l in range(0, len(scopeChannels)):
			self.lines.append(Line2D(self.tdata, self.yData[l], color=col.hsv_to_rgb((1/len(scopeChannels)*l, 1, 1)), label="D"+str(scopeChannels[l])))
			self.ax.add_line(self.lines[l])

		self.ax.set_ylim(-.1, self.maxv)
		self.ax.set_xlim(0, self.maxt)
		self.ax.legend(handles=self.lines)


	def update(self, y):
		if y is None:
			return []

		lastt = self.tdata[-1]
		if lastt >= self.tdata[0] + self.maxt:  # reset the arrays
			self.tdata = [self.tdata[-1]]

			for l in range(0, len(self.scopeChannels)):
				self.yData[l] = [self.yData[l][-1]]

			self.ax.set_xlim(self.tdata[0], self.tdata[0] + self.maxt)
			self.ax.figure.canvas.draw()

		# This slightly more complex calculation avoids floating-point issues
		# from just repeatedly adding `self.dt` to the previous value.
		t = self.tdata[0] + len(self.tdata) * self.dt

		self.tdata.append(t)

		for l in range(0, len(self.scopeChannels)):
			assert isinstance(self.lines[l], Line2D)

			self.yData[l].append(y[self.scopeChannels[l]])
			self.lines[l].set_data(self.tdata, self.yData[l])

		return self.lines


def emitter(p=0.1):
	"""Return an array from the serial console"""
	global serialStream
	global receiveBuffer

	assert serialStream is not None

	if serialStream.in_waiting > 0:
		receiveBuffer += serialStream.read_all().decode('ASCII') # type: ignore

		splitted = receiveBuffer.split('\n')
		receiveBuffer = splitted[len(splitted) - 1]

		if(len(splitted) > 1):
			for l in splitted[ : -1]:
				stripped = l.strip()
				str_begin = stripped.find('[') + 1
				str_end = stripped.find(']') - 1

				if str_begin < 0 or str_end < 0 or str_begin >= str_end:
					print("[Serial] " + l)
					continue
				else:
					data = [float(i) for i in stripped[str_begin : str_end].split(',')]
					#print(data)
					yield data
					return

	yield None


def main():
	global serialStream

	print("Starting Serial Plotter")
	fig, ax = plt.subplots()
	#scope = Scope(ax, [1, 5, 9, 13])
	scope = Scope(ax)

	# Start Serial
	print("Connecting to " + devices[device_reciever].name)
	with serial.Serial(devices[device_reciever].name, 115200, timeout=5) as ser:
		serialStream = ser

		# pass a generator in "emitter" to produce data for the update func
		ani = animation.FuncAnimation(fig, scope.update, emitter, interval=50, blit=False, save_count=100)

		plt.show()
	
	serialStream = None


# --- Main ---
if __name__ == "__main__":
	#mpl.use('wxAgg')
	#mpl.use('QtAgg')
	main()
