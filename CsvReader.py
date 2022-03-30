import sys
import matplotlib.pyplot as plt
class CsvFileReader:
	def __init__(self):
		self.__vel_filter = []
		self.__vel_origin = []
	def CsvFileReader(self, dir):
		self.__vel_filter.clear()
		self.__vel_origin.clear()
		xaxis = []
		f = open(dir, "r")
		line = f.readline()
		line = f.readline()
		i = 0
		while line:
			linedata = line.split(",")
			if line == "\n":
				break
			self.__vel_origin.append(float(linedata[12]))
			self.__vel_filter.append(float(linedata[11]))
			xaxis.append(i)
			line = f.readline()
			i += 1
		f.close()
		# plt.plot(xaxis, self.__vel_origin, "b-")
		# plt.plot(xaxis, self.__vel_filter, "g-")
		# plt.show()
	
	def GetVel_Origin(self):
		return self.__vel_origin
	
	def GetVel_Filter(self):
		return self.__vel_filter
	