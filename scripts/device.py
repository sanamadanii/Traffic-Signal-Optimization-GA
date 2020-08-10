#===================checking requirements and importing traci module===================
import os, sys
if 'SUMO_HOME' in os.environ:
	tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
	sys.path.append(tools)
else:   
	sys.exit("please declare environment variable 'SUMO_HOME'")

import traci

#==================================now defining device=================================
class Device(object):

	def __init__(self, dets):

		self.dets = dets
	def checkDevice(self, vehicleVect):
		for det in self.dets:
			if traci.inductionloop.getLastStepVehicleNumber(det) > 0:
				for val in traci.inductionloop.getLastStepVehicleIDs(det):
					if val not in vehicleVect:
						vehicleVect.append(val)
		return vehicleVect
