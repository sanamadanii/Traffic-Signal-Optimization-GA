#===================checking requirements and importing traci module===================
import os, sys
import copy
if 'SUMO_HOME' in os.environ:
	tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
	sys.path.append(tools)
else:   
	sys.exit("please declare environment variable 'SUMO_HOME'")

import traci

#===============================Doing Other Imports====================================
from device import Device
from algorithm import updateJunction

#=================================now defining junction================================
class Junction(object):

	def __init__(self, _id, dev_a_dets, dev_b_dets, dev_c_dets, dev_d_dets, phaseMap):
		
		self._id = _id
		self.dev_a = self.createDevice(dev_a_dets)
		self.dev_b = self.createDevice(dev_b_dets)
		self.dev_c = self.createDevice(dev_c_dets)
		self.dev_d = self.createDevice(dev_d_dets)
		self.vehicleVectarr = [[], [], [], []]
		self.isFirstPhase = True
		self.green = 1
		self.QaNum = 30
		self.QbNum = 30
		self.QcNum = 30
		self.QdNum = 30
		self.Qab = 10
		self.Qac = 10
		self.Qad = 10
		self.Qba = 10
		self.Qbc = 10
		self.Qbd = 10
		self.Qca = 10
		self.Qcb = 10
		self.Qcd = 10
		self.Qda = 10
		self.Qdb = 10
		self.Qdc = 10
		self.visitNum = 0
		self.neighbours = []
		self.phaseMap = phaseMap
	def createDevice(self, dets):
		return Device(dets)

	def checkDevices(self):
		self.vehicleVectarr = [self.dev_a.checkDevice(self.vehicleVectarr[0]), self.dev_b.checkDevice(self.vehicleVectarr[1]), \
		self.dev_c.checkDevice(self.vehicleVectarr[2]), self.dev_d.checkDevice(self.vehicleVectarr[3])]
		return self.vehicleVectarr
	
	def prepareVehVectarr(self):
		vehicleVectarr = self.vehicleVectarr
		vehicleVectarrOri = copy.deepcopy(vehicleVectarr)
		vehicleVectarr[0] = list(set(vehicleVectarr[0]) - set(vehicleVectarrOri[1]) - set(vehicleVectarrOri[2]) - set(vehicleVectarrOri[3]))
		vehicleVectarr[1] = list(set(vehicleVectarr[1]) - set(vehicleVectarrOri[2]) - set(vehicleVectarrOri[3]) - set(vehicleVectarrOri[0]))
		vehicleVectarr[2] = list(set(vehicleVectarr[2]) - set(vehicleVectarrOri[3]) - set(vehicleVectarrOri[0]) - set(vehicleVectarrOri[1]))
		vehicleVectarr[3] = list(set(vehicleVectarr[3]) - set(vehicleVectarrOri[0]) - set(vehicleVectarrOri[1]) - set(vehicleVectarrOri[2]))
		self.vehicleVectarr = vehicleVectarr
		return vehicleVectarr
	
	def updateNeighbourInfo(self):
		i = 0
		for neighbour in self.neighbours:
			(fromNode, toNode) = neighbour['connection']
			toNode = 'Q'+toNode+'Num'
			self.neighbours[i]['data'] = getattr(neighbour['junction'], toNode)
			i += 1
		return
	
	def update(self):
		self.updateNeighbourInfo()
		self =  updateJunction(self)