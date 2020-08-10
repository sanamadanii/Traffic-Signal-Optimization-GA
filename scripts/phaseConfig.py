import os, sys
import copy
if 'SUMO_HOME' in os.environ:
	tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
	sys.path.append(tools)
else:   
	sys.exit("please declare environment variable 'SUMO_HOME'")

import traci

def setJunctionPhase(junction, setAllRed):
	if setAllRed == True:
		traci.trafficlights.setRedYellowGreenState(junction._id, "rrrrrrrrrrrrrrrrrrrrrrrrrrrr")
	elif junction.phaseMap[junction.green] == 1:
		traci.trafficlights.setRedYellowGreenState(junction._id, "rrrrrrrrrrrrrrrrrrrrrggggggg")
	elif junction.phaseMap[junction.green] == 2:
		traci.trafficlights.setRedYellowGreenState(junction._id, "gggggggrrrrrrrrrrrrrrrrrrrrr")
	elif junction.phaseMap[junction.green] == 3:
		traci.trafficlights.setRedYellowGreenState(junction._id, "rrrrrrrrrrrrrrgggggggrrrrrrr")
	elif junction.phaseMap[junction.green] == 4:
		traci.trafficlights.setRedYellowGreenState(junction._id, "rrrrrrrgggggggrrrrrrrrrrrrrr")
	else:
		traci.trafficlights.setRedYellowGreenState(junction._id, "rrrrrrrrrrrrrrrrrrrrrrrrrrrr")
	return
