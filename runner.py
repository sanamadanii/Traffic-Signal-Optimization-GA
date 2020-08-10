import os
import sys
import optparse, argparse
import subprocess
import random
import pdb
import xml.etree.ElementTree as ET
import numpy as np


if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare the environment variable 'SUMO_HOME'")

from sumolib import checkBinary
import traci
PORT = 8873

def run(max_step=-1):
    
    """execute the TraCI control loop"""
    traci.init(PORT)
    step = 0
    # we start with phase 2 where EW has green
    #traci.trafficlights.setPhase("0", 0)
    #pdb.set_trace()
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        step += 1
        if step > max_step and max_step > 0:
            break
    traci.close()
    sys.stdout.flush()

def run_sumo( summaryFile='summary.xml'):
    travel_times = []
    nsims = 1

    for j in xrange(nsims):

        sumoBinary = checkBinary('sumo')

        # this is the normal way of using traci. sumo is started as a
        # subprocess and then the python script connects and runs
        sumoProcess = subprocess.Popen([sumoBinary, "-c", "city.sumocfg", #"--additional-files", "city.add.xml", 
                                        "--tripinfo-output", "tripinfo.xml", #"--duration-log.statistics", "true", 
                                        "--summary", summaryFile, "--remote-port", str(PORT)], stdout=sys.stdout, stderr=sys.stderr)
        run()
        sumoProcess.wait()
        
        tree = ET.parse(summaryFile)
        root = tree.getroot()
        totalTravel = 0.0
        prev_num_cars_done = 0.0
        for child in root:
            totalTravel += float(child.attrib['meanTravelTime']) * (float(child.attrib['ended']) - prev_num_cars_done)
            prev_num_cars_done = float(child.attrib['ended'])

        travel_times.append(totalTravel / prev_num_cars_done)
        
        TLIds = traci.trafficlights.getIDList()
        for i in TLIds:
            print(traci.trafficlights.setPhaseDuration(i,5))

    print "Mean travel time:", (sum(travel_times) / len(travel_times))
    print("sana")
    return (sum(travel_times) / len(travel_times))
def main():
    run_sumo()
    print("sana")

    #sumoCmd = [sumoBinary, "-c", sumoConfig, "--start"]
    #traci.start(sumoCmd)
    #traci.init(PORT)
    
    #pdb.set_trace()
    #run_sumo()
    #traci.close()
    #sys.stdout.flush()
if __name__ == '__main__':
    main()
