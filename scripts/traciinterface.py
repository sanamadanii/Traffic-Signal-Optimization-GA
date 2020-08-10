
#====================================Import Modules====================================
from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import subprocess
import random
import copy
import numpy as np

try:
    sys.path.append(os.path.join(os.path.dirname(
        __file__), '..', '..', '..', '..', "tools"))  # tutorial in tests
    sys.path.append(os.path
.join(os.environ.get("SUMO_HOME", os.path.join(
        os.path.dirname(__file__), "..", "..", "..")), "tools"))  # tutorial in docs
    from sumolib import checkBinary
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of \
        your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

import traci
from junction import Junction
from device import Device
from phaseConfig import setJunctionPhase

#===============================Generate Route File====================================

#put code to generate route file here, or better make it in some other python file, import and then run here!

#====================================Make Junctions====================================
junction_U = Junction(_id = 'U',
	dev_a_dets = ['0', '1', '2', '3', '4', '5'], 
	dev_b_dets = ['6', '7', '8', '9', '10', '11'], 
	dev_c_dets = ['54', '55', '56', '57', '58', '59'], 
	dev_d_dets = ['60', '61', '62', '63', '64', '65'],
	phaseMap = {1:1, 2:2, 3:4, 4:3})

junction_L = Junction(_id = 'L', 
	dev_a_dets = ['18', '19', '20', '21', '22', '23'], 
	dev_b_dets = ['12', '13', '14', '15', '16', '17'], 
	dev_c_dets = ['66', '67', '68', '69', '70', '71'], 
	dev_d_dets = ['24', '25', '26', '27', '28', '29'],
	phaseMap = {1:1, 2:2, 3:4, 4:3})

junction_R = Junction(_id = 'R', 
	dev_a_dets = ['30', '31', '32', '33', '34', '35'], 
	dev_b_dets = ['48', '49', '50', '51', '52', '53'], 
	dev_c_dets = ['36', '37', '38', '39', '40', '41'], 
	dev_d_dets = ['42', '43', '44', '45', '46', '47'],
	phaseMap = {1:3, 2:1, 3:2, 4:4})

#set neighbours
junction_U.neighbours = [{'junction': junction_L,'connection': ('d', 'b'), 'data':0}, {'junction': junction_R, 'connection': ('c', 'b'), 'data':0}]
junction_L.neighbours = [{'junction': junction_R,'connection': ('c', 'a'), 'data':0}, {'junction': junction_U, 'connection': ('b', 'd'), 'data':0}]
junction_R.neighbours = [{'junction': junction_L,'connection': ('a', 'c'), 'data':0}, {'junction': junction_U, 'connection': ('b', 'c'), 'data':0}]
#========================================run()=========================================



def run(i):

    print()
    print("Iteration : ",i)
    print()
    output.write("\n")
    output.write("Iteration:"),output.write(str(i))
    output.write("\n")
    output.write("--------------------------------")
    output.write("\n")
    
    endSimTime=16
    global steps
    steps=0
    step=0
    global allWaitingTime

    allWaitingTime = []

    global allTravelTime
    allTravelTime = []

    global allarrived
    allarrived =[]

    global alldeparted
    
    global fitnessInd
    
    global phaseInd
     

    global AllFit
    
    global currentvechiles
    global AllPhase

    global BestPopInd
    global BestPopFit
    global fuelConsumption
    fuelConsumption=[]
    
    initial=[[50, 23, 59],[23, 50, 45],[52, 7, 49],[22, 59, 38],[25, 7, 44],[32, 56, 59],[15, 46, 9],[10, 8, 44],[6, 57, 27],[40, 10, 33],              [45, 44, 14],[59, 19, 56],[22, 12, 28],[8, 23, 26],[45, 16, 39],[56, 48, 57],[46, 51, 47],[17, 54, 51],[54, 30, 11],[36, 25,                30],[24, 41, 56],[47, 48, 39],[60, 28, 39],[52, 44, 18],[50, 58, 55],[8, 21, 9],[50, 24, 47],[32, 30, 19],[29, 38, 23],[13,                44, 45]]
    global x
    x=0
    
    global updated
    if i == 0 :
        
        while step < endSimTime:
            print()
            print("---------------------------------------------------------")
            print()
            print("Steps : " , step)
            #traci.simulationStep()
            GAphase = initial[x]
            x+=1
            phaseInd.append(GAphase)
            #print(" PHASEInd : ",phaseInd)
            temp1 = []
            temp2 = []
            phase = int( (GAphase[0]+GAphase[1]+GAphase[2])/3)
            runDeviceDetect(phase)
            
            """
		    gets data from devices for
		    junctions for "time" number of simulation steps
	    """
            edgeIDs = traci.edge.getIDList()
            for j in edgeIDs:
                temp1.append(traci.edge.getTraveltime(j))
                temp2.append(traci.edge.getWaitingTime(j))

            if sum(alldeparted)==0:
            	allWaitingTime.append(sum(temp1))
            else:
                allWaitingTime.append(sum(temp1)/sum(alldeparted))
            allTravelTime.append(sum(temp2))         
          
            Cr = phase * (7/21)
            if sum(allarrived) == 0:
                fitness= (sum(temp2)+sum(temp1)+ (sum(alldeparted)-sum(allarrived))*traci.simulation.getTime() )/ 1 + Cr
            else:
                fitness= (sum(temp2)+sum(temp1)+ (sum(alldeparted)-sum(allarrived))*traci.simulation.getTime() )/ (sum(allarrived))**2 + Cr
            	
            fitnessInd.append(fitness)
            AllFit.append(fitnessInd)
            AllPhase.append(phaseInd)
            
            #print(len(alldeparted))
            currentvechiles=traci.vehicle.getIDList()
            for i in range(len(currentvechiles)):
                Fue=traci.vehicle.getFuelConsumption(str(currentvechiles[i]))
                #print("FUE",Fue)
                fuelConsumption.append(Fue)
            

            print("Phase Average: ", phase,"   ","Fitness:",fitness)
            print("Arrived Cars : ", sum(allarrived))
            print("Departed Cars : ", sum(alldeparted))
            print("Current Simulation time : ", traci.simulation.getTime())
            print("Avg Waiting time: ",sum(allWaitingTime)/len(allWaitingTime))
            print("Avg Travel time: ",sum(allTravelTime)/len(allTravelTime))
            print("Avg Fuel Consumption: ",sum(fuelConsumption)/len(fuelConsumption))
            
            output.write("Step:"),output.write(str(step))
            output.write ("\n")
            output.write("Phases:"), output.write(str(GAphase))
            output.write ("\n")
            output.write ("Phase Average: ")  ,output.write(str(phase))
            output.write ("\n")
            output.write("Fitness:"), output.write(str(fitness))
            output.write ("\n")
            output.write("Arrived Cars:"),output.write(str(sum(allarrived)))
            output.write ("\n")
            output.write("Departed Cars:"),output.write(str(sum(alldeparted)))
            output.write ("\n")
            output.write ("Avg Waiting time: ")   ,output.write(str(sum(allWaitingTime)/len(allWaitingTime)))
            output.write ("\n")
            output.write ("Avg travel time: ")   ,output.write(str(sum(allTravelTime)/len(allTravelTime)))
            output.write ("\n")
            output.write("Avg Fuel Consumption:"),output.write (str(sum(fuelConsumption)/len(fuelConsumption)))
            output.write("\n")
            output.write("-------------------------------------------------------------------------")
            output.write("\n")
            
            
           
              
            
        
            
            
            
            step+=1
            useAlgoAndSetPhase()
            """	
	    use an algorithm to set the phase for the junctions
	    """
            prepareJunctionVectArrs()
            '''
	    prepare the vehicleVectarr for junctions
	    '''

            setJunctionPhasesInSUMO()
            '''
	    set the junction's phases in the SUMO simulator
	    '''
        Bstfit=min(fitnessInd)
        idx=fitnessInd.index(Bstfit)
        BstInd=phaseInd[idx]
        BestPopFit.append(Bstfit)
        BestPopInd.append(BstInd)
        
        
        
         
        
       
   

    else:
        
        step = 0
        allWaitingTime=[]
        allTravelTime=[]
        allarrived = []
        alldeparted=[]
        fuelConsumption=[]
        steps=0
        
        
        
        ga=GA(AllPhase[-1],AllFit[-1],pcross=0.6,pmuta=0.05)
        updated = ga[0]
        BestPopInd.append(ga[1])
        BestPopFit.append(ga[2])
        
        AllPhase.append(updated)
        

        h=0

        phaseInd=[]
        fitnessInd=[]
        
          
        while step < len(updated):
            # traci.simulationStep()
            temp1 = []
            temp2 = []
              
            Ind = updated[h] 
            
            h+=1
            phase = int( (Ind[0]+Ind[1]+Ind[2])/3 )
            runDeviceDetect(phase)
            """
            gets data from devices for
            junctions for "time" number of simulation steps

            """
                
            edgeIDs = traci.edge.getIDList()

            for j in edgeIDs:
                temp1.append(traci.edge.getTraveltime(j))
                temp2.append(traci.edge.getWaitingTime(j))
                              
          
            if (sum(alldeparted)==0):
                allWaitingTime.append(sum(temp1))
            else:
                allWaitingTime.append(sum(temp1)/sum(alldeparted))

                
            allTravelTime.append(sum(temp2)) 
            
            
            Cr = phase * (7/21)
            if sum(allarrived) == 0:
                fitness= (sum(temp2)+sum(temp1)+ (sum(alldeparted)-sum(allarrived))*traci.simulation.getTime() )/ 1 + Cr
            else:
                fitness= (sum(temp2)+sum(temp1)+ (sum(alldeparted)-sum(allarrived))*traci.simulation.getTime() )/ (sum(allarrived))**2 + Cr
              
            fitnessInd.append(fitness)
            fuelConumption=[]
            currentvechiles=traci.vehicle.getIDList()
            for i in range(len(currentvechiles)):
                Fue=traci.vehicle.getFuelConsumption(str(currentvechiles[i]))
                #print("FUE",Fue)
                fuelConsumption.append(Fue)
            
                
            print()
            print("---------------------------------------------------------")
            print()
            print("Steps: ",step)
            print("Phase Average: ", phase,"   ","Fitness:",fitness)
            print("Arrived Cars : ", sum(allarrived))
            print("Departed Cars: ", sum(alldeparted))
            print("Current Simulation time : ", traci.simulation.getTime())
            print("Waiting time: ",sum(allWaitingTime)/len(allWaitingTime))
            print("Travel time: ",sum(allTravelTime)/len(allTravelTime))
            print("Fuel Consumption: ",sum(fuelConsumption)/len(fuelConsumption))
            
            
            output.write("Step:"),output.write(str(step))
            output.write ("\n")
            output.write ("Phase Average: ")  ,output.write(str(Ind))
            output.write ("\n")
            output.write ("Phase Average: ")  ,output.write(str(phase))
            output.write ("\n")
            output.write("Fitness:"), output.write(str(fitness))
            output.write ("\n")
            output.write("Arrived Cars:"),output.write(str(sum(allarrived)))
            output.write ("\n")
            output.write("Departed Cars:"),output.write(str(sum(alldeparted)))
            output.write ("\n")
            output.write ("Avg Waiting time: ")   ,output.write(str(sum(allWaitingTime)/len(allWaitingTime)))
            output.write ("\n")
            output.write ("Avg travel time: ")   ,output.write(str(sum(allTravelTime)/len(allTravelTime)))
            output.write ("\n")
            output.write("Avg Fuel Consumption:"),output.write (str(sum(fuelConsumption)/len(fuelConsumption)))
            output.write("\n")
            output.write("-------------------------------------------------------------------------")
            output.write("\n")
            
            
 
            
              
            useAlgoAndSetPhase() 
            """
            use an algorithm to set the phase for the junctions
            """
            prepareJunctionVectArrs()
            '''
            prepare the vehicleVectarr for junctions
            '''

            setJunctionPhasesInSUMO()
            '''
            set the junction's phases in the SUMO simulator
            '''
            step+=1
            
            
            
    AllFit.append(fitnessInd)         
    return BestPopFit,BestPopInd
    #output.close()
    
    
       
        
       
      
    





#==========================Supplimentary functions for run()===========================
def setJunctionPhasesInSUMO():
	setJunctionPhase(junction_U, setAllRed = False)
	setJunctionPhase(junction_L, setAllRed = False)
	setJunctionPhase(junction_R, setAllRed = False)

	return

def useAlgoAndSetPhase():
	
	junction_U.update()
	junction_L.update()
	junction_R.update()
	
	return

def runDeviceDetect(time):
	
	global steps

	for _ in range(time):
		
		junction_U.checkDevices()
		junction_L.checkDevices()
		junction_R.checkDevices()
		allarrived.append(traci.simulation.getArrivedNumber())
		alldeparted.append(traci.simulation.getDepartedNumber())

		traci.simulationStep()
		steps += 1
	
	return 

def prepareJunctionVectArrs():
	
	junction_U.prepareVehVectarr()
	junction_L.prepareVehVectarr()
	junction_R.prepareVehVectarr()
	
	return

def individual(indSiz):
    ind=[]
    for i in range (indSiz):
        ind.append(random.randint(5,60))
    return ind
    


def Elitism(pop,popFit):
    elit=[] 
    elit_fit=[]
    
    bestFit=min(popFit)
    idx=popFit.index(bestFit)
    elit.append(pop[idx])
    elit_fit.append(bestFit)
    pop.remove(pop[idx])  
    popFit.remove(popFit[idx])
    
    bestFit2=min(popFit)
    idx=popFit.index(bestFit2)
    elit.append(pop[idx])
    elit_fit.append(bestFit2)
    pop.remove(pop[idx])  
    popFit.remove(popFit[idx])

    return elit,pop,popFit,elit_fit


def Fit_Calculations(popFit):  
    sum=0.0
    for i in range(len(popFit)):
        sum+=popFit[i]
    fit = []
    for i in range(len(popFit)):
        fit.append(popFit[i]/sum)
    return fit

def Calc_Commulative_fit(fit):
    CommulativeFit = []
    temp=0.0
    for i in range(len(fit)):
        CommulativeFit.append(fit[i]+temp)
        temp= CommulativeFit[i]
    return CommulativeFit


def Roullette_Selection(CommulativeFit,pop):
    selected_ind=[]
    for i in range(np.shape(CommulativeFit)[0]):
        selection_Prob=np.random.rand()
        selected=pop[0] 
        for j in range(np.shape(CommulativeFit)[0]):
            if CommulativeFit[j] < selection_Prob :
                continue
            if CommulativeFit[j] >= selection_Prob :
                selected=pop[j]
                selected_ind.append(selected)
                break
    return selected_ind


def crossOver(popSiz,indSiz, selected_ind,pCross = 0.6):
    newpop =[] 
    while len(newpop) != popSiz-2:
        for x in range (0,np.shape(selected_ind)[0],2):
            cutPoint=np.round(np.random.rand()*(indSiz-1))
            p1=selected_ind[x]
            p2=selected_ind[x+1]
            offspring1 =[]
            offspring2= []
            crossProb=np.random.rand()
            if crossProb > pCross :
                newpop.append(p1)
                newpop.append(p2)
            else: 
                for i in range (0,int(cutPoint),1):
                    offspring1.append(p1[i])
                for i in range ((int(cutPoint)),len(p2),1):
                    offspring1.append(p2[i])
                for i in range (0,int(cutPoint),1):
                    offspring2.append(p2[i])
                for i in range ((int(cutPoint)),len(p1),1):                         
                    offspring2.append(p1[i])
                newpop.append(offspring1)
                newpop.append(offspring2)
    return newpop


def mutation(pop,indSiz,elit,pmuta=0.05):
    fine_pop = []
    mut=np.random.randint(5,10)
    for i in range(np.shape(pop)[0]):
        p=pop[i]
        for x in range (0,indSiz,1):
            mut_prob=np.random.rand()
            if  mut_prob < pmuta :
                p[x] = abs(p[x]-mut)
            else:
                continue

        fine_pop.append(p)

    fine_pop.append(elit[0])
    fine_pop.append(elit[1])
    

    return fine_pop


def GA(pop,popFit,pcross=0.6,pmuta=0.05):
    popSiz=len(pop)
    e=Elitism(pop,popFit)
    fit=Fit_Calculations(e[2])
    CommulativeFit=Calc_Commulative_fit(fit)
    selected_ind=Roullette_Selection(CommulativeFit,e[1])
    newpop=crossOver(popSiz,3,selected_ind,0.6)
    pop_final=mutation(newpop,3,e[0],0.05)
    fittest=min(e[3]) 
    idx=e[3].index(fittest)
    best_individual=e[0][idx]


    return pop_final,best_individual,fittest
   
#===============================Start SUMO and call run()==============================
def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options


if __name__ == "__main__":
    options = get_options()
   

   

    global alldeparted
    alldeparted =[] 

    global fitnessInd
    fitnessInd=[]
    global phaseInd
    phaseInd=[]  

    global AllFit
    AllFit=[]

    global AllPhase
    AllPhase=[]
    global updated

    global BestPopInd
    global BestPopFit
    BestPopInd=[]
    BestPopFit=[]

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # first, generate the route file for this simulation
    #generate_routefile()

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    for i in range (10):

        traci.start([sumoBinary, "-c", "../city.sumocfg",
                             "--tripinfo-output", "../tripinfo.xml"]) 
        output = open ("output.txt", "a")  
        Result=run(i)
        print()
        print("------------------------------------------------------------")
        print("End of iteration : ",i)
        print("Best Phases and its fitness for the entire iteration : ")
        print(Result[1][-1],",",Result[0][-1])
        print("------------------------------------------------------------")
        output.write("End of iteration: "), output.write(str(i))
        output.write("\n")
        output.write("Best phases:"), output.write(str(Result[1][-1]))
	output.write("\n")
        output.write("Best fitness:"), output.write(str(Result[0][-1]))
        output.write("\n")
        
       
    
    traci.close()   

