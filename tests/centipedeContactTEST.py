'''
	centipedeContactTEST
	by Nick Quinnell
	last updated 2013-07-15
	nicq@umich.edu
	
	test harness for centipedeContact module

'''


from centipedeContact import *
from pylab import *
from numpy import *

	
'''
	need to incorporate the code into the rest of the centipede code
	create/update a progress message for the contact gait
	then grep/tee/cut the output and put in a .csv file for easy
	plotting and analysis
'''	

def printGait(roll, yaw, gaitState, phi):

	fig1 = False
	
	if(fig1):
		
		figure(1)
		xlabel('Yaw (deg)')
		ylabel('Roll (deg)')

		#plot 'front'
		endFrontIdx = gaitState.index('stance') - 1
		rollFront = roll[0:endFrontIdx]
		yawFront = yaw[0:endFrontIdx]
		print(str(rollFront))
		plot(yawFront, rollFront,'bo')
		hold(True)
	
		#plot 'stance'
		endStanceIdx = gaitState.index('recovery') - 1
		rollStance = roll[endFrontIdx+1:endStanceIdx]
		yawStance = yaw[endFrontIdx+1:endStanceIdx]
		print(str(rollStance))
		plot(yawStance, rollStance, 'go')
	
		#plot 'recovery'
		rollRecov = roll[endStanceIdx+1:]
		yawRecov = yaw[endStanceIdx+1:]
		print(str(rollRecov))
		plot(yawRecov, rollRecov, 'ro')
	
	
		grid(True)
		title('Yaw v Roll for contact gait')
		#xlim(-1.1, 1.1)
		#ylim(-1.1, 1.1)
	
	figure(2)
	xlabel('Yaw (deg)')
	ylabel('Roll (deg)')
	plot(yaw, roll, 'ko')
	grid(True)
	title('Yaw v Roll for contact gait')
	xlim(-17, 17)
	ylim(-42, 42)
	
	show()
	

	

#set up parameters for the gait
initParams = gaitParams()
initParams.rollThresh = -(33*pi/180)
initParams.yawThresh = -(12*pi/180)
initParams.maxRoll = (40*pi/180)
initParams.rollAmp = (40*pi/180)
initParams.yawAmp = (15*pi/180)
initParams.stanceVel = 3
initParams.endRecovWindow = 0.2

myGait = contactGait(initParams)

#set up phi
numVals = 1024
phiVals = linspace(0, 0.5, numVals)

numVals -= 1
phiVals = phiVals[:-1]

#first half of gait
rollVals = zeros(numVals, float)
yawVals = zeros(numVals, float)
gaitStates = list()

textDebug = False
printGraphs = True

i=0
for phi in phiVals:
	myGait.manageGait(phi)
	
	rollVals[i] = myGait.roll*180/pi
	yawVals[i] = myGait.yaw*180/pi
	gaitStates.append(myGait.gaitState)
	
	if textDebug:
		print('phi = ' + str(phi))
		print('gaitState = ' + myGait.gaitState)
		print('myGait.roll = ' + str(myGait.roll))
		print('myGait.yaw = ' + str(myGait.yaw))
		print('cos(yaw) = ' + str(cos(myGait.yaw)))
		print('\n')
	
	#this is just for graphing purposes
	if(0.5 <= phi < 1):
		rollVals[i] = -rollVals[i]
		yawVals[i] = -yawVals[i]
	
	i += 1

	


if printGraphs:
	printGait(rollVals, yawVals, gaitStates, phiVals)

	

	
	
	
