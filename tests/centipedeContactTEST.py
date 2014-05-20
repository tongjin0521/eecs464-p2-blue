'''
	centipedeContactTEST
	by Nick Quinnell	
	nicq@umich.edu
	
	test harness for centipedeContact module

'''


from centipedeContact import *
from pylab import *
from numpy import *

def printGait(roll, yaw, gaitState, phi):

	#config variable to show the individual stages of the gait as diff colors
	fig1 = True
	
	if(fig1):
		
		figure(1)
		xlabel('Yaw (deg)')
		ylabel('Roll (deg)')

		#plot 'front'
		endFrontIdx = gaitState.index('stance') - 1
		rollFront = roll[0:endFrontIdx]
		yawFront = yaw[0:endFrontIdx]		
		plot(yawFront, rollFront,'bo')
		hold(True)
	
		#plot 'stance'
		endStanceIdx = gaitState.index('recovery') - 1
		rollStance = roll[endFrontIdx+1:endStanceIdx]
		yawStance = yaw[endFrontIdx+1:endStanceIdx]
		plot(yawStance, rollStance, 'go')
	
		#plot 'recovery'
		rollRecov = roll[endStanceIdx+1:]
		yawRecov = yaw[endStanceIdx+1:]
		plot(yawRecov, rollRecov, 'ro')

		grid(True)
		title('Yaw v Roll for contact gait')


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

myGait = contactGait(initParams, 1)

#set up phi
numVals = 32
phiVals = linspace(0, 1, numVals)

#cut out the last value
numVals -= 1
phiVals = phiVals[:-1]

#Set up placeholders for gait outputs
rollVals = zeros(numVals, float)
yawVals = zeros(numVals, float)
gaitStates = list()

#config variables for this test
textDebug = False
printGraphs = True

i=0
for phi in phiVals:
	myGait.manageGait(phi)
	
	rollVals[i] = degrees(myGait.roll)
	yawVals[i] = degrees(myGait.yaw)
	gaitStates.append(myGait.gaitState)
	
	if textDebug:
		print('phi = ' + str(phi))
		print('gaitState = ' + myGait.gaitState)
		print('myGait.roll = ' + str(myGait.roll))
		print('myGait.yaw = ' + str(myGait.yaw))
		print('cos(yaw) = ' + str(cos(myGait.yaw)))
		print('\n')
	
	i += 1

if printGraphs:
	printGait(rollVals, yawVals, gaitStates, phiVals)

	

	
	
	
