'''
	centipedeContactTEST
	by Nick Quinnell
	last updated 2013-06-11
	nicq@umich.edu
	
	test harness for centipedeContact module

'''


from centipedeContact import *
from pylab import *
from numpy import *

	
'''
	put in code to plot different parts of gait in diff colors
	prolly use find() to get the range of indexes for each part
	of the gait, then plot the parts separately on the same graph
'''	

def printGait(roll, yaw, gaitState):
	figure(1)
	xlabel('Yaw')
	ylabel('Roll')

	#plot 'front'
	endFrontIdx = gaitState.index('stance') - 1
	rollFront = roll[0:endFrontIdx]
	yawFront = yaw[0:endFrontIdx]
	plot(yawFront, rollFront, color='blue')
	hold(True)
	
	#plot 'stance'
	endStanceIdx = gaitState.index('recovery') - 1
	rollStance = roll[endFrontIdx+1:endStanceIdx]
	yawStance = yaw[endFrontIdx+1:endStanceIdx]
	plot(yawStance, rollStance, color='green')
	
	#plot 'recovery'
	rollRecov = roll[endStanceIdx+1:]
	yawRecov = yaw[endStanceIdx+1:]
	plot(yawRecov, rollRecov, color='red')
	
	grid(True)
	title('Yaw v Roll for contact gait')
	xlim(-1.1, 1.1)
	ylim(-1.1, 1.1)
	show()

	

#set up parameters for the gait
initParams = gaitParams()
initParams.rollThresh = -0.7
initParams.yawThresh = -0.8
initParams.maxRoll = 1
initParams.rollAmp = 1
initParams.yawAmp = 1
initParams.stanceVel = 10

myGait = contactGait(initParams)
#print('myGait.gaitState = ' + myGait.gaitState + '\n\n')

numVals = 16
phiVals = linspace(0, 0.5, numVals)
rollVals = zeros(numVals, float)
yawVals = zeros(numVals, float)
gaitStates = list()

textDebug = True

i=0
for phi in phiVals:
	myGait.manageGait(phi)

	rollVals[i] = myGait.roll
	yawVals[i] = myGait.yaw
	gaitStates.append(myGait.gaitState)
	i += 1

	if textDebug:
		#should use matplotlib to visualize roll/yaw instead of printing
		print('phi = ' + str(phi))
		print('gaitState = ' + myGait.gaitState)
		print('myGait.roll = ' + str(myGait.roll))
		print('myGait.yaw = ' + str(myGait.yaw))
		print('cos(yaw) = ' + str(cos(myGait.yaw)))
		print('\n')


printGait(rollVals, yawVals, gaitStates)

	

	
	
	
