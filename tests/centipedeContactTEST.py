'''
	centipedeContactTEST
	by Nick Quinnell
	last updated 2013-06-19
	nicq@umich.edu
	
	test harness for centipedeContact module

'''


from centipedeContact import *
from pylab import *
from numpy import *

	
'''
	need to put in code to plot second half of gait
	i.e. phi = [0.5, 1]
	
	also need to incorporate the code into the rest of the centipede code
	create/update a progress message for the contact gait
	then grep/tee/cut the output and put in a .csv file for easy
	plotting and analysis
'''	

def printGait(roll, yaw, gaitState, roll2, yaw2, gaitState2, phi):

		
	figure(1)
	xlabel('Yaw')
	ylabel('Roll')
	
	plotSecondHalf = True

	#first half of gait
	#plot 'front'
	endFrontIdx = gaitState.index('stance') - 1
	rollFront = roll[0:endFrontIdx]
	yawFront = yaw[0:endFrontIdx]
	plot(yawFront, rollFront,'bo-')
	hold(True)
	
	#plot 'stance'
	endStanceIdx = gaitState.index('recovery') - 1
	rollStance = roll[endFrontIdx+1:endStanceIdx]
	yawStance = yaw[endFrontIdx+1:endStanceIdx]
	plot(yawStance, rollStance, 'go-')
	
	#plot 'recovery'
	rollRecov = roll[endStanceIdx+1:]
	yawRecov = yaw[endStanceIdx+1:]
	plot(yawRecov, rollRecov, 'ro-')
	
	
	if not plotSecondHalf:
		grid(True)
		title('Yaw v Roll for contact gait (side 1)\ncontact = cos(yaw) > 0.8')
		xlim(-1.1, 1.1)
		ylim(-1.1, 1.1)
		show()
	
	if plotSecondHalf:
		#second half of gait
		#figure(2)
		#xlabel('Yaw')
		#ylabel('Roll')
			
		#get second half of gaitState list
		endFrontIdx = gaitState2.index('stance') - 1
		rollFront = roll2[0:endFrontIdx]
		yawFront = yaw2[0:endFrontIdx]
		plot(-yawFront, -rollFront, 'bo-')
		hold(True)
	
		#plot 'stance'
		endStanceIdx = gaitState2.index('recovery') - 1
		rollStance = roll2[endFrontIdx+1:endStanceIdx]
		yawStance = yaw2[endFrontIdx+1:endStanceIdx]
		plot(-yawStance, -rollStance, 'go-')
	
		#plot 'recovery'
		rollRecov = roll2[endStanceIdx+1:-1]
		yawRecov = yaw2[endStanceIdx+1:-1]
		plot(-yawRecov, -rollRecov, 'ro-')
	
		grid(True)
		title('Yaw v Roll for contact gait (both sides)\nside 1: contact = cos(yaw) > 0.8\nside 2: contact = cos(yaw) > 0.7')
		xlim(-1.1, 1.1)
		ylim(-1.1, 1.1)
		show()
	

	

#set up parameters for the gait
initParams = gaitParams()
initParams.rollThresh = -0.99
initParams.yawThresh = -0.9
initParams.maxRoll = 1
initParams.rollAmp = 1
initParams.yawAmp = 1
initParams.stanceVel = 10
initParams.endRecovWindow = 0.2

myGait = contactGait(initParams)
#print('myGait.gaitState = ' + myGait.gaitState + '\n\n')

#set up phi
numVals = 10
phiVals = linspace(0, 0.9, numVals)

#first half of gait
rollVals = zeros(numVals/2, float)
yawVals = zeros(numVals/2, float)
gaitStates = list()

#second half of gait
rollVals2 = zeros(numVals/2, float)
yawVals2 = zeros(numVals/2, float)
gaitStates2 = list()

textDebug = False

i=0
for phi in phiVals:
	myGait.manageGait(phi)
	
	if i < numVals/2:
		rollVals[i] = myGait.roll
		yawVals[i] = myGait.yaw
		gaitStates.append(myGait.gaitState)
	else:
		rollVals2[i-numVals/2] = myGait.roll
		yawVals2[i-numVals/2] = myGait.yaw
		gaitStates2.append(myGait.gaitState)
	
	i += 1

	if textDebug:
		print('phi = ' + str(phi))
		print('gaitState = ' + myGait.gaitState)
		print('myGait.roll = ' + str(myGait.roll))
		print('myGait.yaw = ' + str(myGait.yaw))
		print('cos(yaw) = ' + str(cos(myGait.yaw)))
		print('\n')


printGait(rollVals, yawVals, gaitStates, rollVals2, yawVals2, 
		  gaitStates2, phiVals)

	

	
	
	
