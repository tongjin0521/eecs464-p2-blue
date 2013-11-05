'''
	centipedeContact.py
	by Nick Quinnell
	last updated 2013-07-15
	nicq@umich.edu
	
	Implementation of Prof Revzen's hexapod ground-sensing gait
	Each half-cycle of the gait consists of three sections:
		Front - begin roll/yaw movement end w/ ground contact or roll limit
		Stance - hold roll constant, yaw until no contact or yaw limit
		Recovery - graceful return to roll = 0

	Phi is an input to many functions in this file.  Phi has a range of [0, 1] 
	and proceeds monotonically from 0 to 1 then resets to 0.  It corresponds to
	the range of 0 to 2*pi radians, so trig functions that take phi as an 
	argument must have phi multiplied by 2*pi to produce valid results -- need 
	to alter the input when phi is in the range [0.5, 1] to transform roll/yaw 
	calculations
	
'''

from math import *
from numpy import *

sgn = lambda x : (x>0) - (x<0)

class gaitParams:
	rollThresh = -(23*pi/180)
	yawThresh = -(12*pi/180)
	maxRoll = (30*pi/180)
	rollAmp = (30*pi/180)
	yawAmp = (13*pi/180)
	stanceVel = 3
	endRecovWindow = 0.2
	

class contactGait:
	#internal states are gaitState and status of ground contact
	gaitState = 'front'
	inContact = False
	
	#outputs are roll and yaw
	roll = 0
	yaw = 0
	
	#which leg to check the contact sensor on
	leg = 1
	
	#parameters for tuning
	params = gaitParams()
	
	#internal variables
	y1 = 0
	r1 = 0
	phi0 = 0
	phi1 = 0.25
	frontLimit = 0
	stanceLimit = 0
	
	#debug
	debug = False
		
	#init takes one argument: an instance of gaitParams	
	def __init__(self, inParams):
		#initialize members
		self.gaitState = 'front'
		self.params = inParams
		self.yaw = inParams.yawAmp	#have to init yaw here to satisfy contraint
									#of the gait (can't have yaw = roll = 0)
		
		#set the leg on which we need to check the contact sensors
		#self.leg = inLeg
									
		#should update do()'s (see below) w/ abs() if needed
		assert abs(self.params.rollThresh) < self.params.rollAmp, "rollAmp < rollThresh"
		assert abs(self.params.yawThresh) < self.params.yawAmp, "yawAmp < yawThresh"
		assert self.params.rollAmp > 0, "rollAmp <= 0"
		assert self.params.yawAmp > 0, "yawAmp <= 0"
									
		#need to set up limits for front/stance/recovery given the parameters
		#this will let the gaitState be purely a function of phi and contact
		
		#set frontLimit
		self.frontLimit = asin(-self.params.rollThresh/self.params.rollAmp)/(2*pi)
		assert self.frontLimit > 0, "invalid rollThresh/rollAmp combination"
		assert self.frontLimit < 0.25, "invalid rollThresh/rollAmp combination"
		#the two 'asserts' above shold be covered by the 4 previous ones
		
		#cannot set stanceLimit until we have determined phi0... or can we?
		
		#initialize phi1 as 0.25, w/ corresponding y1/r1
		self.phi1 = 0.25
		self.y1 = 0
		self.r1 = self.params.rollThresh
		
	#do stuff based on phi & contact
	#this function should be the only one called from outside the class 
	#unless debugging
	def manageGait(self, phi):
		#first find status of foot contact
		self.inContact = self.contact(phi)
		
		locPhi = phi
		if(0.5 < phi <= 1):
			locPhi -= 0.5
			
		if(self.debug):
			print('phi = ' + str(locPhi))
			print('contact = ' + str(self.inContact))
			print('frontLimit = ' + str(self.frontLimit))
			print('stanceLimit = ' + str(self.stanceLimit))
			print('phi0 = ' + str(self.phi0))
			print('phi1 = ' + str(self.phi1))
			print('roll (pre-state) = ' + str(self.roll*180/pi))
			print('yaw (pre-state)= ' + str(self.yaw*180/pi))
		
		
		#determine state from phi & contact
		if 0 <= locPhi < self.frontLimit:
			if self.inContact:
				self.gaitState = 'stance'
			else:
				self.gaitState = 'front'
		elif self.frontLimit <= locPhi < self.stanceLimit:
			#if ((not self.inContact) and self.yawLessThanZero(locPhi)):
			if ((not self.inContact) and (self.yaw < 0.0)):
				self.gaitState = 'recovery'
			else:
				self.gaitState = 'stance'
		else:
			self.gaitState = 'recovery'
			
		if(self.debug):
			print('state = ' + self.gaitState)	
		
		#take appropriate action based on state, calculate roll/yaw
		#may have still have some issues with skipping a phase of the gait
		#i.e. going straight from 'recovery' to 'stance' and skipping a 'front'
		#phase
		if self.gaitState == 'front':
			#calculate roll/yaw
			self.doFront(locPhi)
			
			#update phi0
			self.phi0 = locPhi
			
			#set stanceLimit, since we now have a valid phi0
			#this assumes that we will always go through at least one portion
			#of 'front' before we get to 'stance' -- this may not be the case
			#need to ask Prof Revzen			
			self.stanceLimit = self.phi0 - (self.params.yawThresh - self.params.yawAmp*cos(2*pi*self.phi0))/(self.params.stanceVel)
			
			#reinitialize phi1 as 0.25, w/ corresponding y1/r1 in case we ski
			#the 'stance' portion of the gait
			self.phi1 = 0.25
			self.y1 = 0
			self.r1 = self.params.rollThresh
			
		elif self.gaitState == 'stance':
			#calculate roll/yaw
			self.doStance(locPhi)	
			
			#update local gait variables
			self.phi1 = locPhi
			self.r1 = self.roll
			
			#special update for y1 -> must have y1 <= 0 always?
			if(self.yaw > 0):
				self.y1 = 0
			else:
				self.y1 = self.yaw
			
		elif self.gaitState == 'recovery':
			#calculate roll/yaw
			self.doRecovery(locPhi)
			
		else:
			#should never reach this code, must always be in
			#front or stance or recovery
			print("ERROR: invalid gaitState")
			print("\tgaitState: '"  + "'")		
			
		if(self.debug):	
			print('roll (post-calc) = ' + str(self.roll*180/pi))
			print('yaw (post-calc)= ' + str(self.yaw*180/pi))
			print('\n')
			
		

	#calculate roll/yaw when in 'front' state
	def doFront(self, phi):					
		self.yaw = cos(phi*2*pi) * self.params.yawAmp 
		self.roll = -sin(phi*2*pi) * self.params.rollAmp

	#calculate roll/yaw when in 'stance' state
	#error: if there is contact @ phi=0, doStance() will not change
	#	yaw.  Need to change the sgn(sin(phi0)) to something else - it is
	#	zero when phi0=0
	#CORRECTION: removed sgn(sin(phi0)) -- not required because of how we
	#handle phi in the range [0.5, 1]
	def doStance(self, phi):
		self.yaw = (self.params.yawAmp*cos(2*pi*self.phi0) - 
				self.params.stanceVel*(phi - self.phi0))
		self.roll = -sin(2*pi*self.phi0)*self.params.rollAmp
			
	#calculate roll/yaw when in 'recovery' state
	#need to update the gait pdf with changes here, and assumptions about
	#	the sign of yawAmp
	def doRecovery(self, phi):
		#unlike phi, psi is a real angle that doesn't require *2*pi
		psi = (pi/2)*(2*pi*phi - 2*pi*self.phi1)/(pi - 2*pi*self.phi1)
		self.yaw = self.y1 - ((self.params.yawAmp + self.y1) * sin(psi))
		self.roll = self.r1 * cos(psi)

	#determine contact state
	def contact(self, phi):
		if(cos(self.yaw) > 0.99):
			return True
		return False
			
	#rearranges the condition (yaw < 0) to be a function of phi
	#ATTN: something is wrong here... needs a little work... check notes/math
	def yawLessThanZero(self, phi):
		phi_0 = self.phi0
		yaw_amp = self.params.yawAmp
		vel = self.params.stanceVel
		
		phiLimit = (phi_0 + yaw_amp*cos(2*pi*phi_0)/vel)
		print('phiLimit = ' + str(phiLimit))
		
		temp = phiLimit < phi
		print('current yaw: ' + str(self.yaw))
		print('yaw < 0 = ' + str(temp))
		
		return temp
