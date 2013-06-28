'''
	centipedeContact.py
	by Nick Quinnell
	last updated 2013-06-19
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
	rollThresh = 0
	yawThresh = 0
	maxRoll = 0
	rollAmp = 0
	yawAmp = 0
	stanceVel = 1
	endRecovWindow = 0.01
	

class contactGait:
	#internal states are gaitState and status of ground contact
	gaitState = 'front'
	inContact = False
	
	#outputs are roll and yaw
	roll = 0
	yaw = 0
	
	#parameters for tuning
	params = gaitParams()
	
	#internal variables
	y1 = 0
	r1 = 0
	phi0 = 0
	phi1 = 0
		
	#init takes one argument: an instance of gaitParams	
	def __init__(self, inParams):
		#initialize members
		self.gaitState = 'front'
		self.params = inParams
		self.yaw = inParams.yawAmp	#have to init yaw here to satisfy contraint
									#of the gait (can't have yaw = roll = 0)
		
		#should update do()'s w/ abs() if needed
		assert abs(self.params.rollThresh) < self.params.rollAmp, "rollAmp < rollThresh"
		assert abs(self.params.yawThresh) < self.params.yawAmp, "yawAmp < yawThresh"
		assert self.params.rollAmp > 0, "rollAmp <= 0"
		assert self.params.yawAmp > 0, "yawAmp <= 0"
		
	#do stuff based on gaitState & contact
	#should only call this function unless debugging
	def manageGait(self, phi):
		#first find status of foot contact
		self.inContact = self.contact(phi)
		
		#take appropriate action based on phi/contact
		#calculate roll/yaw, then determine state when next called
		if self.gaitState == 'front':
			self.doFront(phi)
			if self.exitFront():	
			#if 'front' ends, record phi0
				if(0.5 < phi < 1):
					self.phi0 = phi - 0.5
				else:
					self.phi0 = phi
				self.gaitState = 'stance'
		elif self.gaitState == 'stance':
			self.doStance(phi)			
			if self.exitStance():
			#if 'stance' ends, record phi1, y1, r1 
				if(0.5 < phi < 1):
					self.phi1 = phi - 0.5
				else:
					self.phi1 = phi 
				self.y1 = self.yaw	
				self.r1 = self.roll	
				self.gaitState = 'recovery'
				assert self.y1 <= 0, "y1 > 0, ended 'stance' too early"
		elif self.gaitState == 'recovery':
			self.doRecovery(phi)
			if self.exitRecovery(phi):
			#if 'recovery' ends, we're back at front for the other side
				self.roll = 0
				self.yaw = self.params.yawAmp
				self.gaitState = 'front'
		else:
			#should never reach this code, must always be in
			#front or stance or recovery
			print("ERROR: invalid gaitState")
			print("\tgaitState: '"  + "'")			

	#calculate roll/yaw when in 'front' state
	def doFront(self, phi):
		locPhi = phi
		#transform for second half of gait
		if(0.5 <= phi < 1):
			locPhi = phi - 0.5
			
		self.yaw = cos(locPhi*2*pi) * self.params.yawAmp 
		self.roll = -sin(locPhi*2*pi) * self.params.rollAmp

	#function to handle the exit conditions of the 'front' gaitState
	def exitFront(self):
		if (self.roll <= self.params.rollThresh) or self.inContact:
			return True
		return False

	#calculate roll/yaw when in 'stance' state
	#error: if there is contact @ phi=0, doStance() will not change
	#	yaw.  Need to change the sgn(sin(phi0)) to something else - it is
	#	zero when phi0=0
	def doStance(self, phi):
		locPhi = phi
		locPhi0 = self.phi0
		#transform when in second half of gait
		if(0.5 <= phi < 1):
			locPhi = phi - 0.5
		
		self.yaw = (self.params.yawAmp*cos(2*pi*locPhi0) - 
				self.params.stanceVel*(locPhi - locPhi0)*sgn(sin(2*pi*locPhi0)))
		self.roll = -sin(2*pi*locPhi0)*self.params.rollAmp

	#function to handle the exit conditions of the 'stance' gaitState
	def exitStance(self):
		if ((self.yaw < 0 and not self.inContact) or 
						self.yaw <= self.params.yawThresh):
			return True
		return False
			
	#calculate roll/yaw when in 'recovery' state
	#need to update the gait pdf with changes here, and assumptions about
	#	the sign of yawAmp
	def doRecovery(self, phi):
		#transform when in second half of gait
		locPhi = phi
		locPhi1 = self.phi1
		if(0.5 < phi <= 1):
			locPhi = phi - 0.5
			
		#unlike phi, psi is a real angle that doesn't require *2*pi
		psi = (pi/2)*(2*pi*locPhi - 2*pi*locPhi1)/(pi - 2*pi*locPhi1)
		self.yaw = self.y1 - ((self.params.yawAmp + self.y1) * sin(psi))
		self.roll = self.r1 * cos(psi)

	#function to handle the exit conditions of the 'recovery' gaitState
	def exitRecovery(self, phi):
		window = self.params.endRecovWindow
		if (0.5 < phi < 0.5+window) or (phi == 1) or (0 <= phi < window):	
			return True
		return False

	#determine contact state
	def contact(self, phi):
		if(cos(self.yaw) > 0.8):
			return True
		return False
		
