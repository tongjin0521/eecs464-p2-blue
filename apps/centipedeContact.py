'''
	centipedeContact.py
	by Nick Quinnell
	last updated 2013-06-11
	nicq@umich.edu
	
	Implementation of Prof Revzen's hexapod ground-sensing gait
	Each half-cycle of the gait consists of three sections:
		Front - begin roll/yaw movement end w/ ground contact or roll limit
		Stance - hold roll constant, yaw until no contact or yaw limit
		Recovery - graceful return to roll = 0

	Phi is an input to many functions.  Phi has a range of [0, 1] and proceeds
	monotonically from 0 to 1 then resets to 0.  It corresponds to the range of 
	0 to 2*pi radians, so trig functions that take phi as an argument must have
	phi multiplied by 2*pi to produce valid results -- need to alter the input 
	when phi is in the range [0.5, 1] to transform roll/yaw calculations
	
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
		
		#assert here should prolly do abs() to avoid confusion & update do()'s
		assert self.params.rollThresh < self.params.rollAmp, "rollAmp < rollThresh"
		assert self.params.yawThresh < self.params.yawAmp, "yawAmp < yawThresh"
		assert self.params.rollAmp > 0, "rollAmp <= 0"
		assert self.params.yawAmp > 0, "yawAmp <= 0"
		
	#do stuff based on gaitState & contact
	#should only call this function unless debugging
	def manageGait(self, phi):
		#first find status of foot contact
		self.inContact = self.contact()
		
		#take appropriate action based on phi/contact (simple)
		#calculate roll/yaw, then determine state when next called
		if self.gaitState == 'front':
			self.doFront(phi)
			if self.exitFront():	
			#if 'front' ends, record phi0 & jump to stance for calc of roll/yaw
				self.phi0 = phi		
				self.gaitState = 'stance'
		elif self.gaitState == 'stance':
			self.doStance(phi)			
			if self.exitStance():
			#if 'stance' ends, record phi1, y1, r1 
				self.phi1 = phi		
				self.y1 = self.yaw	
				self.r1 = self.roll	
				self.gaitState = 'recovery'
				assert self.y1 <= 0, "y1 > 0, ended 'stance' too early"
		elif self.gaitState == 'recovery':
			self.doRecovery(phi)
			if self.exitRecovery(phi):
			#if 'recovery' ends, we're back at stance for the other side
				self.roll = 0
				self.gaitState = 'front'
		else:
			#should never reach this code, must always be in
			#front or stance or recovery
			print("ERROR: invalid gaitState")
			print("\tgaitState: '"  + "'")			

	#calculate roll/yaw when in 'front' state
	def doFront(self, phi):
		self.yaw = cos(phi*2*pi) * self.params.yawAmp 
		self.roll = -sin(phi*2*pi) * self.params.rollAmp
		
		if(0.5 < phi <= 1):	#transform when in second half of gait
			self.yaw = -self.yaw
			self.roll = -self.roll

	#function to handle the exit conditions of the 'front' gaitState
	#must be called after doFront to use updated roll value
	def exitFront(self):
		if (self.roll <= self.params.rollThresh) or self.inContact:
			return True
		return False

	#calculate roll/yaw when in 'stance' state
	#error: if there is contact @ phi=0, doStance() will not change
	#	yaw.  Need to change the sgn(sin(phi0)) to something else - it is
	#	zero when phi0=0
	def doStance(self, phi):
		self.yaw = (self.params.yawAmp*cos(2*pi*self.phi0) - 
				self.params.stanceVel*(phi - self.phi0)*sgn(sin(self.phi0)))
		self.roll = -sin(2*pi*self.phi0)*self.params.rollAmp
		
		if(0.5 < phi <= 1): #transform when in second half of gait
			self.yaw = -self.yaw
			self.roll = -self.roll

	#function to handle the exit conditions of the 'stance' gaitState
	def exitStance(self):
		if ((self.yaw < 0 and not self.inContact) or 
			self.yaw <= self.params.yawThresh):
			return True
		return False
			
	#calculate roll/yaw when in 'recovery' state
	#not sure this works the way it's supposed to judging from graphs
	def doRecovery(self, phi):
		#unlike phi, psi is a real angle that doesn't require *2*pi
		psi = (pi/2)*(2*pi*phi - 2*pi*self.phi1)/(pi - 2*pi*self.phi1)
		self.yaw = self.y1 - ((self.params.yawAmp + self.y1) * sin(psi))
		self.roll = self.r1 * cos(psi)

	#function to handle the exit conditions of the 'recovery' gaitState
	def exitRecovery(self, phi):
		if phi == 0.5 or phi == 0:	#consider adding windows around 0 & 0.5 for robustness
			return True
		return False

	#determine contact state
	#for some reason, this doesn't work right now
	def contact(self):
		if(cos(self.yaw) > 0.8):
			return True
		return False
		
