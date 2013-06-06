'''
	centipedeContact.py
	by Nick Quinnell
	last updated 2013-06-05
	nicq@umich.edu
	
	Implementation of Prof Revzen's hexapod ground-sensing gait
	Each half-cycle of the gait consists of three sections:
		Front - begin roll/yaw movement end w/ ground contact or roll limit
		Stance - hold roll constant, yaw until no contact or yaw limit
		Recovery - graceful return to roll = 0

	
'''

import math
import numpy

class gaitParams:
	rollThresh = 0
	yawThresh = 0
	maxRoll = 0
	rollAmp = 0
	yawAmp = 0
	

class contactGait:
	#internal states are gaitState and status of ground contact
	gaitState = None
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
		
	#do stuff based on gaitState & contact
	#should only call this function unless debugging
	def manageGait(phi):
		#first find status of foot contact
		inContact = contact()
		
		#take appropriate action based on phi/contact
		if gaitState == 'front':
				#consider making funcs 'frontEnds' 'stanceEnds' 'recoveryEnds'
				#to have easier to read code
			if inContact or (self.roll >= self.params.rollThresh):	
			#if 'front' ends, record phi0 & jump to stance for calc of roll/yaw
				self.phi0 = phi		
				doStance(phi)
			else:	#normal 'front' calculation for roll/yaw
				doFront(phi)
		elif gaitState == 'stance':
			if (self.yaw < 0 and not inContact) or ():	
			#if 'stance' ends, record phi1, y1, r1 
			#and jump to 'recovery' for calc of roll/yaw
				doStance(phi)
			else: 	
				doStance()
			pass
		elif gaitState == 'recovery':
			#do 'recovery' calculations
			doRecovery()
			pass
		else:
			#should never reach this code, must always be in
			#front or stance or recovery
			print("ERROR: invalid gaitState/contact combo")
			print("\tgaitState: '" + str(gaitState) + "'")
			print("\tinContact: '" + str(inContact) + "'")

	#calculate roll/yaw when in 'front' state TODO
	def doFront():
		pass

	#calculate roll/yaw when in 'stance' state TODO
	def doStance(phi, contact):
		pass

	#calculate roll/yaw when in 'recovery' state TODO
	def doRecovery(phi, contact):
		pass

	#determine contact state
	def contact():
		return False
		
