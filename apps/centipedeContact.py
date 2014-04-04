"""
	centipedeContact.py
	by Nick Quinnell
	nicq@umich.edu
	
	WHAT DOES IT DO?
	This code implements the algorith of Prof Revzen's hexapod ground-sensing 
	gait.  Each half-cycle of the gait consists of three sections:
		Front - begin roll/yaw movement end w/ ground contact or roll limit
		Stance - hold roll constant, yaw until no contact or yaw limit
		Recovery - graceful return to roll = 0

    WHAT ARE THE MAIN CLASSES/FUNCTIONS?
	The algorithm is run from the contactGait class's manageGait() method, 
	none of the other functions should be called from outside the contactGait
	class.  The gaitParams class is used to store the required parameters 
	to specify the gait itself.
	
	HOW IS IT TYPICALLY USED?
	The init method of contactGait needs to be called with an instance of
	the gaitParams class.  The default value of the parameters in this class
	are likely to change frequently and should be double-checked before 
	running the code on a real hexapod.  Generally it will look like this:
	    
	  INSIDE INIT METHOD
	    myParams = gaitParams();
	    myParams.<<member1>> = <<value>>
	    myParams.<<member2>> = <<value>>
	    ...
	    
	    myGait = contactGait(myParams)
	    
	  INSIDE ROBOT RUNNING METHOD
	    myGait.manageGait(phi)
	    roll = degrees(myGait.roll)
	    yaw = degrees(myGait.yaw)
	    
	    #send roll/yaw values to robot segments
	    segment.set_pos(yaw, bend, roll)		
	    
	WHAT WOULD DEVELOPERS TYPICALLY EXTEND?
	I would	think that only the methods that would be overridden are the 
	doFront(), doStance(), and doRecovery() as these are the meat of the 
	algorithm
	
	WHAT OTHER MODULES DOES IT DEPEND ON OR SUPPORT?
	Depends on math and numpy modules for the trig functions
	
	
"""

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

class contactGait:
	#internal states are gaitState and status of ground contact
    gaitState = 'front'
    inContact = False
	
	#additional state for strafing behavior -- need to be careful with this one
    #should stay as an int since we're only changing it by interger values
    strafe = 0	
	
	#outputs are roll (radians) and yaw (radians)
	#bend is stored as part of the gait but is not used here
    roll = 0
    yaw = 0
    bend = 0
	
	#which leg on which to check the contact sensor
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
    def __init__(self, inParams, inLeg):
		#initialize members
		self.gaitState = 'front'
		self.params = inParams
		self.yaw = inParams.yawAmp	#have to init yaw here to satisfy contraint
									#of the gait (can't have yaw = roll = 0)
		self.strafe = 0
		
		#set the leg on which we need to check the contact sensors
		#or the leg on which to check the roll angle 
		#(leg member of the Segment class)
		self.leg = inLeg
									
		#safety checks on gait parameters. Consult the gait spec document if req
		assert abs(self.params.rollThresh) < abs(self.params.rollAmp), \
				"error: rollAmp < rollThresh"
		assert abs(self.params.yawThresh) < abs(self.params.yawAmp), \
				"error: yawAmp < yawThresh"
		assert self.params.rollAmp > 0, "error: rollAmp <= 0"
		assert self.params.yawAmp > 0, "error: yawAmp <= 0"
									
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
        self.inContact = self.__contact(phi)
        
        locPhi = phi
        #set locPhi to be in the range [0, 0.5)
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
			#if ((not self.inContact) and self.__yawLessThanZero(locPhi)):
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
            yawThresh = self.params.yawThresh
            yawAmp = self.params.yawAmp
            vel = self.params.stanceVel
            
			#calculate roll/yaw
            self.__doFront(locPhi)
			
			#update phi0
            self.phi0 = locPhi
			
			#set stanceLimit, since we now have a valid phi0
			#this assumes that we will always go through at least one portion
			#of 'front' before we get to 'stance' -- this may not be the case
			#need to ask Prof Revzen			
            self.stanceLimit = self.phi0 - (yawThresh - yawAmp*cos(2*pi*self.phi0))/(vel)
			
			#reinitialize phi1 as 0.25, w/ corresponding y1/r1 in case we skip
			#the 'stance' portion of the gait
            self.phi1 = 0.25
            self.y1 = 0
            self.r1 = self.params.rollThresh
			
        elif self.gaitState == 'stance':
			#calculate roll/yaw
			self.__doStance(locPhi)	
			
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
			self.__doRecovery(locPhi)
			
        else:
			#should never reach this code, must always be in
			#front or stance or recovery
			print("ERROR: invalid gaitState")
			print("\tgaitState: '"  + "'")
        
        #we used a local phi with a smaller range to simplify the 
        #front/stance/recov functions now we need to account for that before 
        #the caller uses the roll/yaw values
        if(locPhi != phi):
            self.roll = -self.roll
            self.yaw = -self.yaw

        #augment gait for strafing if that is required
        if(self.strafe > 0 or self.strafe < 0):
            #slide to the left/right
            strafeRad = radians(self.strafe)
            self.yaw = 0
            self.roll += strafeRad    #add the strafe offset to roll
        else:
            pass
            #do not override normal gait if strafe == 0
			
        if(self.debug):	
			print('roll (post-calc) = ' + str(self.roll*180/pi))
			print('yaw (post-calc)= ' + str(self.yaw*180/pi))
			print('\n')                
			
		

	#calculate roll/yaw when in 'front' state
    def __doFront(self, phi):        
		self.yaw = cos(phi*2*pi) * self.params.yawAmp 
		self.roll = -sin(phi*2*pi) * self.params.rollAmp

	#calculate roll/yaw when in 'stance' state
	#error: if there is contact @ phi=0, doStance() will not change
	#	yaw.  Need to change the sgn(sin(phi0)) to something else - it is
	#	zero when phi0=0
	#CORRECTION: removed sgn(sin(phi0)) -- not required because of how we
	#handle phi in the range [0.5, 1]
    def __doStance(self, phi):
        yawAmp = self.params.yawAmp
        rollAmp = self.params.rollAmp
        vel = self.params.stanceVel
        self.yaw = (yawAmp*cos(2*pi*self.phi0) - vel*(phi - self.phi0))
        self.roll = -sin(2*pi*self.phi0)*rollAmp
			
	#calculate roll/yaw when in 'recovery' state
	#need to update the gait pdf with changes here, and assumptions about
	#	the sign of yawAmp
    def __doRecovery(self, phi):
		#unlike phi, psi is a real angle that doesn't require *2*pi		
		psi = (pi/2)*(2*pi*phi - 2*pi*self.phi1)/(pi - 2*pi*self.phi1)
		self.yaw = self.y1 - ((self.params.yawAmp + self.y1) * sin(psi))
		self.roll = self.r1 * cos(psi)

	#determine contact state - dummy for now
	#assumes contact at a given roll angle
    def __contact(self, phi):
		#contactAtRoll = 1500.0	# centidegrees
        #if(0.5 < phi <= 1.0):
        #    contactAtRoll = -contactAtRoll		#other side of leg
        
        # fake contact using get_pos()
		#if(abs(abs(self.leg.get_pos())-self.legOffset) >= contactAtRoll):
		#	return True
		#return False
		
		#if we're strafing, we don't want to consider contact
        if(self.strafe != 0):
            return False
		
        contactAtRoll = (20.0)*pi/180.0
		# fake contact using roll
        if(abs(self.roll) >= contactAtRoll):
            return True
        return False
			
	#rearranges the condition (yaw < 0) to be a function of phi
	#ATTN: something is wrong here... needs a little work... check notes/math
    def __yawLessThanZero(self, phi):
		phi_0 = self.phi0
		yaw_amp = self.params.yawAmp
		vel = self.params.stanceVel
		
		phiLimit = (phi_0 + yaw_amp*cos(2*pi*phi_0)/vel)
		print('phiLimit = ' + str(phiLimit))
		
		yawLTzero = phiLimit < phi
		print('current yaw: ' + str(self.yaw))
		print('yaw < 0 = ' + str(yawLTzero))
		
		return yawLTzero
