#!/usr/bin/env python
from joy import *
from joy.remote import Sink as RemoteSink
import ckbot.dynamixel as dynamixel
import ckbot.logical as logical
from math import copysign, cos, sin, pi, atan2, sqrt

class armJoy( JoyApp ):
    """
    Initialize the protocol, and app
    """
    def __init__( self, *args, **kw ):
        """
        Initaliaze the app
        """
        p = dynamixel.Protocol()
        kw.update( robot=dict(protocol=p, count=4) )
        JoyApp.__init__( self, confPath="$/cfg/armJoyApp.yml", *args, **kw )        
        self.r = self.robot.at
        self.stop()
        # Modes are either 'arm' or 'torso'
        self.mode = 'arm'

        # Do something where we get the names of each servo and create
        # A dict of initiale positions
        self.zeros = {}
        # How do I do this in a nice clean way?
        for m in self.r:
            self.zeros[m] = getattr(self.r, m).get_pos()

    def set_speed_limit( self, lim ):
        """
        Set speed limit for all servos as a safety measure
        """
        for m in self.r:
            getattr(self.r, m).set_speed( lim )

    def stop( self ):
        """
        This stops all modules
        """
        for m in self.r:
            getattr(self.r, m).go_slack()

    def onStart( self ):
        """
        Initialize joystick control, voltage checking ...
        """
        remote_filt = lambda dic: dic if dic['type_code'] in set([KEYUP, KEYDOWN, JOYAXISMOTION, JOYBUTTONDOWN, JOYBUTTONUP]) else None
        self.remoteInput = RemoteSink( self, convert=remote_filt, bnd=('',31313) )
        self.remoteInput.start()

        self.checkVoltage = self.onceEvery(1.5)
        #self.showInputs = self.onceEvery(0.2)

        # For now I'm just going to use keyboard control
        sf = StickFilter( self )
        sf.setLowpass( "joy0axis0", 5 )
        sf.setLowpass( "joy0axis1", 5 )                
        sf.setLowpass( "joy0axis2", 5 )
        sf.setLowpass( "joy0axis3", 5 )        
        sf.start()    
        self.sf = sf

    def onStop( self ):
        # Stop all modules 
        self.stop()

    def onEvent( self, evt ):    
        # Check the voltage 
        if self.checkVoltage():            
            # I like being able to get voltage ...
            voltage = self.r.elbow.get_voltage()
            progress( "Voltage: %5.2f" % voltage )
            if voltage < 13.0:
                # If voltage is low stop the robot
                progress(" LOW VOLTAGE!!!! " )
                self.stop()
                return
        # Show our input commands
        """
        if self.showInputs():            
            if self.mode == 'arm':
                progress( " elbow: %.3f, shoulder %.3f, pivot %.3f, torso %.3f " % (self.elbow, self.shoulder, self.pivot, self.torso) )
            elif self.mode == 'torso':
                progress( " fbar_l %.3f, fbar_r %.3f, torso %.3f " % (self.fbar_l, self.fbar_r, self.torso ) )
        """

        # Handle joystick motion
        if evt.type == JOYAXISMOTION:
            self.sf.push( evt )
            if self.mode == 'arm':
                self.elbow = self.zeros['elbow'] + self.sf.getValue("joy0axis0")
                self.shoulder = self.zeros['shoulder'] + self.sf.getValue("joy0axis1")
                self.pivot = self.zeros['pivot'] + self.sf.getValue("joy0axis2")
                self.torso = self.zeros['torso'] + self.sf.getValue("joy0axis3" )
                progress( " elbow: %.3f, shoulder %.3f, pivot %.3f, torso %.3f " % (self.elbow, self.shoulder, self.pivot, self.torso) )
            elif self.mode == 'torso':
                self.fbar_l = self.zeros['fbar_l'] + self.sf.getValue("joy0axis0")
                self.fbar_r = self.zeros['fbar_r'] + self.sf.getValue("joy0axis1")
                self.torso = self.zeros['torso'] + self.sf.getValue("joy0axis3" )
                progress( " fbar_l %.3f, fbar_r %.3f, torso %.3f " % (self.fbar_l, self.fbar_r, self.torso ) )

        if evt.type == JOYBUTTONDOWN:            
            if evt.button == 1:
                if self.mode == 'arm':
                    self.mode == 'torso'
                    progress( "Switching mode to TORSO" )
                if self.mode == 'torso':
                    self.mode == 'arm'
                    progress( "Switching mode to ARM" )            

        if evt.type == KEYDOWN:
            if evt.key in [K_q, K_ESCAPE]:
                progress( "Quiting armJoy" )
                self.stop()
                                     
        JoyApp.onEvent( self, evt )
        return


if __name__ == "__main__":
    
    app = armJoy()
    try:        
        app.run()
    except KeyboardInterrupt:
        app.stop()
        
        
