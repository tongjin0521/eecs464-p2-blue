#!/usr/bin/env python
from joy import *
from joy.remote import Sink as RemoteSink
import ckbot.dynamixel as dynamixel
import ckbot.logical as logical
from math import copysign, cos, sin, pi, atan2, sqrt
from time import time as now, sleep
from numpy import matrix 

class dfDrApp( JoyApp ):
    """
    An app for a diff drive robot using a subsumption like architecture to 
    determine its behavior such that it navigates through a set of waypoints
    using line sensor feedback
    """
    def __init__( self, *args, **kw ):
        """
        Initialize the app
        """
        p = dynamixel.Protocol()
        kw.update( robot=dict(protocol=p, count=2) )
        JoyApp.__init__( self, confPath="$/cfg/diffDriveApp.yml", *args, **kw )
        self.r = self.robot.at
        self.stop()

        self.LW_sign = 1
        self.RW_sign = -1     
        self.Turret_sign = -1

        # Waypoints 
        self.waypoints = waypoints
        
        # Line sensor data
        self.line_sense = []

        # Control constants
        self.follow_line_p = 0.2

        # run
        self.run = False
        
    def stop( self ):
        progress( "Stopping all modules" )
        self.r.LW.go_slack()
        self.r.RW.go_slack()
        #self.r.Turret.go_slack()

    def parseField( self ):
        for msg in self.fieldInput.queueIter():
            msg = msg[1]
            if msg.has_key('w'):
                self.waypoints = []
                scale_factor = 57.0/100.0
                for wp in msg['w']:
                    self.waypoints.append(scale_factor*matrix([-wp[0], wp[1]])/100.0)
            elif msg.has_key('b'):
                self.line_sense[0] = msg['b']
            elif msg.has_key('f'):
                self.line_sense[1] = msg['f']

    def followLine( self ):
        """
        A line following behavior.
        Does proportional control on the difference between the front and back sensors
        """
        sense_diff = self.line_sense[0] - self.line_sense[1]
        # Scale to 1.0 instead of 255 
        w_cmd = self.follow_line_p * (sense_diff/255.0) 
        l_cmd = -self.LW_sign * w_cmd/2.0
        r_cmd =  self.RW_sign * w_cmd/2.0

        self.r.LW.set_torque( lcmd )
        self.r.RW.set_torque( rcmd )

    def findLine( self ):
        """
        Try to navigate the robot to the line
        """
        pass

    def updateControl( self ):
        """
        Make control decisions        
        """
        if self.line_sense[0] == 0 or self.line_sense[1]:
            self.findLine()
        else:
            self.followLine()

    def onStart( self ):
        """
        """
        # RemoteSink for user input
        self.remoteInput = RemoteSink( self, bnd=('',31313) )
        self.remoteInput.start()

        # RemoteSink for data from the field 
        self.fieldInput = RemoteSink( self, convert=lambda x: x, allowMisc=10.0 )
        self.fieldInput.start()

        # Timer event for gathering new input data
        self.timeToGetData = self.onceEvery(0.1)
        
        # Timer event for control
        self.timeToControl = self.onceEvery(0.1)

    def onStop( self ):
        """
        """
        self.stop()

    def onEvent( self, evt ):
        """
        Do things based upon events 
        """
        if self.timeToGetData():
            self.parseField()

        if self.run:
            if self.timeToControl():
                self.updateControl()

                # Handle keyboard events 
        if evt.type == KEYDOWN:
            if evt.key == K_SPACE:
                # Toggle run state 
                if self.run = True:
                    self.run = False
                elif self.run = False:
                    self.run = True

            elif evt.key in [K_ESCAPE, K_q]:
                self.stop()
            elif evt.key == K_w:
                self.drive += 0.1
                progress( "drive %5.2f, turn %5.2f" % (self.drive, self.turn) )
            elif evt.key == K_s:
                self.drive -= 0.1
                progress( "drive %5.2f, turn %5.2f" % (self.drive, self.turn) )
            elif evt.key == K_a:
                self.turn += 0.1
                progress( "drive %5.2f, turn %5.2f" % (self.drive, self.turn) )
            elif evt.key == K_d:
                self.turn -= 0.1 
                progress( "drive %5.2f, turn %5.2f" % (self.drive, self.turn) )            
        JoyApp.onEvent( self, evt )
        return                 

if __name__ == "__main__":
    # Run app
    app = dfDrApp()
    app.run()
        
    
        
        
