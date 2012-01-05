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
    RPM2RADS = 9.5493

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

        # State estimation for when we are trying to find a line
        self.pose = matrix([0.0, 0.0, 0.0])
        self.state_t0 = now()

        # Waypoints 
        self.waypoints = []
        
        # Line sensor data
        self.line_sense = [0,0]

        # Control constants
        self.follow_line_p = 0.1

        # run
        self.Run = False
        
    def stop( self ):
        progress( "Stopping all modules" )
        self.r.LW.go_slack()
        self.r.RW.go_slack()
        #self.r.Turret.go_slack()

    def update_pose( self , phase ):
        """
        Estimate pose of the diff drive
        """
        # Get rpm of each wheel
        # Read until we get a non-nonesense value, ... kinda ( filter? )
        while True:
            l_rpm = self.LW_sign*self.r.LW.get_speed()
            if abs(l_rpm) < 70.0:
                break
        while True:
            r_rpm = self.RW_sign*self.r.RW.get_speed()
            if abs(r_rpm) < 70.0:
                break   

        l_rads = l_rpm/self.RPM2RADS
        r_rads = r_rpm/self.RPM2RADS
        ###progress( "l_rads: %5.2f, r_rads: %5.2f" % (l_rads, r_rads) )
        
        # Convert wheel rpm values into velocity and angular rate
        ### NOTE: I need to get all the conversions correct here
        wheel_radius = 0.062
        base_width = 0.165
        v = ((r_rads + l_rads)*wheel_radius)/2.0
        w = ((r_rads - l_rads)*wheel_radius)/base_width
        qdot = matrix([v, w])
        ###progress( "v: %5.2f, w: %5.2f" % (v, w) )

        # Estimate pose by integrating x_velocity, y_velocity and angular rate 
        phi = self.pose[0,2]
        J = matrix( [[ -sin( phi ), 0],
                     [ cos( phi ), 0],
                     [ 0           , 1]] )
        pdot = J*qdot.transpose()
        # Update our dt before integrating 
        curtime = now()
        dt = curtime - self.state_t0
        self.state_t0 = curtime    
        # Integrate 
        self.pose = self.pose + pdot.transpose()*dt
        progress( "Current pose: %s" % repr(self.pose) )
        return

    def parseField( self ):
        for msg in self.fieldInput.queueIter():
            msg = msg[1]
            print msg
            if msg.has_key('w'):
                self.waypoints = []
                scale_factor = 100.0/100.0
                for wp in msg['w']:
                    self.waypoints.append(scale_factor*matrix([-wp[0], wp[1]])/100.0)
            if msg.has_key('b'):
                self.line_sense[0] = msg['b']
            if msg.has_key('f'):
                self.line_sense[1] = msg['f']

    def calcOrientation( self ):
        """
        Calculate current orientation based upon the line that we think we are on 
        using atan2 
        """
        p0 = self.waypoints[0]
        p1 = self.waypoints[1]
        a = p0[0] - p0[0]
        b = p1[1] - p1[1]
        theta = atan2(b, a)
        return theta         

    def followLine( self ):
        """
        A line following behavior.
        Does proportional control on the difference between the front and back sensors
        """
        self.following = True
        # Calculate current orientation
        theta = self.calcOrientation()
        self.pose[0,2] = theta
        theta = (180/pi) * theta

        sense_diff = self.line_sense[1] - self.line_sense[0]
        progress( "sense_diff: %f" % sense_diff )
        # Scale to 1.0 instead of 255 
        w_cmd = self.follow_line_p * (sense_diff/255.0) 
        fforward = 0.12
        w_cmd = w_cmd + copysign( fforward, w_cmd )
        progress( "w_cmd: %f" % w_cmd )
        l_cmd = -self.LW_sign * w_cmd/2.0
        r_cmd =  self.RW_sign * w_cmd/2.0

        self.r.LW.set_torque( l_cmd )
        self.r.RW.set_torque( r_cmd )

        # Don't forget to orient the laser
        self.r.Turret.set_pos( theta * 100.0 ) 


    def findLine( self ):
        """
        Do a spiral and to try and navigate the robot to the line
        """
        self.updatePose()

        if self.following:
            self.following = False
            self.v = 0.0
            if randint(0,1):
                self.w = 0.1
            else:
                self.w = -0.1
        self.v += 0.01
        l_cmd = -self.w + self.v
        r_cmd =  self.w + self.v
        self.r.LW.set_torque( l_cmd )
        self.r.RW.set_torque( r_cmd )        
        # Don't forget to update the turret
        self.r.Turret.set_pos( (18000/pi)*self.pose[0,2] )
        return

    def updateControl( self ):
        """
        Make control decisions        
        """
        # I like being able to get voltage ...
        voltage = self.r.LW.get_voltage()
        progress( "Voltage: %5.2f" % voltage )
        if voltage < 13.0:
            # If voltage is low stop the robot
            progress(" LOW VOLTAGE!!!! " )
            self.stop()
            return

        # Robot behaviors
        if self.line_sense[0] == 0 or self.line_sense[1] == 0:
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
        self.timeToGetData = self.onceEvery(0.05)
        
        # Timer event for control
        self.timeToControl = self.onceEvery(0.1)

    def onStop( self ):
        """
        Stop the robot
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
                if self.Run == True:
                    self.Run = False
                elif self.Run == False:
                    self.Run = True

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
        
    
        
        
