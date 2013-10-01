from joy import *
from joy.remote import Sink as RemoteSink
import ckbot.dynamixel as dynamixel
import ckbot.logical as logical
from math import copysign, cos, sin, pi, atan2, sqrt
from time import time as now, sleep
from numpy import matrix 

class lowpass:
    def __init__( self, firstVal=0.0, tau=0.5 ):
        """
        Initialize lowpass
        """
        self.val = firstVal
        self.tau = tau        
        self.t0 = now()

    def update( self, new_val ):
        """
        Update the low pass with a new val        
        Returns new filtered value 
        """
        # Update dt 
        curtime = now()
        dt = curtime - self.t0
        self.t0 = curtime

        # Update values 
        alpha = dt / ( self.tau + dt )
        self.val = alpha*new_val + (1-alpha)*self.val
        return self.val 

class diffDriveApp( JoyApp ):
    """
    Diff drive using 2 dynamixel servos controlled via joystick
    """
    RPM2RADS = 9.5493

    def __init__( self, *args, **kw ):
        """
        Initialize the protocol, and app
        """
        kw.update( robot=dict(count=2) )
        JoyApp.__init__( self, confPath="$/cfg/diffDriveApp.yml", *args, **kw )
        self.r = self.robot.at
        self.stop()
    
        self.drive = 0
        self.turn = 0
        self.k_drive = 1
        self.k_turn = 0.2 
        self.i_turn = 0

        self.LW_sign = 1
        self.RW_sign = -1     
        self.Turret_sign = -1

        # Lowpass filters for the wheel rpms 
        self.l_filter = lowpass( tau=1.0 )
        self.r_filter = lowpass( tau=1.0 )
        
        # Diff drive state 
        self.pose = matrix([0.0, 0.0, 0.0])
        self.state_t0 = now()
        
        self.cmd_l = lowpass( tau=1.0 )
        self.cmd_r = lowpass( tau=1.0 )

        # Turret state_t0
        self.turret_pose = 0.0
        self.turret_t0 = now()
        self.K_turret = -0.5
        self.turret_max = 0.2

        # Waypoints 
        self.waypoints = None
        self.init_pos = False
        self.wp_cnt = 1

        # Light sensors, eventually will use these to update orientation estimates
        self.line_sense = []
        return        
    
    def stop( self ):
        progress( "Stopping all modules" )
        self.r.LW.go_slack()
        self.r.RW.go_slack()
        #self.r.Turret.go_slack()

    def update_control( self, phase ):    

        # I like being able to get voltage ...
        voltage = self.r.LW.get_voltage()
        progress( "Voltage: %5.2f" % voltage )
        if voltage < 13.0:
            # If voltage is low stop the robot
            progress(" LOW VOLTAGE!!!! " )
            self.stop()
            return

        # Update control inputs for wheels
        self.r.LW.set_torque( self.LW_sign * (self.k_drive*self.drive - self.k_turn*self.turn) )
        self.r.RW.set_torque( self.RW_sign * (self.k_drive*self.drive + self.k_turn*self.turn) )

        """
        # If we have yet to set our current position, set it to the location
        # of the first waypoint
        if not self.waypoints:
            return

        if self.wp_cnt+1 > len(self.waypoints):
            progress( "Finished" )
            self.stop()
            return
        
        if not self.init_pos:
            progress( " Initializing position " )
            self.pose[0,:2] = self.waypoints[0]
            self.init_pos = True
            
        cur_pos = self.pose[0,:2] 
        goal_pos = self.waypoints[self.wp_cnt] 
        dir_vec = goal_pos - cur_pos
        ##progress("dir_vec: %s" % repr( dir_vec ))
        theta_des = -atan2( dir_vec[0,0], dir_vec[0,1] )
        progress( "theta_des: %f" % theta_des )

        # Turn towards waypoint 
        theta_cur = self.pose[0,2]        
        error = theta_des - theta_cur  
        ##progress("error: %f" % error)     
        #self.i_turn += 0.001*error
        w = self.k_turn * error + self.i_turn
        if abs(w) > 0.4:
            w = copysign(0.4, w)
        w_l = -w/2.0
        w_r = w/2.0

        # Calculate remaining command authority 
        max_cmd = 0.3
        cmd_remaining = max_cmd - (abs(w_l)+abs(w_r))        
        if cmd_remaining < 0.0:
            cmd_remaining = 0.0

        # Calculate distance from desired location
        dist = sqrt( (goal_pos[0,0] - cur_pos[0,0])**2 + (goal_pos[0,1] - cur_pos[0,1])**2 )    
        progress( "Distance from wp%i: %f" % (self.wp_cnt, dist) )
        
        # stop if we are close to the goal
        if dist < 0.05:
            self.wp_cnt += 1
            return
        
        lcmd = self.cmd_l.update( self.LW_sign * ( w_l + cmd_remaining/2.0 ) )
        rcmd = self.cmd_r.update( self.RW_sign * ( w_r +cmd_remaining/2.0 ) )
        self.r.LW.set_torque( lcmd )
        self.r.RW.set_torque( rcmd )
        

    def update_turret( self, phase ):
        # Update control input for turret based upon estimated orientation
        # Read until we get a non-nonesense value, ... kinda ( filter? )
        while True:
            turret_rpm = self.Turret_sign * self.r.Turret.get_speed()
            if abs(turret_rpm) < 70.0:
                break        
        turret_rads = turret_rpm/self.RPM2RADS                
        ###progress( "turret_rads: %5.2f" % turret_rads )
        # Integrate to get believed turret position
        # Update our dt before integrating 
        curtime = now()
        dt = curtime - self.turret_t0
        self.turret_t0 = curtime    
        self.turret_pose += turret_rads*dt
        ###progress( "turret pose: %5.2f" % self.turret_pose )

        # Convert radian values to degrees 
        phi = self.pose[0,2]
        rad2deg = (180/pi)
        theta_des = phi        
        theta_cur = self.turret_pose
        error = theta_des - theta_cur
        progress( "theta_cur: %5.2f, theta_des: %5.2f" % (theta_cur, theta_des) )
        progress( "Turret Error: %5.2f" % error )                
        cmd = self.K_turret*error
        if abs(cmd) > self.turret_max:
            cmd = copysign( self.turret_max, cmd )
        self.r.Turret.set_torque( cmd )
        """

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

    def onStart( self ):
        """
        """

        control_fcp = FunctionCyclePlan( self, self.update_control, N=10, maxFreq=10, interval=0.1 )
        control_fcp.onStart = curry( progress, "Initializing Control FCP" )
        control_fcp.onStop = curry( progress, "Stopping Control  FCP" )
        self.control_fcp = control_fcp

        pose_fcp = FunctionCyclePlan( self, self.update_pose, N=5, maxFreq=5, interval=0.2 )
        pose_fcp.onStart = curry( progress, "Initializing Estimator FCP" )
        pose_fcp.onStop = curry( progress, "Stopping Estimator FCP" )
        self.pose_fcp = pose_fcp

        """
        turret_fcp = FunctionCyclePlan( self, self.update_turret, N=10, maxFreq=10, interval=0.1 )
        turret_fcp.onStart = curry( progress, "Initializing Turret FCP" )
        turret_fcp.onStop = curry( progress, "Stopping Turret FCP" )
        self.turret_fcp = turret_fcp        
        """

        self.remoteInput = RemoteSink( self, bnd=('',31313) )
        self.remoteInput.start()

        # Remote Sink for misc data, need to talk to Shai about JoyApp one
        self.fieldInput = RemoteSink( self, convert=lambda x: x, allowMisc=10.0 )
        self.fieldInput.start()

        self.timeToGet = self.onceEvery(0.25)

        # For now I'm just going to use keyboard control
        #sf = StickFilter( self )
        #sf.setLowpass( "joy0axis2", 5 )
        #sf.setLowpass( "joy0axis3", 5 )        
        #sf.start()
        #self.sf = sf
        
    def onStop( self ):
        # If FunctionCyclePlans are running don't forget to stop them 
        if self.control_fcp.isRunning():
            self.control_fcp.stop()
        if self.pose_fcp.isRunning():
            self.pose_fcp.stop()
        # Also remember to stop all modules
        self.stop()
    
    def onEvent( self, evt ):    

        # Get data from field
        if self.timeToGet():            
            for msg in self.fieldInput.queueIter():
                msg = msg[1]
                if msg.has_key('w'):
                    self.waypoints = []
                    scale_factor = 57.0/100.0
                    for wp in msg['w']:
                        self.waypoints.append(scale_factor*matrix([-wp[0], wp[1]])/100.0)
                elif msg.has_key('b') and msg.has_key('f'):
                    self.line_sense = []
                    self.line_sense = [msg['b'], msg['f']]
                    
                    
            #progress('Waypoints %s'
            #         % repr(self.waypoints) )

        # Handle keyboard events 
        if evt.type == KEYDOWN:
            if evt.key == K_SPACE:
                # Toggle FunctionCyclePlans 
                if self.control_fcp.isRunning():
                    self.control_fcp.stop()
                else:
                    self.control_fcp.start()            
                if self.pose_fcp.isRunning():
                    self.pose_fcp.stop()
                else:
                    self.pose_fcp.start()
                """
                if self.turret_fcp.isRunning():
                    self.turret_fcp.stop()
                else:
                    self.turret_fcp.start()
                """
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
    
    app = diffDriveApp()
    app.run()
        
        
