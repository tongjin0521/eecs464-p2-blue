from joy import *
import ckbot.dynamixel as dynamixel
import ckbot.logical as logical
from math import copysign, cos, sin, pi
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
    def __init__( self, *args, **kw ):
        """
        Initialize the protocol, and app
        """
        p = dynamixel.Protocol()
        kw.update( robot=dict(protocol=p, count=3) )
        JoyApp.__init__( self, confPath="$/cfg/diffDriveApp.yml", *args, **kw )
        self.r = self.robot.at
    
        self.drive = 0
        self.turn = 0
        self.k_drive = 1
        self.k_turn = 0.5 

        self.LW_sign = 1
        self.RW_sign = -1 
        
        # Lowpass filters for the wheel rpms 
        self.l_filter = lowpass( tau=0.2 )
        self.r_filter = lowpass( tau=0.2 )
        
        # Diff drive state 
        self.pose = matrix([0.0, 0.0, 0.0])
        self.state_t0 = now()
        return        
    
    def stop( self ):
        progress( "Stopping all modules" )
        self.r.LW.go_slack()
        self.r.RW.go_slack()
        self.r.Turret.go_slack()

    def update_control( self, phase ):        
        # Update control inputs for wheels
        self.r.LW.set_torque( self.LW_sign * (self.k_drive*self.drive - self.k_turn*self.turn) )
        self.r.RW.set_torque( self.RW_sign * (self.k_drive*self.drive + self.k_turn*self.turn) )

        # Update control input for turret based upon estimated orientation
        
        

    def update_pose( self , phase ):
        """
        Estimate pose of the diff drive
        """
        # Get rpm of each wheel
        RPM2RADS = 9.5493
        
        # Account for nonesense values 
        while True:
            l_rpm = self.LW_sign*self.r.LW.get_speed()
            if abs(l_rpm) < 70.0:
                break
        while True:
            r_rpm = self.RW_sign*self.r.RW.get_speed()
            if abs(r_rpm) < 70.0:
                break                
        l_rads = l_rpm/RPM2RADS
        r_rads = r_rpm/RPM2RADS
        progress( "l_rads: %5.2f, r_rads: %5.2f" % (l_rads, r_rads) )
        
        # Convert wheel rpm values into velocity and angular rate
        ### NOTE: I need to get all the conversions correct here
        wheel_radius = 0.05
        base_width = 0.24
        v = ((r_rads + l_rads)*wheel_radius)/2.0
        w = ((r_rads - l_rads)*wheel_radius)/base_width
        qdot = matrix([v, w])
        progress( "v: %5.2f, w: %5.2f" % (v, w) )

        # Estimate pose by integrating x_velocity, y_velocity and angular rate 
        theta = self.pose[0,2]
        J = matrix( [[-sin( theta ), 0],
                     [ cos( theta ), 0],
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
        # Handle keyboard events 
        if evt.type == KEYDOWN:
            if evt.key == K_SPACE:
                if self.control_fcp.isRunning():
                    self.control_fcp.stop()
                else:
                    self.control_fcp.start()            
                if self.pose_fcp.isRunning():
                    self.pose_fcp.stop()
                else:
                    self.pose_fcp.start()

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

        return                 

if __name__ == "__main__":
    
    app = diffDriveApp()
    app.run()
        
        
