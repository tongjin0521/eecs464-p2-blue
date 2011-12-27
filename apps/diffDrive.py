from joy import *
import ckbot.dynamixel as dynamixel
import ckbot.logical as logical
from math import copysign, cos, sin, pi

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
        return        
    
    def stop( self ):
        progress( "Stopping all modules" )
        self.r.L.go_slack()
        self.r.R.go_slack()
        self.r.Turret.go_slack()

    def onStart( self ):
        fcp = FunctionCyclePlan( self, self.update, N=20, maxFreq=15, interval=0.05 )
        fcp.onStart = curry( progress, "Initializing FCP" )
        fcp.onStop = curry( progress, "Stopping FCP" )
        self.fcp = fcp

        sf = StickFilter( self )
        sf.setLowpass( "joy0axis2", 5 )
        sf.setLowpass( "joy0axis3", 5 )
        
        sf.start()
        self.sf = sf
        
    def onStop( self ):
        # If FunctionCyclePlan is running, don't forget to stop it
        if self.fcp.isRunning():
            self.fcp.stop()
        # Also remember to stop all modules
        self.stop()
    

if __name__ == "__main__":
    
    app = diffDriveApp()
    app.run()
        
        
