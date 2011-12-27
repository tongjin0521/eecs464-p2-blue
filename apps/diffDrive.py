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
        return        
    
    def stop( self ):
        progress( "Stopping all modules" )
        self.r.L.go_slack()
        self.r.R.go_slack()
        self.r.Turret.go_slack()

    def onStart( self ):
        

    

if __name__ == "__main__":
    
    app = diffDriveApp()
    app.run()
        
        
