import math
import centipedeContact as centCont

class turnInPlaceGait(centCont.contactGait):
    
    def __init__(self, params):
        centCont.contactGait.__init__(self, params, 1)
        
        
    def manageGait(self, phi):
        #don't really need front/stance/recovery with Lissajous trajectory
        #just do the calculation
        self.__calcRollYaw(phi)
        
    def __calcRollYaw(self, phi):
        '''Lissajous trajectory for roll/yaw
            yaw = sin(2t)
            roll = -sin(t)
            
            gives a figure-8
        '''
        pi = math.pi
        self.yaw = math.sin(2*phi*2*pi) * self.params.yawAmp 
        self.roll = -math.sin(phi*2*pi) * self.params.rollAmp
        
        return
        
        
