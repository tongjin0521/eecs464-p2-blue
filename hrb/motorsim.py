from numpy import zeros, asarray, asfarray, concatenate
from numpy.random import randint
#!!!from integro import odeDP5
  
def clip(a,mn,mx):
    return mn if a<mn else (mx if a>mx else a) # Hybrid clip
    # return mn + (mx-mn)*tanh((a-mn)/(mx-mn)) # Smooth clip

def satFb( x, mx, dx ):
    return dx if abs(x)<mx or x*dx<0 else 0

TH, OM, BL, EI, TMP,TQ0,TQ1,TQ2,TQ3,TQD,POS,GV,GP = list(range(13))

class MotorModel( object ):
    def __init__(self):
        #!!!self.ode = odeDP5(self._flow)
        #!!!self.ode.aux = self._storeAux
        self.goalPos = 0 # [rad] Goal position to move to
        self.goalVel = 0 # [rad/sec] Goal velocity to use
        self.maxSpeed = 1 # [rad/sec] Limit on commanded speeds used by controller
        ## PID gains
        self.Kp = 4. # [amp/rad]
        self.Kd = 3. # [amp/(rad/sec)]
        self.Ki = 1 # [amp/(rad*sec)]
        self.sat = 2.0 # [rad*sec] Integrator saturation
        self.tqLimit = 1. # [amp] Upper limit on torque (in amps)
        self.blMag = .1 # [rad] Gear backlash magnitude (rad)
        self.mu = .05 # [amp/(rad/sec)] dynamic friction
        self.inertia = 1. # [amp/(rad/sec**2)] motor inertia
        self.thrm = 0.05 # [amp**2/sec] thermal sink rate
        self.nolo = 2. # [rad/sec] No-load speed of motor
        self.cmax = 10. # [amp] Maximal drive power system can produce
        return self.clear()

    def clear(self):
        self.y = [zeros(TMP+1)]
        self.t = [0]
        self._aux = {} # store for aux outputs

    def _ext(self, t, th):
        return 0
    '''#!!!
    def _storeAux( self, t, *arg ):
        # Take the aux values associated with the actual time-point emitted
        res = self._aux[t]
        self._aux = {}
        return res
'''
    def _flow(self,t,y,p=None):
        th, om, bl, ei, tmp = y
        de = om - self.goalVel
        if self.goalPos is not None:
            e = th - self.goalPos
            dei = satFb(ei,self.sat,e)
        else:
            e = 0.
            dei = satFb(ei,self.sat,de)
        tqc0 = -self.Kp * e - self.Kd * de - self.Ki * ei
        tqc1 = clip(tqc0,-self.tqLimit, self.tqLimit)
        # Correct torque (current) for Back EMF (generated current)
        tqc2 = tqc1 - om/self.nolo
        tqc3 = clip( tqc2, -self.cmax, self.cmax )
        # Disturbance torque
        tqd = self._ext(t,th-bl)
        # Change in rotation speed includes torque and dynamic friction
        dom = (tqc3 - self.mu * om + tqd) / self.inertia
        ## Integrate blacklash
        dbl = satFb(bl,self.blMag,om)
        # Heat accumulates as a function of current squared
        dtmp = -tmp*self.thrm + tqc3*tqc3
        # Keep a copy of all auxillary outputs
        gp = self.goalPos if self.goalPos else 0
        #!!!self._aux[t] = tqc0,tqc1,tqc2,tqc3,tqd,th-bl,self.goalVel,gp
        self._aux = asfarray( (tqc0,tqc1,tqc2,tqc3,tqd,th-bl,self.goalVel,gp) )
        return asfarray( (om,dom,dbl,dei,dtmp) )

    def step(self,h):
        """
        Do one time-step with Runge-Kutta 4 integration (the classical method)
        INPUT:
          h -- float>0 -- duration of time-step
        """
        t0 = self.t[-1]
        y0 = self.y[-1]
        l = len(y0)
        y0 = y0[:TMP+1]
        tm = t0+h/2.
        t1 = t0+h
        k1 = h*self._flow(t0,y0)
        if l==TMP+1:
          self.y[-1] = concatenate([y0,self._aux])
        else:
          assert l == GP+1
        k2 = h*self._flow(tm,y0+k1/2.)
        k3 = h*self._flow(tm,y0+k2/2.)
        k4 = h*self._flow(t1,y0+k3)
        self.y.append(y0+(k1+2*k2+2*k3+k4)/6.)
        self.t.append(t1)
        return self.t[-1], self.y[-1]

    def get_ty(self):
        """ (INTERNAL)
        Get time history of the simulation
        """
        # Note: last data point never has aux; thus dropped
        return asarray(self.t[:-1]),asarray(self.y[:-1])
  
    def get_pos(self):
        """
        Obtain current position
        """
        return int(self.y[-2][GP] * 18000/3.14159) + randint(-200,200)
      
    def set_pos(self,pos):
        """
        Set desired position / None to use velocity control only
        """
        if pos is None:
            self.goalPos = None
            return
        self.goalPos = pos * 3.14159 / 18000.

    def set_rpm(self,rpm):
        """
        """
        self.goalVel = rpm * 3.14159 /30.0
