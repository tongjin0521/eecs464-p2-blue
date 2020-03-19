from numpy import zeros, nan, asarray, sin, cos, tanh, concatenate
from numpy.random import randn
from ckbot.ckmodule import MissingModule
from integro import odeDP5

def clip(a,mn,mx):
    return mn if a<mn else (mx if a>mx else a) # Hybrid clip
    # return mn + (mx-mn)*tanh((a-mn)/(mx-mn)) # Smooth clip

def satFb( x, mx, dx ):
    return dx if abs(x)<mx or x*dx<0 else 0

TH, OM, BL, EI, TMP,TQ0,TQ1,TQ2,TQ3,TQD,POS,GV,GP = list(range(13))

class MotorModel( object ):
    def __init__(self):
        self.ode = odeDP5(self._flow)
        self.ode.aux = self._storeAux
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
        self.y = [zeros((2,13))]
        self.t = [zeros(2)]
        self.y[0][0,:]=nan
        self.t[0][0]=nan
        self._aux = {} # store for aux outputs

    def _ext(self, t, th):
        return 0

    def _storeAux( self, t, *arg ):
        # Take the aux values associated with the actual time-point emitted
        res = self._aux[t]
        self._aux = {}
        return res

    def _flow(self,t,y,p):
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
        self._aux[t] = tqc0,tqc1,tqc2,tqc3,tqd,th-bl,self.goalVel,gp
        return om,dom,dbl,dei,dtmp

    def runFor(self, dt, ics=None):
        if ics is not None:
            ics = asarray(ics)
            assert ics.size == self.y[-1][-1].size
        else:
            ics = self.y[-1][-1]
        t,y = self.ode(ics[:5],self.t[-1][-1],self.t[-1][-1]+dt)
        self.y.append(y)
        self.t.append(t)
        return t[-1], y[-1]

    def get_t(self):
        return concatenate(self.t)

    def get_pos(self):
        pos = concatenate([yi[:,POS] for yi in self.y])
        enc = (pos % 6.28318) + randn(*pos.shape)*0.03
        return enc * 18000/3.14159

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
