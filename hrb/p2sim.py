#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Mar 23 01:14:25 2020

@author: shrevzen
"""
from numpy import ( 
    asarray, stack, ones, identity, dot, newaxis, cumsum, c_
)
from numpy.linalg import inv
from arm import Arm, jacobian_cdas
from motorsim import MotorModel,TH,BL
from pylab import gca
from joy.plans import AnimatorPlan

class MassArm(Arm):
    def __init__(self):
      Arm.__init__(self)
  
    def setup(self,wl):
      Arm.setup(self,wl)
      m = ones(self.geom[1].shape[1])
      CoM = [ asarray([0,0,0,1]) ] # Center of mass
      M = [ 0 ] # Mass
      I = [ identity(3) ] # Geometric inertia (I/m)
      for gn in self.geom[1:]:
        # CoM position
        M.append(sum(m))
        com = dot(gn,m)/M[-1]
        CoM.append(com)
        # Mass position offsets relative to CoM
        ofs = gn - com[:,newaxis]
        # Inertia matrix
        I.append(dot(m[newaxis,:]*ofs,ofs.T)/M[-1])
      # Link masses and inertias
      self.CoM = asarray(CoM)
      self.M = asarray(M)
      self.I = asarray(I)
      # Gravity torque is the gradient of gravitational potential energy
      doc = self.getGravityTorque.__doc__
      self.getGravityTorque = jacobian_cdas(
        lambda ang : asarray([self.getEgp(self.at(ang))]),
        ones(self.tw.shape[0])*0.05
      )
      self.getGravityTorque.__doc__=doc

    def getCoMs( self, A ):
      """
      Find the CoM-s of all segments

      INPUT:
        A -- list of transformations -- output of .at()

      OUTPUT: n x 3
        Array of CoM-s of all segments
      """
      return asarray([dot(a,com) for a,com in zip(A,self.CoM)])

    def getEgp( self, A ):
      """
      Get gravitational potential energy associated with a configuration
      """
      return dot(self.M,self.getCoMs(A)[:,2])

    def getGravityTorque(self,ang):
      """
      Compute the torque exerted by gravity on each of the joints (up to scale)
      
      INPUT:
        ang -- N -- joint angles
      
      OUTPUT: 1 x N
        torque on each of the joints
      """
      raise RuntimeError("should be overriden by constructor")
  
    def getFrzI( self, A ):
      """
      Compute the frozen chain inertia for all segments.
      The "frozen chain inertia" is the inertia of the remainder
      of the kinematic chain if all joints were locked.

      INPUT:
        A -- list of SE(3) -- output of .at()

      OUTPUT:
        list of I matrices -- the inertia matrix of the frozen chain relative to the center of mass
      """
      # Compute inverse transforms
      iA = [inv(a) for a in A]
      # Transform all inertias to world frame
      #  Each geometric inertia needs to be multiplied by the
      #  mass of the segment, and have its points transformed
      Ik = asarray([ dot(dot(a,ii*m),a.T)
             for a,ii,m in zip(A,self.I,self.M) ])
      # Cumulative inertia of the trailing segments from k out
      Ic = cumsum(Ik[::-1,...],0)[::-1]
      # Computer frozen inertia from each segment out
      fI = [ dot(dot(ia,ic),ia.T) for ia,ic in zip(iA,Ic) ]
      return fI
    
    def plot3D(self,A,ax=None):
      Arm.plot3D(self,A,ax)
      ax = gca()
      cx,cy,cz,_ = self.getCoMs(A).T
      ax.plot3D(cx,cy,cz,'ow')
      ax.plot3D(cx,cy,cz,'+k')
               
class ArmSim(MassArm):
    def __init__(self,wlc):
        wlc = asarray(wlc)
        MassArm.__init__(self)
        self.setup(wlc[:-1])
        self.c = wlc[-1]
        self.m = []
        for k in range(wlc.shape[1]):
          self.m.append(MotorModel())

    def __getitem__(self,idx):
        return self.m[idx % len(self.m)]
    
    def __len__(self):
        return len(self.m)
      
    def step(self,h):
        """
        Do an integration step for the whole arm. 
        
        THEORY OF OPERATION:
          This is a rather tricky thing to do, because the RK integrator 
        requires multiple function evaluations per time point. To support this,
        each motor model has an internal iterator that generates these quadrature
        points. Thus, we call next() on all the motor models to obtain their
        quadrature points, then compute the coupling term (gravity torque), 
        set it up for them, and let them compute the next quadrature point,
        until we are done. Then we return the results.
        """
        its = [ m.stepIter(h) for m in self.m ]
        while True:
          # Collect the next RK quadrature point
          #   we assume the cont value and timestamp will agree between motors
          #   so we only collect that from the first motor
          cont,(t,y0) = next(its[0])
          y = [y0]+[ next(mi)[1][1] for mi in its[1:] ]
          ang0 = asarray([ yi[TH]+yi[BL] for yi in y]) # motor angles
          if not cont:
            break
          tq = 3e-3*self.getGravityTorque(ang0).squeeze() # gravity torque on motors
          for mi,tqi in zip(self.m,tq): # push into motor objects
            mi._ext = -tqi
        ang1 = ang0 - self.c*tq # sagged angles
        return t,ang1,stack(y,1)
    
from joy import JoyApp
from joy.decl import KEYDOWN,K_q,K_ESCAPE,progress
from joy.misc import requiresPyGame
requiresPyGame()

class ArmAnimatorApp( JoyApp ):
    def __init__(self,wlc,*arg,**kw):
      JoyApp.__init__(self,*arg,**kw)
      self.arm = ArmSim(wlc)
      
    def onStart(self):
      self.ani = AnimatorPlan(self,self._animation)
      self.t,self.q,self.y,self.p = [],[],[],[]
      self.T0 = self.now
      self.ani.start()
    
    def _animation(self, fig):
      last = self.T0
      ax = fig.add_subplot(111,projection='3d')
      while True:
        # If enough time elapsed to animate
        now = self.now
        h = now - last
        if h<0.1:
          yield
          continue
        ti,qi,yi =self.arm.step(h) # Speedup ratio into simulation time
        # Store simulation time-step
        self.t.append(ti)
        self.q.append(qi)
        self.y.append(yi)
        self.p.append(self.arm.getTool(qi))
        # Display it
        ax.cla()
        self.arm.plot3D(self.arm.at(qi),ax)
        p = asarray(self.p[-20:]).T
        ax.plot3D(p[0],p[1],p[2],'.-k',alpha=0.3)
        ax.set_title("t = %.2f" % ti)
        last = now
        progress("TEMP: "+" ".join(["%d:%-6.2f" % (n,self.arm[n].get_temp()) for n,qqi in enumerate(qi)]))
        yield

    def onEvent(self,evt):
      # Punt everything except keydown events
      if evt.type != KEYDOWN or evt.key in {K_q, K_ESCAPE}:
        return JoyApp.onEvent(self,evt)
      # row of 'a' on QWERTY keyboard increments motors
      p = "asdfghjkl".find(evt.unicode)
      if p>=0:
        self.arm[p].set_pos(self.arm[p].get_pos() + 500)
        return
      # row of 'z' in QWERTY keyboard decrements motors
      p = "zxcvbnm,.".find(evt.unicode)
      if p>=0:
        self.arm[p].set_pos(self.arm[p].get_pos() - 500)
        return
      
      
    def onStop(self):
      from pylab import figure,savefig
      p = asarray(self.p).T
      fig = figure(1)
      ax3d = fig.add_subplot(221,projection='3d')
      ax3d.plot3D(p[0],p[1],p[2],'.-k')
      ax = fig.add_subplot(222)
      ax.plot(p[0],p[1],'.-k')
      ax.set(xlabel="x",ylabel="y")
      ax.axis('equal')
      ax.grid(1)
      ax = fig.add_subplot(224)
      ax.plot(p[0],p[2],'.-k')
      ax.set(xlabel="x",ylabel="z")
      ax.axis('equal')
      ax.grid(1)
      ax = fig.add_subplot(223)
      ax.plot(p[1],p[2],'.-k')
      ax.set(xlabel="y",ylabel="z")
      ax.axis('equal')
      ax.grid(1)
      savefig("result.png")
      t = asarray(asarray(self.t)*100,int)
      pout = c_[t,asarray(p[:3]*100,int).T]
      with open("result.csv","w") as rf:
        for pp in pout:
          rf.write(repr(list(pp))[1:-1]+"\n")
      
      
if __name__=="__main__":
    app = ArmAnimatorApp(asarray([
        [0,0.02,1,4,1],
        [0,1,0,4,1],
        [0,1,0,4,1.],
    ]).T)
    app.run()
    