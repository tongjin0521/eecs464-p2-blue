#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Mar 23 01:14:25 2020

@author: shrevzen
"""
from numpy import ( 
    asarray, stack, zeros, ones, identity, dot, newaxis, cumsum
)
from numpy.linalg import inv
from arm import Arm, jacobian_cdas
from motorsim import MotorModel,TH,BL,NDim
from pylab import gca

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
               


class P2Sim:
    def __init__(self,wlc):
        wlc = asarray(wlc)
        self.a = MassArm()
        self.a.setup(wlc[:-1])
        self.c = wlc[-1]
        self.m = []
        for k in range(wlc.shape[1]):
          self.m.append(MotorModel())

    def __getitem__(self,idx):
        return self.m[idx]
    
    def __len__(self):
        return len(self.m)
      
    def step(self,h):
        its = [ m.stepIter(h) for m in self.m ]
        while True:
          # Collect the next RK quadrature point
          cont,(t,y0) = next(its[0])
          y = [y0]+[ next(mi)[1][1] for mi in its[1:] ]
          ang0 = asarray([ yi[TH]+yi[BL] for yi in y]) # motor angles
          if not cont:
            break
          tq = 1e-3*self.a.getGravityTorque(ang0).squeeze() # gravity torque on motors
          for mi,tqi in zip(self.m,tq): # push into motor objects
            mi._ext = tqi
        ang1 = ang0 + self.c*tq # sagged angles
        return t,ang1,stack(y,1)
    
if __name__=="__main__":
    a = P2Sim(asarray([
        [0,1,0,3,1],
        [0,1,0,3,1],
        [0,1,0,3,1.],
    ]).T)
    a[0].set_pos(1000)
    a[1].set_pos(1000)
    a[2].set_pos(1000)
    t,q,y = [0],[zeros(len(a))],[zeros((NDim,len(a)))]
    while t[-1]<50:
      ti,qi,yi = a.step(0.1)
      t.append(ti)
      q.append(qi)
      y.append(yi)
      print("\r%.2f" % t[-1],end='',flush=True)
    t = asarray(t[1:])
    q = asarray(q[1:])
    y = asarray(y[1:])