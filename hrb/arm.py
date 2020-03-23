
from scipy.linalg import expm as expM
from mpl_toolkits.mplot3d import Axes3D
from numpy import (
  asarray, all, allclose, empty, empty_like, concatenate, cross, dot,
  ones, newaxis, identity, zeros_like, array, diag, sum
)
from pylab import (
  gcf, gca, plot, axis, clf, subplot, grid, xlabel, ylabel
)
def seToSE( x ):
  """
  Convert a twist (a rigid velocity, element of se(3)) to a rigid
  motion (an element of SE(3))

  INPUT:
    x -- 6 sequence
  OUTPUT:
    result -- 4 x 4

  """
  x = asarray(x,dtype=float)
  if x.shape != (6,):
    raise ValueError("shape must be (6,); got %s" % str(x.shape))
  #
  return expM(screw(x))

def screw( v ):
  """
  Convert a 6-vector to a screw matrix

  The function is vectorized, such that:
  INPUT:
    v -- N... x 6 -- input vectors
  OUTPUT:
    N... x 4 x 4
  """
  v = asarray(v)
  z = zeros_like(v[0,...])
  return array([
      [ z, -v[...,5], v[...,4], v[...,0] ],
      [ v[...,5],  z,-v[...,3], v[...,1] ],
      [-v[...,4],  v[...,3], z, v[...,2] ],
      [ z,         z,        z, z] ])

def unscrew( S ):
  """
  Convert a screw matrix to a 6-vector

  The function is vectorized, such that:
  INPUT:
    S -- N... x 4 x 4 -- input screws
  OUTPUT:
    N... x 6

  This is the "safe" function -- it tests for screwness first.
  Use unscrew_UNSAFE(S) to skip this check
  """
  S = asarray(S)
  assert allclose(S[...,:3,:3].transpose(0,1),-S[...,:3,:3]),"S[...,:3,:3] is skew"
  assert allclose(S[...,3,:],0),"Bottom row is 0"
  return unscrew_UNSAFE(S)

def jacobian_cdas( func, scl, lint=0.8, tol=1e-12, eps = 1e-30, withScl = False ):
  """Compute Jacobian of a function based on auto-scaled central differences.

  INPUTS:
    func -- callable -- K-vector valued function of a D-dimensional vector
    scl -- D -- vector of maximal scales allowed for central differences
    lint -- float -- linearity threshold, in range 0 to 1. 0 disables
         auto-scaling; 1 requires completely linear behavior from func
    tol -- float -- minimal step allowed
    eps -- float -- infinitesimal; must be much smaller than smallest change in
         func over a change of tol in the domain.
    withScl -- bool -- return scales together with Jacobian

  OUTPUTS: jacobian function
    jFun: x --> J (for withScale=False)
    jFun: x --> J,s (for withScale=True)

    x -- D -- input point
    J -- K x D -- Jacobian of func at x
    s -- D -- scales at which Jacobian holds around x
  """
  scl = abs(asarray(scl).flatten())
  N = len(scl)
  lint = abs(lint)
  def centDiffJacAutoScl( arg ):
    """
    Algorithm: use the value of the function at the center point
      to test linearity of the function. Linearity is tested by
      taking dy+ and dy- for each dx, and ensuring that they
      satisfy lint<|dy+|/|dy-|<1/lint
    """
    x0 = asarray(arg).flatten()
    y0 = func(x0)
    s = scl.copy()
    #print "Jac at ",x0
    idx = slice(None)
    dyp = empty((len(s),len(y0)),x0.dtype)
    dyn = empty_like(dyp)
    while True:
      #print "Jac iter ",s
      d0 = diag(s)
      dyp[idx,:] = [ func(x0+dx)-y0 for dx in d0[idx,:] ]
      dypc = dyp.conj()
      dyn[idx,:] = [ func(x0-dx)-y0 for dx in d0[idx,:] ]
      dync = dyn.conj()
      dp = sum(dyp * dypc,axis=1)
      dn = sum(dyn * dync,axis=1)
      nul = (dp == 0) | (dn == 0)
      if any(nul):
        s[nul] *= 1.5
        continue
      rat = dp/(dn+eps)
      nl = ((rat<lint) | (rat>(1.0/lint)))
      # If no linearity violations found --> done
      if ~any(nl):
        break
      # otherwise -- decrease steps
      idx, = nl.flatten().nonzero()
      s[idx] *= 0.75
      # Don't allow steps smaller than tol
      s[idx[s[idx]<tol]] = tol
      if all(s[idx]<tol):
        break
    res = ((dyp-dyn)/(2*s[:,newaxis])).T
    if withScl:
      return res, s
    return res
  return centDiffJacAutoScl

class Arm( object ):
  """
  class Arm

  Represents a series manipulator made of several segments.
  Each segment is graphically represented by a wireframe model

  ATTRIBUTES:
    tw --
  """
  def __init__(self):
      self.setup(asarray([[0,1,0,3],[0,0,1,3]]*3).T)

  def setup(self, wl):
    self.ll = wl[3]
    # arm geometry to draw
    d=0.2
    hexa = asarray([
        [ 0, d,1-d, 1, 1-d, d, 0],
        [ 0, 1,  1, 0,  -1,-1, 0],
        [ 0, 0,  0, 0,   0, 0, 0],
        [ 1, 1,  1, 1,   1, 1, 1],
    ]).T
    sqr = asarray([
        [ d, d, d, d, d, 1-d, 1-d, 1-d, 1-d, 1-d],
        [ 1, 0,-1, 0, 1, 1, 0,-1, 0, 1 ],
        [ 0, 1, 0,-1, 0, 0, 1, 0,-1, 0],
        [ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 ],
    ]).T
    geom = concatenate([
      hexa, hexa[:,[0,2,1,3]], sqr,
    ], axis=0)
    self.geom = [( asarray([[0,0,0,1]]) ).T ]
    #
    # Build twist matrices
    # Build wireframe of all segments in the world coordinates
    #
    tw = []
    LL = 0
    for n,ll in enumerate(self.ll):
      # Scale the geometry to the specifies link length (ll)
      # Shift it to the correct location (LL, sum of the ll values)
      gn = ( asarray([ll,1,1,1])*geom+[LL,0,0,0] ).T
      self.geom.append(gn)
      # Compute the twist for this segment; first get rotation axis
      w = wl[:3,n]
      # Velocity induced at the origin
      v = -cross(w,[LL,0,0])
      # Collect the twists
      tw.append( concatenate([v,w],0) )
      # Accumulate the distance along the arm
      LL += ll
    # Build an array of collected twists
    self.tw = asarray(tw)
    # Position of tool
    self.tool = asarray([LL,0,0,1]).T
    # overwrite method with jacobian function
    self.getToolJac = jacobian_cdas(
      self.getTool, ones(self.tw.shape[0])*0.05
    )

  def at( self, ang ):
    """
    Compute the rigid transformations for a multi-segment arm
    at the specified angles
    """
    ang = asarray(ang)[:,newaxis]
    tw = ang * self.tw
    A = [identity(4)]
    for twi in tw:
      M = seToSE(twi)
      A.append(dot(A[-1],M))
    return A

  def getTool( self, ang ):
    """
    Get "tool tip" position in world coordinates
    """
    # Get the rigid transformation for the last segment of the arm
    M = self.at(ang)[-1]
    return dot(M, self.tool)

  def getToolJac( self, ang ):
    """
    Get "tool tip" Jacobian by numerical approximation

    NOTE: implementation is a placeholder. This method is overwritten
    dynamically by __init__() to point to a jacobian_cdas() function
    """
    raise RuntimeError("uninitialized method called")

  def plotIJ( self, A, axI=0, axJ=1 ):
    """
    Display the specified axes of the arm at the specified set of angles
    """
    for a,g in zip(A, self.geom):
      ng = dot(a,g)
      plot( ng[axI,:], ng[axJ,:], '.-' )
    tp = dot(a, self.tool)
    plot( tp[axI], tp[axJ], 'hk' )
    plot( tp[axI], tp[axJ], '.y' )

  def plot3D(self, A, ax=None ):
    """
    Display the specified axes of the arm at the specified set of angles
    """
    if ax is None:
        ax = gcf().add_subplot(111,projection='3d')
    ax.plot3D( [-10]*4+[10]*4,[-10,-10,10,10]*2,[-10,10]*4,'w.')
    for a,g in zip(A, self.geom):
      ng = dot(a,g)
      ax.plot3D( ng[0,:], ng[1,:], ng[2,:], '.-' )
    tp = dot(a, self.tool)
    ax.plot3D( [tp[0]], [tp[1]], [tp[2]], 'hk' )
    ax.plot3D( [tp[0]], [tp[1]], [tp[2]], '.y' )

  def plotAll( self, ang ):
    """
    Plot arm in 3 views
    """
    A = self.at(ang)
    clf()
    ax = [-20,20,-20,20]
    subplot(2,2,1)
    self.plotIJ(A,0,1)
    axis('equal')
    axis(ax)
    grid(1)
    xlabel('X'); ylabel('Y')
    subplot(2,2,2)
    self.plotIJ(A,2,1)
    axis('equal')
    axis(ax)
    grid(1)
    xlabel('Z'); ylabel('Y')
    subplot(2,2,3)
    self.plotIJ(A,0,2)
    axis('equal')
    axis(ax)
    grid(1)
    xlabel('X'); ylabel('Z')
    ax = gcf().add_subplot(224,projection='3d')
    self.plot3D(A,ax)

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
               

