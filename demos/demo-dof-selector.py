'''
FILE demo-dof-selector.py

This file helps in binding a joystick axis to a dof(motor) and sets the gain and offset values
by using the joystick. It does that by storing all the values in a dictionary and has induvidual
funtions to change those values on receiving Joystick input
'''
from joy.decl import *
from joy import JoyApp
from joy.plans import *

class Simmod(object):
  def __init__(self):
    self.name = "Sim%02x" % id(self)
    self.set_pos = lambda x : None
    progress('Created %s ' % self.name) 
    
class DofSelectorApp( JoyApp ):
  '''
  This is a concrete class which gets the user's joystick inputs and performs the following 
  operations:
  Dof Select: This is used to bind a joystick axis to a motor
  Gain Select: This is used to set the gain value for the binded axis
  Off Select: This is used to set the offset value for the binded axis

  This file uses dictionaries as interface between joystick axes and modules 
  by using axis names as keys to modules, gain and offset values

  '''  
  def __init__(self,robot=None,*arg,**kw):
    #if robot is None:
    #  raise ValueError('This application requires a robot')
    JoyApp.__init__(self,*arg,robot=robot,
      cfg=dict( buttonWait = 0.1, axisWait = 0.2, defaultGain=500 ),
      **kw)
    
  def onStart( self ):
    #Get a list of the modules in a variable
    self.dof = list(self.robot.itermodules())
    #Get a list of the positions of the modules in a variable
    self.setter = [ m.set_pos for m in self.dof ]
    self.btns = set()
    self.gain = {}
    self.ofs = {}
    #It is a dictionary with the axes names as indices to the modules as values
    self.bind = {}
    self.mode = None
    self.btnTime = 0
    self.axisTime = 0

  def onClick(self,evt):
    '''
    This function is used to see the modules binded with axes
    '''
    #to set the proper keys for finding bindings
    if self.mode == evt.button:
      self.mode = None
    else:
      self.mode = evt.button
    progress('[[[ MODE %s ]]]] -- mode changed' % str(self.mode))
    #Check to see if the pressed button is binded
    if not self.bind.has_key(self.mode):
      progress("   >> no bindings defined") 
      return
    #print all the axes with binded modules
    for (joy,axis),dof in self.bind[self.mode].iteritems():
      progress("   >> joy %d axis %d -- module %s"
        % (joy,axis,self.dof[dof].name) )
    
  def doMultiButton( self, evt ):
    '''
    This function is used to check for a short button press
    '''
    if evt.type==JOYBUTTONDOWN:
      self.btns.add(evt.button)
      self.btnTime = self.now + self.cfg.buttonWait
      if 'b' in self.DEBUG:
        debugMsg(self,'Buttons ' +str(self.btns) )
      return
    elif evt.type==JOYBUTTONUP:
      # If button press was short, and was a single button --> click
      if (self.btns == set((evt.button,)) 
          and self.now < self.btnTime):
        self.onClick(evt)
      self.btns.discard(evt.button)
      if 'b' in self.DEBUG:
        debugMsg(self,'Buttons ' +str(self.btns) )
      return

  def getDofGainOfs( self, evt):
    '''
    This function returns the dof, gain and offset lists designated previously
    by the user
    '''
    # Find binding table for current mode
    dof = None
    bnd = self.bind.get(self.mode,None)
    if bnd: dof = bnd.get((evt.joy,evt.axis),None)
    if dof is None:
      return None,None,None
    # Find gain table for DOF-s in this mode
    g = self.gain.get(self.mode,None)
    # If gain table not found, create an empty one
    if g is None:
      g = [self.cfg.defaultGain] * len(self.dof)
      self.gain[self.mode] = g
    # Find ofs table for DOF-s in this mode
    o = self.ofs.get(self.mode,None)
    # If ofs table not found, create an empty one
    if o is None:
      o = [0] * len(self.dof)
      self.ofs[self.mode] = o
    # Return dof number and tables for this mode
    return dof,g,o    
      
  def doSelectDof( self, evt ):
    '''
    This function is used to bind a module to the axis being moved
    '''
    bnd = self.bind.get(self.mode,None)
    if bnd is None:
      bnd = {}
      self.bind[self.mode] = bnd
    key = (evt.joy,evt.axis)
    
    # Find the module binded for the axis being moved
    dof = bnd.get(key,0)
    # Cycle through DOF
    #if the axis movement is positive, it will select the next module for that axis 
    if evt.value>0:
      dof = (1+dof) % len(self.dof)
    #if the axis movement is negative, it will select the previous module for that axis
    else:
      dof = (len(self.dof)-1+dof) % len(self.dof)
    # Store back in bindings      
    bnd[key] = dof 
    progress("Selected module %s for axis %d" 
      % (self.dof[dof].name,evt.axis) )
    self.axisTime = self.now + self.cfg.axisWait

  def doSelectGain( self, evt ):
    '''
    This function is used to set the gain value for the module
    binded to the input axis
    '''
    dof,g,o = self.getDofGainOfs(evt)
    if dof is None:
      progress("Could not find binding")
      return
    if evt.value>0:
      g[dof] *= 1.1
    else:
      g[dof] /= 1.1
    progress("Module %s gain set to %.2g (in mode %s)" 
      % (self.dof[dof].name, g[dof],str(self.mode)))
    self.axisTime = self.now
    
  def doSelectOfs( self, evt ):
    '''
    This function is used to set the offset value for the module
    binded to the input axis
    '''
    dof,g,o = self.getDofGainOfs(evt)
    if dof is None:
      progress("Could not find binding")
      return
    if evt.value>0:
      o[dof] += 0.05
    else:
      o[dof] -= 0.05
    progress("Module %s offset set to %.2g (in mode %s)" 
      % (self.dof[dof].name, o[dof],str(self.mode)))
    self.axisTime = self.now

  def onEvent(self, evt):
    if evt.type==QUIT or (evt.type==KEYDOWN and evt.key==ord('q')):
      self.stop()
      return
    # 
    if evt.type in (JOYBUTTONUP,JOYBUTTONDOWN):
      self.doMultiButton(evt)
      return
    #
    if (len(self.btns)==2
        and self.now>self.btnTime 
        and evt.type==JOYAXISMOTION 
        and self.now>self.axisTime
        and abs(evt.value)>0.5):
      #
      if self.btns==set((4,6)):
        self.doSelectDof(evt)
        return
      #
      if self.btns==set((5,7)):
        self.doSelectGain(evt)
        return        
      #
      if self.btns==set((6,7)):
        self.doSelectOfs(evt)
        return       
    #
    if (evt.type==JOYAXISMOTION and not self.btns):
      dof,g,o = self.getDofGainOfs(evt)
      if dof is None:
        progress('Mode %s has no binding for %d' 
          % (str(self.mode),evt.axis) )
      else:
        val = (evt.value-o[dof])*g[dof]
        progress('Setting %s to %.2g' % (self.dof[dof].name, val ))
        self.setter[dof](val)
      
if __name__=="__main__":
  print("""
  """)
  import sys
  app = DofSelectorApp(robot={})
  app.DEBUG[:]=['b']
  app.run()

