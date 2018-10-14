#!/usr/bin/python

#the above statement tells python to use a particular interpreter if you have multiple versions
#of python 
'''
FILE demo-rawPosable.py

This file demonstrates the recording and playback of the positions of all the modules using 
different commands
It records the position of the module by get_pos function and saves the position in a list
It will iterate through that list for playback
'''

import ckbot.logical as L
import cmd
from sys import stdout as STDOUT
from time import sleep, time as now
from exceptions import StandardError

class RawPosable( cmd.Cmd ):
  '''
  This is a concrete class which creates a cmd interface with defined commands
  for recording and playback using its own defined functions

  Each command is defined in its own function and the interface is created by 
  inheriting cmd.Cmd. It records the position of the modules by get_pos callback 
  for each module and plays the reocrding by set_pos callbacks. 

  It is useful for those who want to create their own playback and recording system
  for the modules  
  '''
  def __init__(self):
    cmd.Cmd.__init__(self)#This creates a cmd interface
    self.c = L.Cluster()#this creates objects for the connected modules
    self.p = []#poses
    self.ngs = None 
    '''
    an array of tuples representing each module
    Each tuple has three values representing module name, get_pos callback
    and set_pos callback
    ''' 
    self.prompt = "RawPose> "
  
  def do_populate(self,line):
    '''
    Creates an array of modules using the modules discovered by populating
    a Cluster
    '''
    try:
      n = eval(line)
    except StandardError,err:
      print err
      return
    if type(n) != tuple:
      n = (n,)
    self.c.populate(*n)
    self.ngs = [
        (m.name,m.get_pos,m.set_pos) 
        for m in self.c.itermodules()
    ]
    self.ngs.sort()

  def do_who(self,line):
    '''
    Print the list of modules discovered by populate command 
    '''
    print self.c.p.scan()
  
     
  def do_EOF(self,line):
    ''' quit the cmd '''
    self.do_off('')
    return True
    
  #turn off the execution
  def do_off(self,line=''):
    '''turn off the execution'''
    for m in self.c.itermodules():
      if hasattr(m,'go_slack'):
        try:
          m.go_slack()
        except Error,err:
          print Error,err
  #delete the last pose from the recording
  def do_drop(self,line=''):
    '''Delete the last recorded pose'''
    if self.p:
      self.p.pop(-1)
    print "%d poses remain" % len(self.p)
  #add a pose to the recording  
  def emptyline(self):
    '''Record a Pose ( use populate command  first)'''
    if self.ngs is None:
      print "poplate the cluster first!"
      return
    print "Adding pose",len(self.p)+1
    #Use the get_pos callback for each module
    v = [x[1]() for x in self.ngs]
    print " ".join(["%6s " % n for n,g,s in self.ngs ])
    print " ".join(["%6d " % vi for vi in v])
    self.p.append( v )
  
  def do_execute(self,line=''):
    '''Play the recording  '''
    #The arguement line specifies the time interval between each pose
    #If line is empty, it will be set to 0.1
    if not line.strip():
      dt = 0.1
    else:
      dt = eval(line)
    err = None
    try:
      t0 = now()
      for pose in self.p:
        print "%6.2f " % (now()-t0),
        print " ".join(["%6s " % n for n,g,s in self.ngs ])
        print "%6s" % ':',
        print " ".join(["%6d " % vi for vi in pose])
        STDOUT.flush()#It will as soon as possible without clearing the buffer
        # s is module_name.setpos callback which is used to set the motor to the desired position
        #Use set_pos on all the modules
        for v,(n,_,s) in zip(pose,self.ngs):#v is pose and s is set_pos callback
          s(v)
        sleep(dt)
    except Error,err:
      pass
    self.do_off()
    if err:
      raise err
  
  #this function checks the voltage all the time    
  def postcmd(self,stop,line):
    self.c.p.update()
    try:
      #It is an iterator
      m = self.c.itermodules().next()
      v = m.get_voltage()
      if v<16:
        print "Voltage:",v
      if v<14:
        print "*** LOW VOLTAGE *** force quit!"
        self.do_off()
        stop = True
    except StopIteration:
      pass
    return stop
    
if __name__=="__main__":
  '''
  Basic procedure:
  1. Run the file without arguements
  2. Use populate command with the number of modules connected as arguement
  3. Record poses by pressing enter ie creating an emptyline
  4. Delete the poses usig drop command
  5. Use execute command to play the recording
  6. Use EOF command to exit
  '''
  app = RawPosable()
  try:
    app.cmdloop()
  finally:
    app.do_off()
