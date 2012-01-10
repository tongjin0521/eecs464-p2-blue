#!/usr/bin/python

import ckbot.logical as L
import cmd
from sys import stdout as STDOUT
from time import sleep, time as now

class RawPosable( cmd.Cmd ):
  def __init__(self):
    cmd.Cmd.__init__(self)
    self.c = L.Cluster()
    self.p = []
    self.ngs = None
    self.prompt = "RawPose> "
    
  def do_populate(self,line):
    try:
      n = eval(line)
    except Error,err:
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
    print self.c.p.scan()
    
  def do_EOF(self,line):
    self.do_off('')
    return True
    
  def do_off(self,line=''):
    for m in self.c.itermodules():
      if hasattr(m,'go_slack'):
        try:
          m.go_slack()
        except Error,err:
          print Error,err
  
  def do_drop(self,line=''):
    if self.p:
      self.p.pop(-1)
    print "%d poses remain" % len(self.p)
    
  def emptyline(self):
    if self.ngs is None:
      print "poplate the cluster first!"
      return
    print "Adding pose",len(self.p)+1
    v = [x[1]() for x in self.ngs]
    print " ".join(["%6s " % n for n,g,s in self.ngs ])
    print " ".join(["%6d " % vi for vi in v])
    self.p.append( v )

  def do_execute(self,line=''):
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
        STDOUT.flush()
        for v,(n,_,s) in zip(pose,self.ngs):
          s(v)
        sleep(dt)
    except Error,err:
      pass
    self.do_off()
    if err:
      raise err
      
  def postcmd(self,stop,line):
    self.c.p.update()
    try:
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
  app = RawPosable()
  try:
    app.cmdloop()
  finally:
    app.do_off()
