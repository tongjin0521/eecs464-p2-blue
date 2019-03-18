# -*- coding: utf-8 -*-
"""
Created on Tue Jan 10 15:07:14 2017
This file reads data from a serial port and prints it line by line using a NBRPlan

@author: birds
"""
from joy import JoyApp, progress
from joy.plans import NBRPlan

class App(JoyApp):
  '''
  This is a concrete class which reads the data line by line from a
  serial port and prints the data along with their timestamp
  
  It does that using an NBRPlan which creates a list by reading the data in the serial port
  linearly along with a list of timestamps and prints them line by line while popping each line
  in the list

  It will be useful if want to read data from a microcontroller 
  '''
  def onStart(self):
    #fn is the path of the port
    self.nbr = NBRPlan(self,fn="/dev/ttyUSB1")
    self.nbr.start()
  
  def onEvent(self, evt):
    #ts represents the time stamp when the data is read
    #ln represents the list of lines in the port file 
    #the data is continously removed after being printed
    while self.nbr.ln:
      ts = self.nbr.ts.pop()
      ln = self.nbr.ln.pop()
      progress("== %6d %s" % (ts,repr(ln)))
    else:
      progress(" <<EMPTY>>",sameLine=True)
    return JoyApp.onEvent(self,evt)
    
if __name__=="__main__":
  app = App()
  app.run()
