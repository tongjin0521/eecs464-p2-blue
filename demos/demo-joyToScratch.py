'''
FILE demo-joyToScratch.py
This file demonstrates the use of a joystick to give output in scratch window. It
uses broadcast and sensorUpdate functions to update the scratch window  
'''
from joy.decl import *
from joy import JoyApp

class JoyToScr(JoyApp):
  '''
  This is a concrete class which receives joystick input and gives output
  in the scratch window
  
  Scratch is connected to the code via a scr{} object
  The Joystick movements that are defined in onEvent are connected to scratch
  sensors using sensorUpdate function of the scratch object and the buttons 
  are mapped to scratch events using broadcast function of the scratch object

  It would be useful for those who want to perform scratch events using joystick
  '''
  def __init__(self):
    JoyApp.__init__(self,scr={})
    #This is the speed of the movement of the cat in scratch
    self.speed = 10
  
  #starts the buttons in the scratch window
  def onStart( self ):
    self.scr.sensorUpdate( j0ax1=0, j0ax0=0 )
    self.scr.broadcast( 'j0btn0', 'j0btn1' )
  
  #takes the joypad inputs and gives the output in scratch window
  def onEvent( self, evt ):
    if evt.type==JOYAXISMOTION and evt.joy==0:
      if evt.axis==0:
        self.scr.sensorUpdate( j0ax0=int(evt.value*self.speed) )
      elif evt.axis==1:
        self.scr.sensorUpdate( j0ax1=-int(evt.value*self.speed) )
    elif evt.type==JOYBUTTONDOWN and evt.joy==0:
      if evt.button==0:
        self.scr.broadcast( 'j0btn0' )
      elif evt.button==1:
        self.scr.broadcast( 'j0btn1' )
    #this statement calls the default statements from Joyapp whuch
    #include the quitting event
    elif evt.type != TIMEREVENT:
      JoyApp.onEvent(self,evt)
      
if __name__=="__main__":
  print '''
  DEMO: connecting joystick to Scratch
  ------------------------------------
  
  When this program runs, axes 0 and 1 of joystick 0 are mapped to
  Scratch sensors j0ax0 and j0ax1; joystick buttons 0 and 1 are
  mapped to events j0btn0 and j0btn1
  
  '''
  #creates an interface whcih will work forthe output on the scratch window
  j2s = JoyToScr()
  j2s.run()
  
