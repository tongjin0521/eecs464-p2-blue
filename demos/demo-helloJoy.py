'''
FILE demo-helloJoy.py

This demonstrates basic joyapp programming and it converts your mouse postion to y-value and prints it on your screen
'''
from joy.decl import *
from joy import JoyApp

class HelloJoyApp( JoyApp ):
  """HelloJoyApp

     The "hello world" of JoyApp programming.
     This JoyApp pipes the y coordinate of mouse positions (while left
     button is pressed) to a specified setter. By default this setter is
     given by "#output " -- i.e. it is a debug message.

     See JoyApp.setterOf() for a specification of possible outputs

     This file is intended to introduce you to our joy library
  """
  def __init__(self,spec,*arg,**kw):
    # This is a "constructor". It initializes the JoyApp object.
    # Because we added an additional parameter, we extend the
    # JoyApp constructor. The first step is to call the superclass
    # constructor so that we'll have a valid JoyApp instance.
    JoyApp.__init__(self, *arg,**kw)
    # Store output specifier for later use
    self.spec = spec

  def onStart(self):
    # This function is called when the JoyApp is ready to start up,
    # i.e. after all PyGame devices have been activated, robot Cluster
    # is populated, scratch interface is live, etc.
    self.output = self.setterOf(self.spec)

  def onEvent(self,evt):
    # Supress bogus VM events that convert mouse to joystick
    if evt.type==JOYAXISMOTION and evt.joy==1:
      return
    # All unknown events --> punt to superclass
    if evt.type != MOUSEMOTION or evt.buttons[0] == 0:
      return JoyApp.onEvent(self,evt)
    # If we reach this line, it was a MOUSEMOTION with button pressed
    #   so we send the value out to the output
    self.output( evt.pos[1] )
if __name__=="__main__":
  print("""
  Uses MOUSEMOTION events; you may need to restart your 'ctrl' process with:
  ctrl -e MOUSEMOTION

  This demo will output mouse drag x coordinates to the designated output CLP.
  If mouse button isn't pressed, the event is punted to superclass.
  """)
  app = HelloJoyApp("#output ")
  app.run()
