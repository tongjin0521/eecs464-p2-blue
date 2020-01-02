'''
FILE demo-scratchMyBot.py
This FILE deonstrates how to connect a scratch object directly to your modules.
Whenever the cat moves in the scratch file, SCRATCHUPDATE function will detect the
movement and move the motors using set_post
'''
from joy.decl import *
from joy import JoyApp

class ScrMyBot(JoyApp):
  '''
  This is a concrete class which takes the value of the cat's position in the scratch
  window and processes it to give the respective output on the motor

  It detects the cat movement using SCRATCHUPDATE event and takes the value and checks
  the limiting conditions of the motor and gives the output on the robot object for our 
  robot

  It will be useful for those who want to check the motor's output for different 
  positions. It expects the motors to have mode=0
  '''
  #initializes a robot with count being the number of modules
  def __init__(self):
    JoyApp.__init__(self,scr={},robot=dict(count=2))
    self.gain = 100
    self.limit = 5000
  #creates a node for our robot
  def onStart( self ):
    self.aNode = list(self.robot.values())[1]
  # This functio gives the output as we move the cat in the scratch window.
  # SCRATCHUPDATE will detect the cat movement and use set_pos function to
  # give out respective move on the motor
  def onEvent( self, evt ):
    if evt.type==SCRATCHUPDATE and evt.scr==0:
      if evt.var=='robotPos1':
        pos = evt.value * self.gain
        if pos>self.limit: 
         pos = self.limit
        elif pos<-self.limit: 
         pos = -self.limit
        self.aNode.set_pos( pos )
    elif evt.type != TIMEREVENT:
      JoyApp.onEvent(self,evt)
      
if __name__=="__main__":
  print('''
  DEMO: connecting Scratch sprite to robot
  ----------------------------------------
  
  When this program runs, the x position of the cat sprite on 
  the screen controls a servo in the robot.
  
  ''')
  smb = ScrMyBot()
  #directs to onStart
  smb.run()
  
