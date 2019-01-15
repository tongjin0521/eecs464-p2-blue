"""
FILE: demo-animator.py

This file creates an animation using a given plot. It does that on an infinite loop with changing frames

"""
from joy import JoyApp
from joy.plans import AnimatorPlan
from joy.decl import KEYDOWN
from numpy import linspace, sin
 
def animationGen(fig):
  '''
  animationGen-erator

  animation generator gets a figure handle(It is a window ready for drawing). It is supposed to draw an animation frame
  on the figure than yield

  It creates a motion picture effect by shifting initial angle in eachframe for a continuous stream of frames

  It will be useful for studying the motion of a continuously moving motor

  '''
  #th spans four cycles
  th = linspace(0,6.28*4,200)

  while True:#loop forever for creating frames
    for k in th:#making waveforms move by shifting initial angle
      fig.clf()
      ax = fig.gca()
      ax.plot( th, sin(th+k), 'b-')
      ax.plot( th[::4], sin((th[::4]+k)*2), 'r.-' )
      yield
#The interface which handles our input
class App(JoyApp):
  #calls a cocrete class called AnimatorPlan which handles how the frames move through the parameters such as fps, delay etc
  #and makes them move by .animate() function in the plan behaviour
  def onStart(self):
      AnimatorPlan(self,animationGen).start()

  def onEvent(self,evt):
      if evt.type == KEYDOWN:
        return JoyApp.onEvent(self,evt)

if __name__=="__main__":
  app = App()
  app.run()
