"""
FILE: demo-animator.py


"""
from joy import JoyApp
from joy.plans import AnimatorPlan
from joy.decl import KEYDOWN  
from numpy import linspace, sin

def animationGen(fig):
  th = linspace(0,6.28*4,200)
  while True:
    for k in th:
      fig.clf()
      ax = fig.gca()
      ax.plot( th, sin(th+k), 'b-')
      ax.plot( th[::4], sin((th[::4]+k)*2), 'r.-' )
      yield
      
class App(JoyApp):
  def onStart(self):
      AnimatorPlan(self,animationGen).start()

  def onEvent(self,evt):
      if evt.type == KEYDOWN:
        return JoyApp.onEvent(self,evt)
        
if __name__=="__main__":
  app = App()
  app.run()
  