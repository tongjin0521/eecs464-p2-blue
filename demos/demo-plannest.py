'''
FILE demo-plannest.py

This file demonstrates how to do subplan nesting.
subplan nesting is often used incorrectly so it is crucial to
use yield function to successfully control the schedule of multiple plans
'''
from joy import JoyApp, Plan, progress

class PlanTest(Plan):
  '''
  This class does subplan nesting which is visualized in the behaviour function

  In order to do subplan nesting, yield function is used to control the schedule
  of the plans in the plan hierarchy and if it is not used, nesting won't work
  appropriately

  This is used if we need to use plans in different levels which require
  schedueling
  '''
  def __init__(self,app,nm,sub):
    '''
    initializes the plan with plan name and subplans
    '''
    Plan.__init__(self,app)
    self.nm = nm
    self.sub = sub

  def behavior(self):
    # prints the start message
    progress(self.nm+" start")
    # starts an iterator which has 3 iterations for a given plan
    for k in range(3):
      #denotes the start of the original plan for the current iteration
      progress(self.nm+" step %d start" % k)
      #starts the subsequent subplans
      yield self.sub
      #returns to the original plan after all the subplans are completed for the same iteration
      progress(self.nm+" step %d end" % k)
    progress(self.nm+" end")

class App(JoyApp):
  '''
  Demostration of sub-plan nesting - plans p1 > p2 > p3 have 3-layer nesting, i.e.
    p1 - starts (iteration 1)
        p2 - starts (iter 1)
           p3 - starts (iter 1)
           p3 - ends
           p3 - starts (iter 2)
           p3 - ends
           p3 - starts (iter 3)
           p3 - ends
        p2 - starts (iter 2)
    etc.
  '''
  def onStart(self):
     # creats an object self.p3 which does not have a subplan
     self.p3 = PlanTest(self,"    P3",None)
     # self.p2 has a subplan of p3
     self.p2 = PlanTest(self,"  P2",self.p3)
     # self.p1 has a subplan of p2
     self.p1 = PlanTest(self,"P1",self.p2)
     # starts the first plan to begin sub-plan nesting
     self.p1.start()
#creates an interface object
app =App()
#directs to onStart
app.run()
