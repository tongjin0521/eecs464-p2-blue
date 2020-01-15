'''
FILE demo-shaveandhaircut.py

This file demonstrates how to use plans sequentially or to use a single plan based
on the argument given. It operates plans sequentially by passing multiple plans as
arguments and yielding one after the other.
'''
from joy.decl import *
from joy import JoyApp, Plan, SheetPlan
from joy.misc import loadCSV

class ShaveNHaircutPlan( Plan ):
  """
  ShaveNHaircutPlan shows a simple example of sequential composition:
  its behavior is to run the shave plan followed by the haircut plan.
  *use two modules

  It starts shave and haircut sequentially by yielding the plans one after the other

  It is a useful demonstration of operation of multiple plans. It expects the motors
  to have mode=0
  """
  def __init__(self,app,shave,haircut,*arg,**kw):
    Plan.__init__(self,app,*arg,**kw)
    self.shave = shave
    self.haircut = haircut

  def behavior( self ):
    progress("Both: starting 'Shave' sequence")
    yield self.shave
    #this will start shave and wait for its completion
    progress("Both: starting 'Haircut' sequence")
    yield self.haircut
    #this starts haircut plan and waits for its completions
    progress("Both: done")

class ShaveNHaircutApp( JoyApp ):
  '''
  This decides the sequence based on users input

  it uses onEvent to receive user input and starts the plans accordingly which are based on
  a CSV spreadsheet and uses sheetPlan to execute the plans

  Its useful for those who want to use plans sequentially
  '''
  SHAVE = loadCSV("shave.csv")
  HAIRCUT = loadCSV("haircut.csv")

  def __init__(self,shaveSpec,hairSpec,*arg,**kw):
    JoyApp.__init__(self, *arg,**kw)
    self.shaveSpec = shaveSpec
    self.hairSpec = hairSpec

  def onStart(self):
    #
    # Use a sheetplan to realize SHAVE, outputting to self.shaveSpec
    #
    self.shaveplan = SheetPlan(self, self.SHAVE, x=self.shaveSpec )
    # give us start and stop messages; in your own code you can omit these
    self.shaveplan.onStart = lambda : progress("Shave: starting")
    self.shaveplan.onStop = lambda : progress("Shave: done")
    #
    # Use a sheetplan to realize H, outputting to s
    #
    self.hairplan = SheetPlan(self, self.HAIRCUT, x=self.hairSpec )
    # give us start and stop messages; in your own code you can omit these
    self.hairplan.onStart = lambda : progress("Haircut: starting")
    self.hairplan.onStop = lambda : progress("Haircut: done")
    #
    # Set up a ShaveNHaircutPlan using both of the previous plans
    #
    self.both = ShaveNHaircutPlan(self, self.shaveplan, self.hairplan)

  #acts if a key is pressed
  def onEvent(self,evt):
    if evt.type != KEYDOWN:
      return
    # assertion: must be a KEYDOWN event
    if evt.key == K_s:
      if ( not self.shaveplan.isRunning()
           and not self.both.isRunning() ):
        self.shaveplan.start()
    elif evt.key == K_h:
      if ( not self.hairplan.isRunning()
           and not self.both.isRunning() ):
        self.hairplan.start()
    elif evt.key == K_b:
      if ( not self.shaveplan.isRunning()
           and not self.hairplan.isRunning()
           and not self.both.isRunning() ):
        self.both.start()
    else: # punt to superclass
        return JoyApp.onEvent(self,evt)

if __name__=="__main__":
  from sys import argv, stdout, exit
  #give default values to the command line arguements
  robot = None
  scr = None
  shaveSpec = "#shave "
  hairSpec = "#haircut "
  #process the command line arguements
  args = list(argv[1:])
  while args:
    arg = args.pop(0)
    if arg=='--mod-count' or arg=='-c':
    #detects number of modules specified after -c
      N = int(args.pop(0))
      robot = dict(count=N)
    elif arg=='--shave' or arg=='-s':
    #detects the shavespec specified after -s  which basically controls the type of output we will see for shave
      shaveSpec = args.pop(0)
      if shaveSpec[:1]==">": scr = {}
    elif arg=='--haircut' or arg=='-h':
    #detects the shavespec specified after -s  which basically controls the type of output we will see for shave
      hairSpec = args.pop(0)
      if hairSpec[:1]==">": scr = {}
    elif arg=='--help':
    #help
      stdout.write("""
  Usage: %s [options]

    'Shave and a Haircut' example of running Plan-s in parallel and in
    sequence. The example shows that plans can run in parallel, and that
    Plan behaviors (e.g. the ShaveNHaircutPlan defined here) can use other
    plans as sub-behaviors, thereby "calling them" sequentially.

    When running, the demo uses the keyboard. The keys are:
      's' -- start "Shave"
      'h' -- start "Haircut"
      'b' -- start "Both", calling "Shave" and "Haircut" in sequence
      'escape' -- exit program

    Options:
      --mod-count <number> | -c <number>
        Search for specified number of modules at startup

      --shave <spec> | -s <spec>
      --haircut <spec> | -h <spec>
        Specify the output setter to use for 'shave' (resp. 'haircut')

        Typical <spec> values would be:
         '#shave ' -- to print messages to the terminal with '#shave ' as prefix
         '>x' -- send to Scratch sensor 'x'
         'Nx3C/@set_pos' -- send to position of CKBot servo module with ID 0x3C

        NOTE: to use robot modules you MUST also specify a -c option

    """ % argv[0])
      exit(1)
    # ENDS cmdline parsing loop
  #start an interface
  app = ShaveNHaircutApp(shaveSpec,hairSpec,robot=robot,scr=scr)
  #directs to onStart
  app.run()
