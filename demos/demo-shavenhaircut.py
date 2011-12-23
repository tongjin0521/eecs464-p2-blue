from joy import *

class ShaveNHaircutPlan( Plan ):
  """
  ShaveNHaircutPlan shows a simple example of sequential composition:
  its behavior is to run the shave plan followed by the haircut plan.
  """
  def __init__(self,app,shave,haircut,*arg,**kw):
    Plan.__init__(self,*arg,**kw)
    self.shave = shave
    self.haircut = haircut
    
  def behavior( self ):
    progress("Both: starting 'Shave' sequence")
    yield self.shave
    progress("Both: starting 'Haircut' sequence")
    yield self.haircut    
    progress("Both: done")
    
class ShaveNHaircutApp( JoyApp ):
  # Load both patterns from their CSV files
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
    elif evt.key == K_ESCAPE:
        self.stop()

if __name__=="__main__":
  robot = None
  scr = None
  shaveSpec = "#shave "
  hairSpec = "#haircut "
  args = list(sys.argv[1:])
  while args:
    arg = args.pop(0)
    if arg=='--mod-count' or arg=='-c':
      N = int(args.pop(0))
      robot = dict(count=N)
    elif arg=='--shave' or arg=='-s':
      shaveSpec = args.pop(0)
      if shaveSpec[:1]==">": scr = {}
    elif arg=='--haircut' or arg=='-h':
      hairSpec = args.pop(0)
      if hairSpec[:1]==">": scr = {}
    elif arg=='--help' or arg=='-h':
      sys.stdout.write("""
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
        
    """ % sys.argv[0])
      sys.exit(1)
    # ENDS cmdline parsing loop
  
  app = ShaveNHaircutApp(shaveSpec,hairSpec,robot=robot,scr=scr)
  app.run()
