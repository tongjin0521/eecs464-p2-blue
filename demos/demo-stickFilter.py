from joy import *

class StickFilterApp( JoyApp ):
  def __init__(self,*arg,**kw):
    JoyApp.__init__(self,*arg,**kw)
  
  def onStart( self ):
    # Set up a StickFilter Plan
    sf = StickFilter(self)
    # Event channels can be names using a pygame event object, or using string
    #   names that follow a very strict structure. For tutorial puposes we
    #   show both options here.
    evt = pygame.event.Event(JOYAXISMOTION,joy=0,axis=0,value=None)
    # This channel is lowpass filtered -- useful for direct position control    
    sf.setLowpass( evt, 5 )
    # This channel is integrated, saturating at +/- 9.
    sf.setIntegrator( "joy0axis1", lower=-9, upper=9)
    # This channel is integrated, saturating at +/- 9.
    sf.setIntegrator( "midi3sc1_slider2", lower=-9, upper=9 )
    # Start it up, otherwise nothing will happen...
    sf.start()
    self.sf = sf
    # Set up a temporal filter for controlling the rate of our messages to the
    #  operator. In this case, we want 4 messages a second
    self.timeToShow = self.onceEvery(0.25)
        
  def onEvent( self, evt ):
    # If its time to show the operator a status message --> do so
    if self.timeToShow():
      progress("State x %.3f y %.3f %.3f" % (
        self.sf.getValue("joy0axis0"), self.sf.getValue("joy0axis1"), self.sf.getValue("midi3sc1_slider2") 
      ))
    # Send all joystick motion events to our StickFilter
    if evt.type in [JOYAXISMOTION,MIDIEVENT]:
      self.sf.push(evt)
      return
    # If we got this far, let our superclass handle the event 
    #  This provides a debug printout and the option to quit via [escape] or [q]
    JoyApp.onEvent(self,evt)

if __name__=="__main__":
  print """
    Demonstration of the StickFilter plan
    -------------------------------------
    
    Shows the use of the StickFilter plan, which allows arbitrary
    linear filters to be applied to input channels. This includes
    integrating joystick inputs, smoothing encoder measurements, 
    etc.
    
    The example shows how axis 0 can be set to low-pass filtering,
    and axis 1 to integration.
  """
  app = StickFilterApp()
  app.run()
