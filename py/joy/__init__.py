#  This library is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public
#  License as published by the Free Software Foundation; either
#  version 3.0 of the License, or (at your option) any later version.
#
#  The library is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
#  General Public License for more details.
#
# (c) Shai Revzen, U Penn, 2010, U Michigan, 2020
#
#
#
"""
Overview
========

JoyApp provides an application framework for writing software for the CKBot
modular robots developed in [ ModLab ](http://modlabupenn.org).

The JoyApp framework is based on the excellent [ pygame ](http://www.pygame.org),
which it extends with several custom event types. Programmers using JoyApp will
invariably want to know about [ pygame event handling ][1].

Plans
-----

In addition to new event types, JoyApp provides support for *Plans*. Plans are
similar to threads in that they execute concurrently with each other. Unlike
threads, they cooperatively multitask by releasing control of the processor
using the Python keyword `yield`. In addition, Plans maintain their own event
queue and event handler. Their constructor offers facilities for binding the
plan to settable `ckbot.logical.Cluster` properties.

Debugging
---------

Debugging and notification are centrally accessed through the `progress`
function, which prints time-stamped messages on standard output. Many debugging
messages use `debugMsg` which identifies the object emitting the message as the
first parameter. Objects are identified using names generated by `dbgId`. The
actual debug messages generated are controlled by the singleton list `DEBUG`
which is copied into every JoyApp and Plan as `self.DEBUG`. This allows changes
to the contents of the module-wide `DEBUG` toaffect all objects, unless they
have individually overriden this behavior byusing some other list in their
`self.DEBUG`.

[1]: http://www.pygame.org/docs/ref/event.html
"""

# Standard library dependencies
import sys
from warnings import warn
from glob import glob
from os import getenv, sep as OS_SEP
from types import GeneratorType

# pygame interface or compatibility layer
from . import pygix

# Try to get matplotlib with Agg backend
try:
    from matplotlib import use as mpl_use
    mpl_use('Agg')
    from matplotlib.pylab import figure
except ImportError:
    print(">>> matplotlib not found; plotting disabled")
    def figure(*arg,**kw):
        return None

# YAML for configuration file parsing and formatting
import yaml

# Misc small functions
from . misc import *

# Interface to robots
import ckbot
from ckbot.logical import Cluster
from ckbot.ckmodule import AbstractServoModule,AbstractProtocolError

# Logging interface
from . loggit import progress,LogWriter,dbgId,debugMsg,PROGRESS_LOG

# Plan classes
from . plans import EventWrapperPlan
from . import remote
from . import safety

## Obtain settings from the os environment
# PYCKBOTPATH
try:
  if PYCKBOTPATH:
    pass
except NameError:
  # Library location specified by PYCKBOTPATH environment var
  PYCKBOTPATH = getenv('PYCKBOTPATH',__path__[0]+OS_SEP+'..'+OS_SEP+'..')
if PYCKBOTPATH[-1:] != OS_SEP:
  PYCKBOTPATH=PYCKBOTPATH+OS_SEP
# DEBUG flags
DEBUG = (getenv("JOYDEBUG",'')).split(",")

# Connect debug hooks of sub-modules
ckbot.logical.progress = progress

# Joy event classes and support functions
from . events import describeEvt, describeKey

# MIDI interface
try:
  from . midi import joyEventIter as midi_joyEventIter, init as midi_init
except ImportError:
  def midi_init():
    progress('*** MIDI support could not be started -- "import midi" failed ')
  def midi_joyEventIter():
    return []

# Useful constants
from . decl import *


class NoJoyWarning( UserWarning ):
  """
  Warning used to indicate unknown keys in a configuration file or cfg
  parameter of the JoyApp constructor.
  """
  pass

class JoyAppConfig( object ):
  """
  Instances of the concrete class JoyAppConfig store configuration settings in
  their attributes. The list of attribute names that are actually settings is
  stored in the private member __keys.

  JoyAppConfig contents can be loaded and save in YAML format.

  Typical usage:
  >>> cfg = JoyAppConfig( spam = 'tasty', parrot = 'blue' )
  >>> cfg.update( dict( parrot='dead' ) )
  >>> cfg.save('/tmp/Monty')
  >>> cfg2 = JoyAppConfig( spam = 'yuck!' )
  >>> cfg2.load('/tmp/Monty')
  >>> print cfg2.spam
  tasty
  """
  def __init__(self,**kw):
    self.__keys=set()
    self.update(kw)

  def update(self,kw):
    """Update settings from a dictionary"""
    self.__dict__.update(kw)
    self.__keys.update( set(kw.keys()) )

  def save( self, filename ):
    """
    Save configuration to a YAML file
    """
    if filename[-4:]!=".yml":
      filename = filename + ".yml"
    # Construct a dictionary from the configuration
    d = {}
    for key in self.__keys:
      d[key] = getattr(self,key)
    # Export to YAML
    f = None
    try:
      f = open(filename,"w")
      yaml.dump(d,f)
    except IOError as ioe:
      sys.stderr.write( "ERROR accessing '%s' -- %s" % (filename,str(ioe)))
    except yaml.YAMLError as yerr:
      sys.stderr.write( "ERROR creating YAML '%s' -- %s" % (filename,str(yerr)))
    finally:
      if f:
        f.close()

  def load(self,filename):
    """
    Load configuration from a YAML file
    """
    if filename[-4:]!=".yml":
      filename = filename + ".yml"
    f = None
    d = {}
    try:
      f = open(filename,"r")
      d = yaml.safe_load(f)
    except IOError as ioe:
      sys.stderr.write( "ERROR accessing '%s' -- %s" % (filename,str(ioe)))
    except yaml.YAMLError as yerr:
      sys.stderr.write( "ERROR parsing YAML '%s' -- %s" % (filename,str(yerr)))
    finally:
      if f:
        f.close()
    for key,val in d.items():
      if not key in self.__keys:
        warn("Unknown configuration key '%s' ignored" % key,NoJoyWarning)
        continue
      setattr(self,key,val)

class JoyApp( object ):
  """
  Framework for robot and pygame combined applications, with support
  for cooperative multithreading (using Plan subclasses)

  A JoyApp collects together several useful objects in one place. It also
  provides lifecycle event callbacks that should be overriden by subclasses.

  Lifecycle -- JoyApp Creation
  ============================
  __init__ constructor, specifying parameters for:
    self.robot - a ckbot.logical.Cluster
    self.cfg - a JoyAppConfig.

  NOTE: Plan.start should never be called from a JoyApp.__init__ constructor.

  Lifecycle -- JoyApp Execution
  =============================
  run, which executes the remainder of the life-cycle including the main loop.
  This consists of

   1. onStart is called after successful initialization of all sub-systems.
      Plan.start can be called from here; this
      is a recommended location for starting Plan instances.

   2. multiple calls to onEvent, including periodic calls driven by
      TIMEREVENT. These are interspersed with execution of Plan.behavior of all
      active plans. The main loop terminates due to uncaught exceptions in
      onEvent or a call to stop.

   3. onStop, called prior to full shutdown.

  """
  def __init__(self, confPath=None, robot=None, cfg={}):
    """
    Initialize a JoyApp application. This superclass constructor MUST be called
    by all subclasses.

    INPUTS: (all are optional)
      confPath -- str -- path to a YAML configuration file, start with $/ to
            search relative to the PYCKBOTPATH directory
      robot -- dict -- parameters for populating the robot interface;
            passed to ckbot.logical.Cluster when starting up;
            the resulting Cluster will be in self.robot
            Some useful parameters include arch= to set bus hardware
            and count= to set number of modules to populate with.
      cfg -- dict -- new/overridden configuration parameters. These override the
            configuration found in JoyApp.yml and confPath (if present). The
            configuration will be in a JoyAppConfig instance in self.cfg

    Configuration parameters are searched for in the following order
      1. JoyApp constructor cfg keyword argument
      2. YAML file specified by JoyApp constructor confPath argument
      3. YAML file named JoyApp.yml in the current directory

    ATTRIBUTES:
      DEBUG -- list of str -- list of debug topics to monitor, by default shares
            the joy module DEBUG topics..
      cfg -- JoyAppConfig -- configuration settings.
      robot -- ckbot.logical.Cluster -- interface to robot
      remote -- interface to event factory
      now -- float - current time; updated every timer event
      plans -- list of Plan-s -- (private) list of active Plan co-routines
      safety -- list of safety condition testers
    """
    self.DEBUG = DEBUG
    self.cfg = None #: EpyDoc (is a) dummy
    self.__initConf(confPath,cfg)
    if not self.cfg.logFile:
      self.logger = None
      progress('Logging disabled')
    else:
      self.logger = LogWriter(self.cfg.logFile)
      progress('Logging to "%s"' % self.cfg.logFile)
      if self.cfg.logProgress:
        progress('Progress messages will be logged')
        PROGRESS_LOG.add(self.logger)
    if not self.cfg.logVideo:
        self.frNmGen = None
    else:
        progress('Saving video frames to sequence %s' % (self.cfg.logVideo % 0))
        def _gen():
            n = 1
            while True:
                yield self.cfg.logVideo % n
                n += 1
        self.frNmGen = _gen()
    #
    self.now = pygix.now()
    # initialize safety provider list
    self.safety = []
    # init CKBot cluster interface
    if robot is not None:
      self._initRobot(robot)
    else:
      progress("No robot")
      self.robot = None #: Interface to CKBot robot modules
      'Interface to CKBot robot modules'
    # initialize midi module (this is safe to call again multiple times)
    if self.cfg.midi:
      progress("Requesting MIDI support")
      midi_init()
    else:
      progress("MIDI support was not requested")
    # init Plan scheduling structures
    self.plans = [] #: (private) List of currently active Plan instances ("threads")
    self.__new = []

  def __initConf(self, confPath, cfg):
    """
    (private) initialize the .cfg instance variable
    """
    # Default configuration
    self.cfg = JoyAppConfig(
      robotPollRate = 0.1,
      positionTolerance = 50,
      keyboardRepeatDelay = 500,
      keyboardRepeatInterval = 50,
      clockInterval = 20,
      voltagePollRate = 1.0,
      minimalVoltage = 13,
      windowSize = [320,240],
      nodeNames = {},
      logFile = None,
      logProgress = False,
      remote = None,
      midi = None,
      logVideo = None,
    )
    pth = PYCKBOTPATH + 'cfg%sJoyApp.yml' % OS_SEP
    if glob(pth):
      self.cfg.load(pth)
      progress("Loaded %s" % pth)
    else:
      progress("Path not found '%s' (this error can be safely ignored)" % pth )
    if confPath is not None:
      if confPath.startswith("$/"):
        confPath =  PYCKBOTPATH + confPath[2:]
      if glob(confPath):
        self.cfg.load(confPath)
        progress("Loaded %s" % confPath)
      else:
        progress("Nothing to load at confPath='%s'" % confPath)
    self.cfg.update(cfg)

  def _initRobot( self, robot ):
    """
    (protected) Initialize the ckbot.Cluster robot interface

    INPUT:
      robot -- dict -- Dictionary of settings for robot.populate
    """
    progress("Populating:")
    # Collect names from the cfg
    nn = self.cfg.nodeNames.copy()
    nn.update(robot.get('names',{}))
    robot['names']=nn
    # Show the names in the progress log
    for k,v in robot.items():
      progress("\t%s=%s" % (k,repr(v)))
    # Check for both protocol= and arch= parameters
    p = robot.get('protocol',None)
    a = robot.get('arch',None)
    if a and p:
      raise ValueError("Do not combine legacy protocol= with arch=")
    # NOTE: A protocol instance can be passed in arch= parameter of Cluster
    if p:
      del robot['protocol']
      a = p
    # Make sure that robot dictionary does not duplicate the arch= parameter
    if a:
      del robot['arch']
    self.robot = Cluster(arch=a, **robot)
    if self.cfg.robotPollRate:
      self._initPosPolling()
    if self.cfg.minimalVoltage:
      return self._initVoltageSafety()

  def _initPosPolling(self):
    """(protected)
    Set up servos for position polling
    """
    self.__pollPos = []
    for m in self.robot.itermodules():
      # Collect modules that have a voltage sensing capability
      if not isinstance(m,AbstractServoModule):
        continue
      p = m.get_pos_async()
      if p is None:
        continue
      m.__promise = p
      m.__promise_time = self.now
      m.__last_pos = 0
      self.__pollPos.append(m)


  def _initVoltageSafety( self ):
    """(protected)
    collect all robot modules that have a get_voltage() method
    and set them up for polling position
    """
    vs = []
    for m in self.robot.itermodules():
      # Collect modules that have a voltage sensing capability
      if hasattr(m,'get_voltage'):
        vs.append(m)
      if not isinstance(m,AbstractServoModule):
        continue
    # If any voltage sensing modules found, and sensing was requested
    #   then add the safety provider
    if not vs:
      return
    self.safety.append(safety.BatteryVoltage(
      vmin = self.cfg.minimalVoltage,
      sensors = vs,
      pollRate = self.cfg.voltagePollRate
    ))

  def setterOf( self, func ):
    """
    Convert a setter specification (which may be a string) to a setter function.
    Specifications include Cluster properties as supported by the
    ckbot.logical.Cluster class, single parameter callables, Scratch
    variables (indicated by pre-pending a ">"), debug messages (indicated
    by pre-pending a "#"), or any valid specification with logging (indicated
    by an additional "^" prefix)

    If specification is a tuple, its second member must be a function that
    pre-processes the values, e.g.:
        ('#x',lambda x : x/2.0)
    specified output to debug variable 'x', after scaling down by two.
        '^Nx22/@set_pos'
    would set the position on NID 0x22, and also log the set operation using
    the JoyApp's logger.
    If the tuple is longer than 2, entries 3 and up are used as the initial
    parameters, with the value coming in last.
    """
    if type(func)==tuple:
      pre = func[1]
      if not callable(pre):
        raise ValueError("Second member of tuple must be callable")
      fun = self.setterOf(func[0])
      func = func[2:]
      if func:
        def preProcFun( value ):
          return fun( func + (pre( value ),) )
      else:
        def preProcFun( value ):
          return fun( pre( value ) )
      func = preProcFun
    elif type(func)==str:
      if func[:1]=="#":
        # --> obtain customized message emitter
        pfx = func
        def emitMsg( val ):
          progress( "%s %s" % (pfx, repr(val) ))
        func = emitMsg
      elif func[:1]=="^":
        if func[1]=="^":
          raise ValueError("Only one '^' allowed in spec '%s'" % func)
        sf = self.robot.setterOf(func[1:])
        if self.logger is not None:
          func = self.logger.setterWrapperFor( sf, attr=dict(binding=func[1:]) )
        else: # no logger --> emit as progress message and set via cluster
          fmt = func[1:] + " = %s"
          def emitMsgAndSet( val ):
            progress( fmt % repr(val) )
            return sf(val)
          func = emitMsgAndSet
      else: # else --> obtain setter from cluster
        func = self.robot.setterOf(func)
    return func

  def _posEventPump( self ):
    """(private)
    <<NOTE: feature is currently disabled in _timeslice due to timing problems
      caused by robot.getLive(). This may be solvable by replacing the call to
      getLive() by something less disruptive>>

    Check asynchronous position polling of robot modules for new updates and
    push the necessary custom events into pygame
    """
    self.robot.getLive()
    t = self.now
    for m in self.robot.itermodules():
      if (not hasattr(m,'_JoyApp__promise') or (t - m.__promise_time < self.cfg.robotPollRate )):
        continue
      if (m.__promise):
        pos = m.pna.async_parse('h',m.__promise)
        if abs(pos-m.__last_pos)>self.cfg.positionTolerance:
          evt = JoyEvent(CKBOTPOSITION, module = m.node_id, pos = pos )
          pygix.postEvent(evt)
        m.__last_pos = pos
      m.__promise = m.get_pos_async()
      m.__promise_time = t

  def _pollSafety( self ):
    """(private)
    Poll all safety conditions and shut down immediately if violated
    """
    try:
      for s in self.safety:
        s.poll(self.now)
    except AbstractProtocolError as ape:
      progress("WARNING: safety test encountered a communication error '%s'" % str(ape))
    except safety.SafetyError as se:
      if self.robot:
         self.robot.off()
      self.stop()
      progress("(say) DANGER: safety conditions violated -- shutting down")
      progress(str(se))
      raise

  def _timeslice( self, timerevt ):
    """(private)

    Timer events get propagated to all Plans and trigger execution
    of a timeslice.
    """
    self.now = pygix.now()
    self.plans.extend(self.__new)
    self.__new = []
    if 'P' in self.DEBUG: debugMsg(self, "plans=%s" % dbgId(self.plans) )
    # Push timer events to all active plans
    for p in self.plans:
      p.push(timerevt)
    # Execute plans
    nxt = []
    for p in self.plans:
      # If a plan is running --> let it run a step
      if p.isRunning():
        p.step()
      # If it is still running --> remember to run it next time
      if p.isRunning():
        nxt.append(p)
      elif 'P' in self.DEBUG: # else --> harvested (died)
        debugMsg(self, "harvested %s" % dbgId(p) )
    if 'P' in self.DEBUG:
      debugMsg(self, "add plans=%s" % repr(self.__new) )
    self.plans = nxt + self.__new
    self.__new = []

  def onceEvery( self, tau ):
    """
    Return a function that returns True only once every
    tau time units. Typically, this is used for limiting
    debug output to appearing periodically, for example:

    oncePer = self.onceEvery(1)
    while True:
      if oncePer():
        print "Hello world"
    """
    last = [self.now]
    def onceEvery():
      if self.now-last[0]>tau:
        last[0] = self.now
        return True
      return False
    return onceEvery

  def _startRemote( self ):
    """(private) start remote interface if configured"""
    if not self.cfg.remote:
      self.remote = None
      return
    try:
        cfg = dict(self.cfg.remote)
    except:
        cfg = {}
    self.remote = remote.Sink(self, **cfg )
    progress("Starting up a remote.Sink interface")
    self.remote.start()

  def run( self ):
    """
    Run the JoyApp.

    After initializing the application, calls onStart()
    and executes event loop (onEvent()) until .stop() is called
    or an caught exception causes abnormal termination.

    onStop() is called before the application is shut down.
    """
    self.screen = pygix.startup(self.cfg)
    # Obtain figure for plotting; None if running headless
    if self.cfg.windowSize is not None:
        w,h = self.cfg.windowSize
        self.fig = figure(666,figsize=(w/80,h/80),dpi=80)
    else:
        self.fig = None
    self._frame = None
    self.isRunning = lambda : True
    self._startRemote()
    try:
      # Call onStart subclass hook
      self.onStart()
      # ----------- STARTS: MAIN LOOP --------------
      while self.isRunning():
        # Check all safety check providers
        self._pollSafety()
        # Poll midi interfaces (if requested)
        if self.cfg.midi:
          for evt in midi_joyEventIter():
            pygix.postEvent(evt)
        # Update UI (if active)
        if self._frame:
            buf,sz = self._frame
            pygix.showMplFig(buf,sz,self.screen)
            self._frame = None
        # Update robot (if one connected)
        if self.robot:
            # Let communication library do housekeeping
            self.robot.update()
            # Poll for robot position changes (if enabled)
            self._posEventPump()
        # Loop and process all events in the queue
        for evt in pygix.iterEvents():
          # TIMEREVENT-s cause the execution timeslices
          if evt.type == TIMEREVENT:
            self._timeslice(evt)
          self.onEvent(evt)
      # ----------- ENDS: MAIN LOOP --------------
    except Exception as exc:
      if '!' in self.DEBUG:
          raise
      else:
          printExc()
    # App cleanup bugs
    try:
      self.onStop()
      if self.robot:
        self.robot.off()
      if self.logger:
        self.logger.close()
        if self.cfg.logProgress:
          PROGRESS_LOG.remove(self.logger)
    finally:
      pygix.shutdown()

  def animate(self):
      """
      Get image in self.fig for presenting next GUI update

      If no self.fig (headless system), call is silently ignored
      """
      if not self.fig:
          return
      if self.frNmGen is not None:
          fn = next(self.frNmGen)
          self.fig.savefig(fn)
      else: # video only when not saving frames
          self._frame = self.fig.canvas.print_to_buffer()

  def stop(self):
    """
    Stop the JoyApp
    """
    self.isRunning = lambda : False

  def _startPlan( self, plan ):
    """(private)

    Called by Plan.start() to notify JoyApp that plan should be started
    """
    if not plan in self.plans:
      self.__new.append(plan)

  def onStart(self):
    """(default)

    Override this to run initialization code after JoyApp object is initialized
    but before any plans run

    By default, prints a message
    """
    progress("%s --- starting up main loop" % str(self) )

  def onStop( self ):
    """(default)

    Override this to run cleanup code before shut down

    By default, prints a message
    """
    progress("%s --- shutting down" % str(self) )

  def remapToKey(self,evt):
    """(default)
    Remap other events into keypress events

    INPUT:
      evt -- event object -- the event that occurred

    OUTPUT: key code or None
      The key code must be one of the K_ constants, or None if the event
      should be ignored
    """
    return None

  def on_K_q(self,evt):
    """(default)
    Handler for K_q -- terminate JoyApp
    """
    return self.stop()

  def _doOnK(self,evt):
    """(private)

    Process event and look for on_K_* event handler
    Return True iff successful in handling

    The event handler is found based on the key code from a KEYDOWN event,
    or produced by .remapToKey(evt). In the latter case, if the number
    returned is unfamiliar as a key, looks for on_K_XXXX where XXXX is a
    0 padded 4 digit hex representation of the key value (%04X format)
    """
    event = None
    if evt.type == KEYDOWN:
      event = evt.key
    elif evt.type == QUIT:
      event = K_ESCAPE
    else:
      event = self.remapToKey(evt)
    # Look for named event handler for keys
    if event is None:
      return False
    # Escape always terminates; cannot be overriden by on_K_ESCAPE
    if event is K_ESCAPE:
      self.stop()
      return True
    # Search for on_K_* event handler
    dcr = describeKey(event)
    # Exotic keys get hex codes
    if dcr is None:
      dcr = "K_%04X" % event
    hndlr = getattr(self, "on_"+dcr,None)
    if self.logger:
      self.logger.write('handler',dcr=dcr,h=repr(hndlr))
    if hndlr is None:
      return False
    res = hndlr(evt)
    if type(res) is GeneratorType:
      pln = EventWrapperPlan(self,evt,res)
      pln.start()
      if self.logger:
        self.logger.write('wrapper',plan=pln,evt=evt,gen=res)
    return True

  def onTimer(self):
    """
    Called on timer events, i.e. every timeslice
    """
    pass

  def onEvent( self, evt ):
    """(default)

    Top-level event handler

    Override this method to install your event handling code.
    The default JoyApp will try to handle keys with on_K_*() methods, and
    will try to map all other events into keys using
    The default JoyApp displays all events in human readable form on screen.
    Hitting <escape> or closing the window will cause it to quit.
    """
    if evt.type == TIMEREVENT:
      return self.onTimer()
    if self.logger:
        self.logger.write('event',**describeEvt(evt,parseOnly=1))
    # Map any non-key events into key events using self.remapToKey
    if self._doOnK(evt):
      return
    # Unhandled key -- display it
    if self.fig and evt.type==KEYDOWN:
      # Display keypress in the GUI (if GUI exists)
      self.fig.clf()
      ax = self.fig.gca()
      ax.plot([-1,1,1,1,-1],[-1,-1,1,1,-1],'w-')
      ax.text(0,0,evt.unicode)
      self.animate()
    progress('Event '+describeEvt(evt))

def runFlatScript(loc):
  """
  Implements the ability to construct a JoyApp as a flat script.
  Typical usage is:
    >>> from joy import runFlatScript
    >>> runFlatScript(locals())

  This will copy over all on_* functions and onStart and onStop into the
  scope of a new JoyApp instance, and then run that instance.

  To configure the JoyApp, you can have ROBOT and/or CFG defined in the
  loc namespace. These are mapped into the robot= and cfg= keyword arguments
  of the JoyApp constructor.

  See demos/demo-flatscript.py for more information
  """
  rob = loc.get('ROBOT',None)
  cfg = loc.get('CFG',{})
  app = JoyApp(robot=rob,cfg=cfg)
  loc['app']=app
  if rob:
    loc['robot']=app.robot.at
  for k,v in loc.items():
    if k.startswith("on_") or k in {"onTimer","onStart","onStop"}:
      setattr(app,k,v)
  app.run()

def test():
  import sys

  cmds=dict(cfg={})
  args = list(sys.argv[1:]) # copy
  while args:
    arg = args.pop(0)
    if arg=='--with-robot' or arg=='-r':
      cmds.update(robot={})
    if arg=='--with-midi' or arg=='-m':
      cmds['cfg'].update(midi=True)
    elif arg=='--help' or arg=='-h':
      cmds = None
    elif arg=='--mod-count' or arg=='-c':
      N = int(args.pop(0))
      if not cmds.has_key('robot'): cmds.update(robot={})
      cmds['robot'].update(count=N)
    elif arg=='--logfile' or arg=='-L':
      cmds['cfg'].update(logFile=args.pop(0))
    elif arg=='--logall' or arg=='-A':
      cmds['cfg'].update(logProgress=True)
  if cmds is None:
    sys.stdout.write("""
Usage: %s [options]

  Default JoyApp -- show all events

  Options:
    --with-robot | -r
      Start robot interface

    --with-midi | -m
      Attempt to collect MIDI controller events

    --mod-count <number> | -c <number>
      Search for specified number of modules at startup

    --logfile <filename> | -L <filename>
      Log events to a logfile

    --logall | -A
      Log *EVERYTHING*
  """ % sys.argv[0])
    sys.exit(1)
  app = JoyApp(**cmds)
  app.run()
