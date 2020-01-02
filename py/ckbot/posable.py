"""File posable.py

Provides basic functionality for "posable programming", i.e. programming a robot by recording its poses and then playing them back.

Main Classes and Functions
==========================

class PoseRecoder -- provides most of the functionality, allowin poses to be recorded and played back.

function recordFromCluster -- an interactive interface on top of a PoseRecorder, allowing users to easily record poses.
"""

from time import time as now, sleep
from sys import stdout as STDOUT
from os.path import dirname
from numpy import asarray
from cmd import Cmd
from re import split as re_split
from scipy.interpolate import interp1d
from .ckmodule import Module
# Dependencies for socket control in CLI
from socket import socket, AF_INET, SOCK_DGRAM, SOL_SOCKET, SO_REUSEADDR, error as SocketError
from errno import EAGAIN
from json import loads as json_loads, dumps as json_dumps
from subprocess import Popen

class PoseRecorder( object ):
    """Concrete class PoseRecorder

    Provides a convenient interface for 'poseable programming' by allowing
    positions of servos to be recorded and played back.

    Convenience methods for saving and loading .csv files are also provided

    For typical usage, see the source code for recordFromCluster()
    """
    def __init__(self,mods,upd=None):
        """Initialize the PoseRecorder to record a list of modules

        INPUT:
          mods -- sequence of modules with .get_pos(), .set_pos() and .go_slack() methods
          upd -- update(t) function to call as part of pose update
        """
        self.setServos(mods)
        self.record_time = None
        if callable(upd):
            self.upd_CB = upd
        else:
            self.upd_CB = lambda t : None
        self.reset()
        self.off()
        self.kind = 'linear'
        self.period = 1.
        self.count = 1

    def setServos(self,mods):
        for m in mods:
            assert isinstance(m,Module),"%s must be a module" % str(m)
        self.servos = mods

    def update(self,t=None):
        """
        Update function to serve modules which require that
        """
        if t is None:
            t = now()
        self.upd_CB(t)
        for m in self.servos:
            if not hasattr(m,"update"):
                continue
            m.update(t)

    def getPose(self):
        """collect positions of all servos"""
        return [ m.get_pos() for m in self.servos ]

    def setPose(self,pose):
        """set positions of all servos"""
        for m,v in zip(self.servos, pose):
           if v is None:
               m.go_slack()
           else:
               m.set_pos(v)

    def off(self):
        """Emegency Stop for all servos -- .go_slack()-s all modules, ignoring any exceptions"""
        for m in self.servos:
          try:
            m.go_slack()
          except Exception as ex:
            print("Exception in %s.go_slack():" % str(m), str(ex))

    def reset( self ):
        """Reset the recording, allowing a new recording to be started"""
        self.plan = []
        self.record_time = None

    def snap( self, t = None ):
        """Add a snapshot of the current pose to the recording"""
        if not self.plan:
          t = 0
        if t is None:
          t = self.plan[-1][0]+1
        self.plan.append( [t]+self.getPose())

    def snapClosed( self, dt = 1 ):
        """Add the first pose at the end of the recording, with specified delay.
        This allows loops to be cleanly repeated.

        INPUT:
          dt -- float -- delay in seconds between last and first pose
        """
        if not self.plan:
            raise IndexError("No recoding found; call .snap() at least once")
        self.plan.append( [self.plan[-1][0]+dt]+self.plan[0][1:] )

    def show( self, stream=STDOUT, delim=" ", fmt="%5d", hfmt="%5s", rdel="\n", slc=slice(None)):
        """Write the current recording out on a stream in a text-based format

        INPUT:
          stream -- output stream object. Must support .write()
          delim -- str -- delimiter user between columns
          fmt -- str -- format string for numbers (except time)
          hfmt -- str -- format string for column headings
          rdel -- str -- row delimiter
          slc -- slice -- range of pose sequence to show
        """
        stream.write( delim.join(
          ["%5s" % "t"]
          +[hfmt % m.name for m in list(self.servos)[slc]]
          ) + rdel )
        for pose in self.plan[slc]:
          stream.write( delim.join(
            ["%5d" % pose[0]]
            +[fmt % v for v in pose[1:]]
          ) + rdel )

    def unsnap(self):
        """Drop the last pose from the recording"""
        if self.plan:
            self.plan.pop(-1)

    def appendFrom( self, stream, cdel=",", dt=1  ):
        """append a recording from a textual representation

        INPUT:
          stream -- list -- a recording with one line per list entry
                 -- file -- a file containing a recording
          cdel -- str -- column delimiter
          dt -- float -- gap in time between last entry and newly loaded data

        Caveats:
          (1) columns must match the list of servos for which the
              PoseRecorder was configured.
          (2) the first (header) row of the file is ignored

        Example: load froma literal multiline string
        >>> pr.appendFrom('''
        0, 1000, 0, 1000
        1,    0, 1000, 0
        '''.split("\n"))
        """
        if type(stream) is not list:
            stream = stream.readlines()
        t0 = 0 if not self.plan else (dt+self.plan[-1][0])
        for nl,line in enumerate(stream[1:]):
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            val = [float(v.strip()) for v in line.split(cdel)]
            if len(val) != len(self.servos)+1:
                raise ValueError("%d: Found %d values instead of %d"
                   % (nl+1,len(val)-1,len(self.servos)))
            val[0] += t0
            self.plan.append(val)

    def loadCSV( self, stream ):
        """Syntactic sugar for loading a .csv file using .appendFrom()"""
        self.reset()
        self.appendFrom(stream)

    def saveCSV( self, stream=STDOUT ):
        """Syntactic sugar for using .show() to write a .csv formatted recording

        INPUT:
          stream -- a file object or a file path. The '.csv' will be added if missing

        Typical usage:
          pr.saveCSV('motion.csv')
        """
        if type(stream) is str:
            if not stream.endswith(".csv"):
                stream = stream + ".csv"
            stream = open(stream, "w")
        self.show(stream, delim=", ", fmt="%d", hfmt='"%s"')
        if stream is not STDOUT:
          stream.close()

    def playback( self, period=None, count=None, rate = 0.03, urate = 0.01, kind=None ):
        """Play back the current recording one or more times.

           INPUT:
             period -- float / None -- duration for entire recording
               if period is None, the recording timestamps are used
             count -- integer -- number of times to play
             rate -- float -- delay between commands sent (sec)
             urate -- float -- delay between .update calls
             kind -- str -- a scipy.interpolation.interp1d kind
            """
        assert rate>=urate
        # playback current pose for a given amount of time and with a given period
        if not self.plan or len(self.plan)<2:
          raise ValueError("No recording -- .snap() poses first!")
          return
        gait = asarray(self.plan,int)
        if kind is None:
            kind = self.kind
        gaitfun = interp1d( gait[:,0], gait[:,1:].T, kind=kind ) # Gait interpolater function
        dur = gait[-1,0]-gait[0,0]
        if period is None:
            period = self.plan[-1][0] - self.plan[0][0]
        if count is None:
            count = self.count
        t0 = now()
        t1 = t0
        try:
          while t1-t0 < period*count:
            # Do inner update loop
            t1 = now()
            while now()<t1+rate-urate:
                self.update(t1)
                sleep(urate)
            # send position command
            phi = (t1-t0)/period
            phi %= 1.0
            goal = gaitfun(phi*dur).round()
            print("\rphi: %.2f: " % phi, " ".join([
                "%6d" % g for g in goal
            ]))
            self.setPose( goal )
            # Adjust to match requested rate
            dt = rate-(now()-t1)
            if dt>0:
                sleep(dt)
        except KeyboardInterrupt:
            self.off()
            raise

class LiveJoyRemote( object ):
    """Concrete class LiveJoyRemote is asupport class for
    the do_live command in the CLI. This command is special in that
    it sets up a UDP socket for receiving events from a subprocess
    that it spawns; this subprocess is a demo_remoteSource JoyApp
    which pumps MIDI events back.
    """
    def __init__( self, pr ):
        self.sock = socket( AF_INET, SOCK_DGRAM )
        self.sock.setsockopt( SOL_SOCKET, SO_REUSEADDR, 1  )
        self.sock.bind(("localhost",0))
        self.sock.setblocking(False)
        self.pr = pr

    def _recv(self):
        """(private)
        Receive an event
        """
        try:
            # Create event from the next packet
            pkt = self.sock.recv(1024)
            dic = json_loads(pkt)
            #
        except SocketError as err:
            # If socket is out of data --> we're done
            if err.errno == EAGAIN:
              return None
            # otherwise --> was a real error, raise again
            raise
        except ValueError as ve:
            # Value errors come from JSON decoding problems.
            # --> Log and continue
            print('ERROR: received bad UDP packet: %s' % repr(pkt))
            return None
        return dic

    def show(self):
        """Show current plan"""
        print("#### Recording")
        print("# duration ",self.pr.duration," count ",self.pr.count)
        self.pr.show()
        print("#### END")

    def run(self):
        # Obtain current motor states
        self.pose = list(self.pr.getPose())
        # Create JoyApp event source
        pth = dirname(__file__)+"/../../demos"
        cmd = "PYGIXSCHD=pygame python %s/demo-remoteSource.py -p %d -e MIDIEVENT -e KEYDOWN" % (pth,self.sock.getsockname()[1])
        print("\trunning: %s" % cmd)
        joy = Popen(cmd,shell=True,stdout=open('/dev/null','w'))
        # While subprocess is running
        while joy.poll() is None:
            # Allow PoseRecorder to do housekeeping
            self.pr.update()
            # Check for incoming message
            evt = self._recv()
            if evt is None:
              sleep(0.05)
              continue
            ### We got an event
            # Find its handler
            hnd = '_on%s' % evt.get('type','<<-NONE->>')
            # Find handler method
            meth = getattr( self, hnd, self._onUnknownType )
            # call handler
            if meth(**evt):
                break
            self.pr.setPose(self.pose)
            msg = ", ".join([('---' if p is None else "%6d" % int(p)) for p in self.pose])
            STDOUT.write("\r"+msg+"\r")
            STDOUT.flush()

        # terminate subprocess if it is still running
        if joy.returncode is None:
            joy.terminate()
        STDOUT.write("\n...Live mode ending\n")

    def _onUnknownType(self,**kw):
        print("[ERR] Unknown event ",repr(kw))

    def _onMIDIEvent(self,index=None,sc=None,kind=None,value=None,**kw):
        ### We got an event
        # Find its handler
        hnd = '_onMIDI%s' % kind
        # Find handler method
        meth = getattr( self, hnd, None )
        # call handler
        if meth is None:
            return self._onUnknownType( type="MIDIEvent", index=index, sc=sc, kind=kind, value=value, **kw)
        else:
            return meth(index=index,value=value)

    def _onMIDIslider(self,index=None,value=None,**kw):
        idx = index % len(self.pose)
        self.pose[idx] = (value-63.5)*10000/63.5

    def _onMIDIrecord(self,value=None,**kw):
        """Append the current pose to the recording"""
        if value!=127:
            return
        self.pr.snap()
        self.show()

    def _onMIDIrewind(self,value=None,**kw):
        """Delete last step / Reset the recording if none active"""
        if value!=127:
            return
        print("Removing last pose; first hit stop if you want to reset recording")
        self.pr.unsnap()
        self.show()

    def _onMIDIstop(self,value=None,**kw):
        """Emergency stop the recording -- go_slack() all modules"""
        self.pr.off()
        print(">>>> STOP <<<<")

    def _onMIDIrepeat(self,value=None,**kw):
        """Close a loop in the recording
        """
        if value!=127:
            return
        print("**** Closing loop")
        self.pr.snapClosed(1.0)
        self.show()

    def _onMIDIplay(self,value=None,**kw):
        if value != 127:
            return
        try:
            self.pr.playback()
        except KeyboardInterrupt:
            self.pr.off()
        print("Playback done")

    def _onKeyDown(self,key=None,unicode=None,**kw):
        if unicode==" ":
            print("Record pose")
            self.pr.snap()
            self.show()
            return
        idx = "wertyuio".find(unicode)
        if idx>=0:
          self.pose[idx % len(self.pose)] += 1000
          return
        idx = "sdfghjkl".find(unicode)
        if idx>=0:
          self.pose[idx % len(self.pose)] -= 1000
          return
        print("[KEY] ",repr(unicode))

class PoseRecorderCLI( Cmd ):
    """Concrete class PoseRecorderCLI provides a cmd.Cmd based commandline
    interface to a PoseRecorder.

    Start one and give the 'help' command to get more information
    """
    def __init__(self,pr,clust=None):
        """
        INPUTS:
          pr -- PoseRecorder -- PoseRecorder to control
          clust -- Cluster / None -- Cluster; to allow the general
            command interface
        """
        Cmd.__init__(self)
        self.prompt = "PoseRecorder >> "
        assert isinstance(pr,PoseRecorder)
        self.pr = pr
        self.liv = LiveJoyRemote(pr)
        self.pr.duration = None
        self.pr.count = 1
        self.updT = 0.2
        self.pr.kind = 'linear'
        self.clust = clust
        if clust is None:
          self.tgt = []
        else:
          self.tgt = clust.values()
          pr.setServos(self.tgt)

    def run(self):
        self.cmdloop()

    def emptyline(self):
        Cmd.emptyline(self)
        self.do_show()

    def _update(self):
        t0 = now()
        while now()-t0<self.updT:
            self.pr.update(now())
            sleep(0.05)

    def do_setKind(self,line):
        knds = ['linear', 'nearest', 'zero', 'slinear', 'quadratic', 'cubic']
        for k in knds:
            if k.startswith(line.lower()):
                self.pr.kind = k
                print("Interpolation kind set to '%s'" % k)
                return
        print("Unknown interpolation kind '%s'" % line)
        print("\tSupported:",",".join(knds))

    def do_target(self,line):
        """Specify target for commands

        Target may be either * or a space-separate list of module names
        """
        if self.clust is None:
          print("No Cluster specified; 'target' command is not available")
          return
        line = line.strip()
        if line == "*":
          self.tgt = list(self.clust.values())
        else:
          try:
            self.tgt = [getattr(self.clust.at,nm) for nm in line.split(" ")]
          except AttributeError as ae:
            print("Unknown module:",ae)
            return
        self.pr.setServos(self.tgt)
        self._update()

    def do_live(self,line):
        """Run a JoyApp interface allowing KORG midi control of recordings"""
        self._update()
        self.liv.run()
        self._update()

    def do_update(self,line):
        """Run protocol & module updates for specified time (sec)"""
        if not line:
            line = "5"
        try:
            T = float(line)
        except ValueError:
            print("Specify a time in seconds")
            return
        t0 = now()
        while now()-t0<T:
            self._update()
            STDOUT.write(".")
            STDOUT.flush()
        STDOUT.write("DONE\n")

    def do_cmd(self,line):
        """Specify command to broadcast to targets"""
        if self.clust is None:
          print("No Cluster specified; 'cmd' command is not available")
          return
        self._update()
        sp = re_split("\s+",line,1)
        if len(sp)>1:
          cmd,val = sp[0],int(sp[1])
        else:
          cmd,val = sp[0],None
        #
        res = []
        for m in self.tgt:
          try:
            if val is None:
              res.append(getattr(m,cmd)())
            else:
              res.append(getattr(m,cmd)(val))
          except AttributeError as ae:
            print("Module",m.name,"does not support '%s'" % cmd)
            res.append(None)
        #
        print(" ".join([
            "%6s" % m.name for m in self.clust.values()]))
        print(" ".join([
            "%6s" % str(v) for v in res]))

    def do_show(self,line=None):
        """Show the current recording in text form"""
        print("# duration ",self.pr.duration," count ",self.pr.count)
        self.pr.show()

    def do_pose(self,line=None):
        """Append the current pose to the recording"""
        self._update()
        self.pr.snap()

    def do_reset(self,line=None):
        """Reset the recording"""
        self.pr.reset()

    def do_off(self,line=None):
        """Emergency stop the recording -- go_slack() all modules"""
        self.pr.off()
        self._update()

    def do_closeLoop(self,line):
        """closeLoop <dt> -- Close a loop in the recording, with delay <dt>
        """
        try:
            f = float(line)
        except ValueError:
            f = 1
        self.pr.snapClosed(f)

    def do_drop(self,line):
        """Drop the last pose from the recording"""
        self.pr.unsnap()

    def do_save(self,line):
        """save<file> Save recording to a .csv file"""
        try:
            self.pr.saveCSV(line)
        except IOError as ioe:
            print(ioe)

    def do_load(self,line):
        """Append recording from a .csv file
           load <file>
        """
        try:
            stream = open(line,"r").readlines()
        except IOError as ioe:
            print(ioe)
            return
        self.pr.appendFrom(stream)

    def do_count(self,line):
         """Set number of times to loop"""
         try:
            self.pr.count = int(line)
         except ValueError as ve:
            print(ve)

    def do_duration(self,line):
         """Set duration of one playback period (empty for automatic)"""
         if not line:
             self.pr.duration = None
         else:
             try:
                 self.pr.duration = float(line)
             except ValueError as ve:
                 print(ve)

    def do_setUpdateTime(self,line):
        """Set the update time (sec) used for update functions
        """
        try:
            val = float(line)
            if val<0.001:
                val = 0.001
            self.updT = val
        except ValueError:
            print("Could not parse '%s' as a valid duration")

    def do_run(self,line=None):
         """Run the recording. Set duration and cycle count with 'duration' and 'count' commands"""
         try:
             self._update()
             self.pr.playback()
             self._update()
         except KeyboardInterrupt:
             self.pr.off()

    def do_inline(self,line=None):
         "Show inline code for recording"
         STDOUT.write(".appendFrom('''")
         self.pr.show(delim=",",rdel="; ")
         STDOUT.write("'''.split(';'))\n")

    def do_EOF(self,line=None):
        return self.do_quit()

    def do_quit(self,line=None):
        "quit"
        return True

    def do_exit(self,line=None):
        "quit"
        return True

def recordFromCluster( c ):
    """Interactively prompt user and record poses from all modules of a cluster
    INPUT:
      c -- Cluster to record from
    OUTPUT: PoseRecorder with the recording
    """
    p = PoseRecorder(list(c.values()))
    while True:
        p.reset()
        while True:
            if input("<Enter> to store <q><Enter> when done: ") is 'q':
                break
            p.snap()
            p.show()
        if input("Have successfuly recorded? (y/n): ") is 'y':
            break
    return p
