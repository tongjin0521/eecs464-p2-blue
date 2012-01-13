import pygame.midi as pygm
import sys
from joy.events import MIDIEVENT, JoyEvent

DEV = None

class Channels(object):
  def __init__(self):
    self._keys = set()
    
  def update( self, dic ):
    self._keys.update(dic)
    self.__dict__.update(dic)    
        
class MidiInput( object ):
  BUF_SIZE = 1024
  
  def __init__(self,name,ID):
    self.mi = pygm.Input(ID,MidiInput.BUF_SIZE)
    self.name = name
    self.ID = ID
    self.at = Channels()
    assert pygm.get_device_info(ID)[2], "Must be an input device"

  def iterevents(self):
    while self.mi.poll():
      for evt in self.mi.read(MidiInput.BUF_SIZE):
        yield evt
    return

class KorgNanoKontrol( MidiInput ):
  N2C = {
    'rewind' : (176, 47),
    'play' : (176, 45),
    'forward' : (176, 48),
    'repeat' : (176, 49),
    'stop' : (176, 46),
    'record' : (176, 44),
    'sc1_btnU1' : (176, 23),
    'sc1_btnL1' : (176, 33),
    'sc1_slider1' : (176, 2),
    'sc1_knob1' : (176, 14),
    'sc1_btnU2' : (176, 24),
    'sc1_btnL2' : (176, 34),
    'sc1_slider2' : (176, 3),
    'sc1_knob2' : (176, 15),
    'sc1_btnU3' : (176, 25),
    'sc1_btnL3' : (176, 35),
    'sc1_slider3' : (176, 4),
    'sc1_knob3' : (176, 16),
    'sc1_btnU4' : (176, 26),
    'sc1_btnL4' : (176, 36),
    'sc1_slider4' : (176, 5),
    'sc1_knob4' : (176, 17),
    'sc1_btnU5' : (176, 27),
    'sc1_btnL5' : (176, 37),
    'sc1_slider5' : (176, 6),
    'sc1_knob5' : (176, 18),
    'sc1_btnU6' : (176, 28),
    'sc1_btnL6' : (176, 38),
    'sc1_slider6' : (176, 8),
    'sc1_knob6' : (176, 19),
    'sc1_btnU7' : (176, 29),
    'sc1_btnL7' : (176, 39),
    'sc1_slider7' : (176, 9),
    'sc1_knob7' : (176, 20),
    'sc1_btnU8' : (176, 30),
    'sc1_btnL8' : (176, 40),
    'sc1_slider8' : (176, 12),
    'sc1_knob8' : (176, 21),
    'sc1_btnU9' : (176, 31),
    'sc1_btnL9' : (176, 41),
    'sc1_slider9' : (176, 13),
    'sc1_knob9' : (176, 22),
    'sc2_btnU1' : (176, 67),
    'sc2_btnL1' : (176, 76),
    'sc2_slider1' : (176, 42),
    'sc2_knob1' : (176, 57),
    'sc2_btnU2' : (176, 68),
    'sc2_btnL2' : (176, 77),
    'sc2_slider2' : (176, 43),
    'sc2_knob2' : (176, 58),
    'sc2_btnU3' : (176, 69),
    'sc2_btnL3' : (176, 78),
    'sc2_slider3' : (176, 50),
    'sc2_knob3' : (176, 59),
    'sc2_btnU4' : (176, 70),
    'sc2_btnL4' : (176, 79),
    'sc2_slider4' : (176, 51),
    'sc2_knob4' : (176, 60),
    'sc2_btnU5' : (176, 71),
    'sc2_btnL5' : (176, 80),
    'sc2_slider5' : (176, 52),
    'sc2_knob5' : (176, 61),
    'sc2_btnU6' : (176, 72),
    'sc2_btnL6' : (176, 81),
    'sc2_slider6' : (176, 53),
    'sc2_knob6' : (176, 62),
    'sc2_btnU7' : (176, 73),
    'sc2_btnL7' : (176, 82),
    'sc2_slider7' : (176, 54),
    'sc2_knob7' : (176, 63),
    'sc2_btnU8' : (176, 74),
    'sc2_btnL8' : (176, 83),
    'sc2_slider8' : (176, 55),
    'sc2_knob8' : (176, 65),
    'sc2_btnU9' : (176, 75),
    'sc2_btnL9' : (176, 84),
    'sc2_slider9' : (176, 56),
    'sc2_knob9' : (176, 66),
    'sc3_btnU1' : (176, 107),
    'sc3_btnL1' : (176, 116),
    'sc3_slider1' : (176, 85),
    'sc3_knob1' : (176, 94),
    'sc3_btnU2' : (176, 108),
    'sc3_btnL2' : (176, 117),
    'sc3_slider2' : (176, 86),
    'sc3_knob2' : (176, 95),
    'sc3_btnU3' : (176, 109),
    'sc3_btnL3' : (176, 118),
    'sc3_slider3' : (176, 87),
    'sc3_knob3' : (176, 96),
    'sc3_btnU4' : (176, 110),
    'sc3_btnL4' : (176, 119),
    'sc3_slider4' : (176, 88),
    'sc3_knob4' : (176, 97),
    'sc3_btnU5' : (176, 111),
    'sc3_btnL5' : (176, 120),
    'sc3_slider5' : (176, 89),
    'sc3_knob5' : (176, 102),
    'sc3_btnU6' : (176, 112),
    'sc3_btnL6' : (176, 121),
    'sc3_slider6' : (176, 90),
    'sc3_knob6' : (176, 103),
    'sc3_btnU7' : (176, 113),
    'sc3_btnL7' : (176, 122),
    'sc3_slider7' : (176, 91),
    'sc3_knob7' : (176, 104),
    'sc3_btnU8' : (176, 114),
    'sc3_btnL8' : (176, 123),
    'sc3_slider8' : (176, 92),
    'sc3_knob8' : (176, 105),
    'sc3_btnU9' : (176, 115),
    'sc3_btnL9' : (176, 124),
    'sc3_slider9' : (176, 93),
    'sc3_knob9' : (176, 106),
    #'sc4_btnU1' : (176, 16),
    #'sc4_btnL1' : (176, 17),
    #'sc4_slider1' : (176, 7),
    #'sc4_knob1' : (176, 10),
    'sc4_btnU2' : (177, 16),
    'sc4_btnL2' : (177, 17),
    'sc4_slider2' : (177, 7),
    'sc4_knob2' : (177, 10),
    'sc4_btnU3' : (178, 16),
    'sc4_btnL3' : (178, 17),
    'sc4_slider3' : (178, 7),
    'sc4_knob3' : (178, 10),
    'sc4_btnU4' : (179, 16),
    'sc4_btnL4' : (179, 17),
    'sc4_slider4' : (179, 7),
    'sc4_knob4' : (179, 10),
    'sc4_btnU5' : (180, 16),
    'sc4_btnL5' : (180, 17),
    'sc4_slider5' : (180, 7),
    'sc4_knob5' : (180, 10),
    'sc4_btnU6' : (181, 16),
    'sc4_btnL6' : (181, 17),
    'sc4_slider6' : (181, 7),
    'sc4_knob6' : (181, 10),
    'sc4_btnU7' : (182, 16),
    'sc4_btnL7' : (182, 17),
    'sc4_slider7' : (182, 7),
    'sc4_knob7' : (182, 10),
    'sc4_btnU8' : (183, 16),
    'sc4_btnL8' : (183, 17),
    'sc4_slider8' : (183, 7),
    'sc4_knob8' : (183, 10),
    'sc4_btnU9' : (184, 16),
    'sc4_btnL9' : (184, 17),
    'sc4_slider9' : (184, 10),
    'sc4_knob9' : (184, 7),
  } # ENDS: N2C
  C2N = dict( [(x,y) for y,x in N2C.iteritems()] )

  def __init__(self,*argv,**kw):
    MidiInput.__init__(self,*argv,**kw)
  
  def getUpdate( self ):
    evt = {}
    for (a,b,c,d),t in self.iterevents():
      nm = self.C2N.get((a,b),None)
      if not nm:
        continue
      evt[nm] = c
    self.at.__dict__.update(evt)
    return evt

def joyEventIter():
  """(protected) generate all pending events as JoyApp events
  This is used internally by JoyApp's event pump to collect midi events
  """
  for k,dev in DEV.iteritems():
    evt = dev.getUpdate()
    for ch,val in evt.iteritems():
	yield JoyEvent( MIDIEVENT, dev=k, dial=ch, value=(val-64)/64.0 )
      
if 0: # function to help create an N2C dictionary 
  import time, sys
  def foo(f='rewind'):
    nm = [
      'rewind',
      'play',
      'forward', 
      'repeat',
      'stop',
      'record'
    ]
    for sc in xrange(1,5):
      for k in xrange(1,10):
        nm.append('sc%d_btnU%d' % (sc,k))
        nm.append('sc%d_btnL%d' % (sc,k))
        nm.append('sc%d_slider%d' % (sc,k))
        nm.append('sc%d_knob%d' % (sc,k))
    while nm and nm[0] != f:
      nm.pop(0)
    last = (None,None)
    for k in nm:
       print "'%s'" % k,
       sys.stdout.flush()
       while True:
           time.sleep(0.1)
           l = list(DEV[3].iterevents())
           if not l:
             continue
           if l[-1][0][:2] != last[:2]:
             break
       last = l[-1][0]  
       print ": (%d,%d)," % (last[0],last[1])
       sys.stdout.flush()
     
def init():
   """
   Initialize the pygame midi interface and enumerate the devices in
   the .DEV module variable
   """
   global DEV
   if DEV is not None:
     return
   DEV = {}
   pygm.init()
   for k in xrange(pygm.get_count()):
     _,nm,inp,_,_ = pygm.get_device_info(k)
     if not inp:
       continue
     cls = {
       'nanoKONTROL MIDI 1' : KorgNanoKontrol,
     }.get( nm, None )
     if cls:
       DEV[k] = cls( nm, k )
       
if __name__=="__main__":
  from time import sleep, time as now
  print "Running test"
  init()
  t0 = now()
  while True:
    for dev in DEV.itervalues():
      print "%6.2f "%(now()-t0),str( dev.getUpdate() ) 
    sleep(0.05)
    
  
