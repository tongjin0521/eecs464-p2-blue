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
# (c) Shai Revzen, U Penn, 2010
#
#  
#
"""
joy.events defines a few utility functions for handling pygame events in the JoyApp framework. 
Because joy extends pygame with several new event types, you should use joy.event
functions to handle tasks such as printing human readable event descriptions --
the pygame builtins won't know anything about the JoyApp specific event types.

New Event Types
---------------

TIMEREVENT -- timeslice event timer for Plan execution
CKBOTPOSITION -- a CKBot moved
SCRATCHUPDATE -- a Scratch variable was updated

Main Functions
--------------
describeEvt -- describe an event in a string
"""
from pygame.locals import *
import pygame

## Declare new event type numbers
# TIMEREVENT-s are generated regularly and used to drive Plan execution
TIMEREVENT = USEREVENT
# CKBOTPOSITION events indicate a change in the position of a robot module 
#   a "change" needs to be at least `positionTolerance` units, and position
#   changes are polled `robotPollRate` seconds apart. Both `positionTolerance`
#   and `robotPollRate` are configuration parameters and can be set via the
#   JoyApp.yml configuration file.
CKBOTPOSITION = USEREVENT+1
# SCRATCHUPDATE events indicate a change in a Scratch channel
SCRATCHUPDATE = USEREVENT+2
# MIDIEVENT-s indicate input from a MIDI device
MIDIEVENT = USEREVENT+3

_EVENT_STRUCTURE = {
    QUIT             : (),
    ACTIVEEVENT      : ('gain', 'state'),
    KEYDOWN          : ('unicode','key','mod'),
    KEYUP	         : ('key','mod'),
    MOUSEMOTION	     : ('pos','rel','buttons'),
    MOUSEBUTTONUP    : ('pos','button'),
    MOUSEBUTTONDOWN  : ('pos','button'),
    JOYAXISMOTION    : ('joy','axis','value'),
    JOYBALLMOTION    : ('joy','ball','rel'),
    JOYHATMOTION     : ('joy','hat','value'),
    JOYBUTTONUP      : ('joy','button'),
    JOYBUTTONDOWN    : ('joy','button'),
    VIDEORESIZE      : ('size','w','h'),
    VIDEOEXPOSE      : (),
    CKBOTPOSITION    : ('module','pos'),
    SCRATCHUPDATE    : ('scr','var','value'),
    TIMEREVENT       : (),
    MIDIEVENT        : ('dev','dial','value'),
  }
  
_JOY_EVENT_NAMES = {
    SCRATCHUPDATE : "ScratchUpdate",
    CKBOTPOSITION : "CKBotPosition",
    TIMEREVENT : "TimerEvent",
    MIDIEVENT : "MIDIEvent"
  }
  
def describeEvt( evt, parseOnly = False ):
  """
  Describe an event stored in a pygame EventType object.
  
  Returns a human readable string that consists of all fields in the 
  event object. These are printed in a format that makes it easy to 
  cut and paste them into code that pattern matches events, e.g.
  
  >>> evt = pygame.event.Event(pygame.locals.MOUSEMOTION,pos=(176, 140),rel=(0, 1),buttons=(0, 0, 0))
  >>> print describeEvt( pygame.event.Event( JOYBUTTONDOWN, joy=1, button=3 ))
  type==MOUSEMOTION pos==(176, 140) rel==(0, 1) buttons==(0, 0, 0)
  
  If parseOnly is set, returns a dictionary with the extracted fields:
  
  >>> print joy.events.describeEvt(evt,1)
  {'buttons': (0, 0, 0), 'type': 'MouseMotion', 'pos': (176, 140), 'rel': (0, 1)}
  """
  assert type(evt) is pygame.event.EventType
  plan = _EVENT_STRUCTURE[evt.type]
  if evt.type<USEREVENT:
    nm = pygame.event.event_name(evt.type)
  else:
    nm = _JOY_EVENT_NAMES.get(evt.type,None)
    if nm is None:
      nm = "EVENT_%02d" % evt.type
  if parseOnly:
    res = dict(type=nm, type_code=evt.type)
    for item in plan:
      res[item] = getattr(evt,item)
  else:
    detail = ["type==%s" % nm.upper()]
    if plan:
      for item in plan:
        detail.append("%s==%s" %(
          item, repr(getattr(evt,item)) ))
    res = " ".join(detail)
  return res

def JoyEvent( type_code=None, **kw ):
  """
  Wrapper for pygame.Event constructor, which understands the additional
  event types unique to JoyApp
  
  INPUT: 
    type_code -- integer -- the typecode for the event type
    
  OUTPUT:
    Event object, or ValueError object with error details, if failed
  """
  if type_code is not None:
    et = type_code
  else:
    if kw.has_key('type'):
      et = kw['type']
    else:
      return ValueError('Unknown event type -- pass a type_code parameter')
  atr = {}
  for nm in _EVENT_STRUCTURE[et]:
    if kw.has_key(nm):
      atr[nm] = kw[nm]
    else:
      return ValueError('Missing property "%s"' % nm)
  return pygame.event.Event( et, atr )

