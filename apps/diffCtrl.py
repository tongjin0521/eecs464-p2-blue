#!/usr/bin/env
# I can't seem to get remoteSource to do what I want ... this is worrisome
from socket import socket, AF_INET, SOCK_DGRAM
import pygame
from pygame.locals import *
from joy.events import describeEvt
from json import dumps as json_dumps
from time import time as now

if __name__=="__main__":
  print """
	Send pygame keyboard events over UDP
	"""    
  s = socket( AF_INET, SOCK_DGRAM )
  dst = ('172.16.16.135', 31313)
  t0 = now()
  
  pygame.init()
  disp = pygame.display.set_mode( (400, 400 ) )

  while True:
    evts = pygame.event.get()
    for evt in evts:
      if evt.type in [MOUSEMOTION]:
        continue
      dic = describeEvt( evt, parseOnly = True )
      dic['t'] = now()-t0
      jsn = json_dumps( dic, ensure_ascii = True )
      s.sendto( jsn, dst )
      print "Sending %s to %s:%d" % ( (jsn,)+dst )
    pygame.display.flip()
  
  
