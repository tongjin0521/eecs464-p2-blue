from pylab import figure, gcf, plot, axis, text, find, draw, clf
from socket import socket, AF_INET, SOCK_DGRAM, error as SocketError
from numpy import asarray, array, mean
from json import loads as json_loads

if __name__ != "__main__":
  raise RuntimeError("Run this as a script")

try:
  s.close()
except Exception:
  pass

s = socket(AF_INET, SOCK_DGRAM )
s.bind(("",0xB00))
s.setblocking(0)

lst = []
msg = None
while True:
  try:
    # read data as fast as possible
    m = s.recv(1<<16)
    if m and len(m)>2:
      msg = m
    continue
  except SocketError, se:
    # until we've run out; last message remains in m
    pass
  # make sure we got something
  if not msg:
    continue
  # display it
  clf()
  for d in json_loads(msg):
    a = array(d['p'])
    plot( a[[0,1,2,3,0],0], a[[0,1,2,3,0],1], '.-b' )
    plot( a[[0],0], a[[0],1], 'og' )
    text( mean(a[:,0]), mean(a[:,1]), d['i'], ha='center',va='center' )
    axis([0,1024,0,768])
  draw()
  
