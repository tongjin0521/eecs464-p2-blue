import serial as S
import sys

s = S.Serial('/dev/ttyUSB0',baudrate=115200)
if len(sys.argv)>1:
  s.timeout = float(sys.argv[1])
else:
  s.timeout = 0.05

while 1:
  # Broadcast a ping
  s.write('\xff\xff\xfe\x02\x01\xfe')
  # get reply from up to 100 modules
  recv = s.read(6*100)
  l = recv.split("\xff\xff")
  for msg in l:
    if msg:
      print "0x%02x" % ord(msg[0]),
  print " # ",len(l)-1

