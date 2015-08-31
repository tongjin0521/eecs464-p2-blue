from time import sleep
import ckbot.logical as L
nm = {
  0x08 : 'L1',
  0x90 : 'L1',
  0x13 : 'B1',
#  0x01 : 'B1',
  0x02 : 'F2',
  0x09 : 'F2',
  0x19 : 'L2',
  0x18 : 'L2',
  0x1d : 'B2',
  0x14 : 'B2',
#  0x02 : 'F3',
  0x20 : 'F3',
  0x92 : 'L3',
  0x01 : 'L3'
}
c = L.Cluster(names=nm, count=7) 
try:
  while 1:
    for m in [c.at.B1,c.at.F2,c.at.B2,c.at.F3]:
      m.set_pos(0)
    for k in xrange(63):
      s = sin(k/10.0)
      r = int(sqrt(abs(s))*sign(s)*4000)
      print r
      c.at.L1.set_pos(-r)
      c.at.L2.set_pos(-r)
      c.at.L3.set_pos(r)
      sleep(0.015)
except KeyboardInterrupt:
  c.off()
  raise
