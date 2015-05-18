import math
from time import sleep
import ckbot.logical as L
nm = {
   0x14 : 'L1',
   0X20 : 'B1',
   0x0C : 'F2',
   0x1A : 'L2',
   0x90 : 'B2',
   0x02 : 'F3',
   0x1E : 'L3'


#   0x16 : 'L1',
#   0X01 : 'B2',
#   0x09 : 'F3',
#   0x13 : 'L2',
#   0x14 : 'B1',
#   0x93 : 'F2',
#   0x20 : 'L3'

#  0x1E : 'L1',
#  0x90 : 'L1',
#  0x13 : 'B1',
#  0x01 : 'B1',
#  0x02 : 'F2',
#  0x09 : 'F2',
#  0x19 : 'L2',
#  0x18 : 'L2',
#  0x0c : 'B2',
#  0x1d : 'B2',
#  0x14 : 'B2',
#  0x02 : 'F3',
#  0x20 : 'F3',
#  0x92 : 'L3',
#  0x01 : 'L3',
#  0x14 : 'L3'
}
c = L.Cluster(names=nm, count=7)
MAX_BODY = 9000
MAX_SEG = 8000
Y_var = 1500
count = 400
phase = 0
r  = 0
try:
      
    for m in [c.at.B1,c.at.F2,c.at.B2,c.at.F3]:
      m.set_pos(0)
        
    while 1:
      
        #rotates body to +MAX_BODY
        if phase == 0:
          c.at.L2.go_slack()
          c.at.L1.set_pos(-r)
          c.at.L3.set_pos(r)
          r = r + count
          if r >= MAX_BODY:
            c.at.L1.go_slack()
            c.at.L3.go_slack()
            phase = 1
            r = 0
          
        #bends body to +MAX_SEG
        if phase == 1:
          c.at.B2.set_pos(-r)
          c.at.F3.set_pos(-r)
          c.at.B1.set_pos(r)
          c.at.F2.set_pos(r)
          r = r + count
          if r >= MAX_SEG:
            phase = 3
            r = MAX_BODY
      
        #rotates body to -MAX_BODY
        if phase == 3:
          c.at.L1.set_pos(-r)
          c.at.L2.go_slack()
          #c.at.L2.set_pos(MAX_BODY)
          #c.at.L3.go_slack()
          c.at.L3.set_pos(r)
          #c.at.B2.set_pos(r)
          #c.at.F3.set_pos(r)
          #c.at.B1.set_pos(-r)
          #c.at.F2.set_pos(-r)
          r = r -(count + 100)
          if r <= -(MAX_BODY + Y_var):
            phase = 4
            r = MAX_SEG

        #straghtens body to 0
        if phase == 4:
          c.at.B1.set_pos(r)
          c.at.F2.set_pos(r)
          c.at.F3.set_pos(-r)
          c.at.B2.set_pos(-r)
          r = r - count
          if r <= 0:
            phase = 0
            r = 0
        
        print phase
        print r
        sleep(0.020)
      
except KeyboardInterrupt:
  c.off()
  raise
