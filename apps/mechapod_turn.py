import math
from time import sleep
import ckbot.logical as L
nm = {
    0x1E : 'F',
    0x20 : 'S1',
    0x0C : 'S2',
    0x19 : 'M',
    0x1C : 'S3',
    0x08 : 'S4',
    0x14 : 'H'
    }
PORT = dict(TYPE='TTY', glob='/dev/ttyUSB*', baudrate=1000000)
c = L.Cluster(names=nm, count=7, port=PORT)
Max_roll = 4000
Max_yaw = 2000
phase = 0
r = 0
r0 = 400
y0 = 200
y = 0
tr = 2000
be0 = 100
be = 0
be_max= 0
Dr = 0

try:
    for m in [c.at.F,c.at.S1,c.at.S2,c.at.M,c.at.S3,c.at.S4,c.at.H]:
        m.set_pos(0)
    
    sleep(2)

    while be < be_max:
        c.at.S2.set_pos(be)
        c.at.S3.set_pos(-be)
        be += be0
        sleep(0.02)

    while y > -Max_yaw:
        c.at.F.set_pos(y+tr)
        c.at.M.set_pos(-y+tr)
        c.at.H.set_pos(-y-tr)
        y -= y0
        sleep(0.020)


    
    while 1:
        if phase == 0:
            c.at.F.set_pos(y+tr)
            c.at.M.set_pos(-y+tr)
            c.at.H.set_pos(-y-tr)
            c.at.S1.set_pos(r+Dr)
            c.at.S4.set_pos(r+Dr)
            r += r0
            y += y0
            if r >= Max_roll:
                phase = 1

        if phase == 1:
            c.at.F.set_pos(y+tr)
            c.at.M.set_pos(-y+tr)
            c.at.H.set_pos(-y-tr)
            c.at.S1.set_pos(r+Dr)
            c.at.S4.set_pos(r+Dr)
            r -= r0
            y += y0
            if r <= 0:
                phase = 2

        if phase == 2:
            c.at.F.set_pos(y+tr)
            c.at.M.set_pos(-y+tr)
            c.at.H.set_pos(-y-tr)
            c.at.S1.set_pos(r+Dr)
            c.at.S4.set_pos(r+Dr)
            r -= r0
            y -= y0
            if r <= -Max_roll:
                phase = 3

        if phase == 3:
            c.at.F.set_pos(y+tr)
            c.at.M.set_pos(-y+tr)
            c.at.H.set_pos(-y-tr)
            c.at.S1.set_pos(r+Dr)
            c.at.S4.set_pos(r+Dr)
            r += r0
            y -= y0
            if r >= 0:
                phase = 0
        sleep(0.020)

    


except KeyboardInterrupt:
    c.off()
    raise
