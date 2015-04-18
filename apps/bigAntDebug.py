from ckbot import logical as L
import time


c = L.Cluster(port=dict(TYPE='tty', glob="/dev/ttyUSB*", baudrate=115200))

c.populate(6)

for i in c.itermodules():
  i.go_slack()

t = time.time()

#for m in c.itermodules():
#   m.set_torque(0.1)
time.sleep(2.0)

while 0: #time.time() - t < 10:
   for m in c.itermodules():
      if (m.get_pos() > 5000):
         time.sleep(0.1)
         m.set_torque(-0.1)
      if (m.get_pos() < -5000):
         time.sleep(0.1)
         m.set_torque(0.1)
      time.sleep(0.1)
   print str(time.time() - t)


while time.time() - t < 10:
 dat = {}
 
 for i in c.itermodules():
  dat[i.name] = (i.get_pos())

 time.sleep(0.5) 
 print str(time.time() - t)

 for keys,value in dat.iteritems():
   if (value > 5000):
    getattr(c.at, keys).set_torque(-0.1)
   if (value < -5000):
    getattr(c.at, keys).set_torque(0.1)

 time.sleep(0.2)


 
for m in c.itermodules():
   m.go_slack()
