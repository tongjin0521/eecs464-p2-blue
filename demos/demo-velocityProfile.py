#stella's velocity demo code!

if __name__!="__main__":
  import sys
  sys.exit()

import ckbot.logical #need to make sure modules can populate and the pcan can recognise the node ID
import joy  #used to call the feedbackrate function
import time #used for all of the time related functions
import numpy
#from pylab import plot, show, xlim, xlabel, ylabel, title, grid, savefig
import pylab   
c = ckbot.logical.Cluster() #program recognises how many and which modules are attached
  
c.populate() # node IDs of modules are stored
  
c.at.Nx05.get_od() #getting object dictionary
  
c.at.Nx05.od.set_feedbackrate(100) #will get feedback from modules at a rate of 100 Hz
  
c.at.Nx05.reset() # stores feedback rate in eeprom memory. only have to do once then can delete from code.

time.sleep(5) #has code wait for 5 seconds. for more information type 'help()' then time in the ipython console.

c.at.Nx05.set_pos(-5000) 

time.sleep(3)

T0 = time.time() # sets a variable to be equal to the actual time since the start of the epoch.
    
c.at.Nx05.set_pos(5000)

period = 3

pos = [] #creates an empty position list
t = []
start_time = time.time() #sets a new variable to a later time 

while(T0 - start_time < period):    
    val = str(c.at.Nx05.get_pos()/100)
    pos.append(val)    #first appends the position value, then right after it the corresponding time.
    t.append(time.time() - start_time) #akes the 
    T0 = time.time() #re defines T0 as a later time, which eventually will end the while loop when T0-start_time becomes >= 3 seconds (the period)

#print time_list, list

pylab.plot(t[:31], pos[:31])

pylab.xlabel('time (s)')
pylab.ylabel('position (deg)')
pylab.title('Position Plot')
pylab.grid(True)

pylab.savefig('position_plot')

pylab.show() #so we can graph the position and time values


vel = []

for i in range(len(pos)-1):
    vel.append((pos[i+1]-pos[i])/(t[i+1]-t[i]))
 
pylab.plot(t[:31], vel[:31])

pylab.xlabel('time (s)')
pylab.ylabel('velocity (deg/s)')
pylab.title('Velocity Plot')
pylab.grid(True)

pylab.savefig('velocity_plot')

pylab.show() #so we can graph the position and time values 
