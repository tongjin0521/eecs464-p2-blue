"""Run the robot, collect and log trial data
Feature add list:
-Save data runs as gzipped csv
-Can run the robot automatically and sense when it goes out of bound. Use tail to grab end of the file
-OPTIONAL: can view position in 2D or 3D while running (use tail). This would be a separate utility, called perhaps with the User interface option
"""

#Import standard python libraries
import subprocess
import time
import datetime
import threading
import string
import sys
import os
import types
import shutil
import socket
#Import non-standard python libraries
try:
    import qualisys as qs
except ImportError:
    print(timestamp() + "Can't import qualisys")
    raise
try:
    import ckbot.logical, ckbot.nobus
except ImportError:
    print(timestamp() + "Can't import ckbot")
    raise
try:
    import pygame
except ImportError:
    print(timestamp() + "Can't import pygame")
    raise
	
def timestamp():
    """Returns a timestamp in format Year-month-day Hour:minute:second"""
    return datetime.datetime.fromtimestamp(time.time()).strftime('%Y-%m-%d %H:%M:%S ')
		
"""
Notes:
Motor commands, including motors off, and all ckbot stuff, might end up moved to whatever process handles all robot walking

#    x = float(params.readlines()[0])
#    results.write(str(pow(x,2)))
#    print(timestamp()+ "Trial data written to: " + fname2)
#dummy section above that returns just the value x^2 for test
pkt = Q.pudp.next()
if isinstance(pkt,qs.DataPacket):
    c3d = pkt.comp[0]
        if isinstance(c3d,qs.C_3D_NoLabel):
			x.append(c3d.mrk['x'][0])
			y.append(c3d.mrk['y'][0])
			z.append(c3d.mrk['z'][0])

"""
	
class super_state( object ):
    def __init__(self):
        pass
    def state_method( self, l ):
        raise NotImplementedError()
    def transition_method( self, l ):
        raise NotImplementedError()

class state_pretrial_init(super_state):
    def state_method(self,l):
        global Q
        Q.close() #Trial is reset, so close and reopen connection to flush the buffer
        try:
            Q.connectOnNet("192.168.254.1")
            Q.command("Version 1.9" )
			
            print(timestamp() + "Connected to Qualisys system")
        except socket.timeout:
            print(timestamp() + "Socket timeout when trying to reopen Qualisys connection")
    def transition_method(self,l):
        return state_pretrial()
		
class state_pretrial(super_state):
    def state_method(self,l):
        for evt in l:
            if evt.type == 7:
                if evt.axis == 1:
                    try:
                        left.set_torque(-evt.value)
                        #print(-evt.value)
                    except:
                        left.set_speed(-evt.value)
                if evt.axis == 3:
                    try:
                        right.set_torque(evt.value)
                        #print(-evt.value)
                    except:
                        right.set_speed(evt.value)
    def transition_method(self,l):
        global modifier
        for evt in l:
            if evt.type == 11:
                if evt.button == 11:
                    modifier = 0
                    return state_running_init()
                elif evt.button == 9:
                    print(timestamp() + "Trial aborted, exiting.")
                    return state_exit()
        return state_pretrial()
	
class state_running_init(super_state):
    def state_method(self,l):
        global Q
        left.go_slack()
        right.go_slack()
        #start streaming the frames, restart or call the gait process
        try:
            Q.command("StreamFrames Frequency:100 UDP:%d 3D" % (Q.udp.getsockname()[1]))
            print(timestamp() + "StreamFrames command sent")
        except socket.timeout:
            print(timestamp() + "Socket timeout when trying to send StreamFrames command")
        print(timestamp() + "Running robot gait.")
    def transition_method(self,l):
        return state_running()
	
class state_running(super_state):
    def state_method(self,l):
        pass
    def transition_method(self,l):
        global modifier
        global Q
        global temp
        for evt in l:
            if evt.type == 11:
                if evt.button == 11: #Pausing, streamframes stop
                    try:
                        Q.command("StreamFrames Stop")
                    except socket.timeout:
                        print(timestamp() + "Socket timeout when trying to pause StreamFrames")
                    return state_paused_init()
                elif evt.button == 14:
                    print(timestamp() + "Trial reset.")
                    return state_pretrial_init()
                elif evt.button == 9:
                    Q.close() #aborting trial anyway
                    print(timestamp() + "Trial aborted, exiting.")
                    return state_exit()
                elif evt.button == 15:
                    modifier = modifier - 10
                    print(timestamp() + "Total modifier is " + str(modifier) + " points.")
                elif evt.button == 13:
                    modifier = modifier + 10
                    print(timestamp() + "Total modifier is " + str(modifier) + " points.")
        return state_running()
	
class state_paused_init(super_state):
    def state_method(self,l):
        print(timestamp() + "Paused.")
        left.go_slack()
        right.go_slack()
        #pause or stop gait process
    def transition_method(self,l):
        return state_paused()
			
class state_paused(super_state):
    def state_method(self,l):
        pass
    def transition_method(self,l):
        global modifier
        for evt in l:
            if evt.type == 11:
                if evt.button == 11:
                    return state_running_init()
                elif evt.button == 14:
                    print(timestamp() + "Trial reset.")
                    return state_pretrial_init()
                elif evt.button == 9:
                    print(timestamp() + "Trial aborted, exiting.")
                    return state_exit()
                elif evt.button == 12:
                    return state_write()
                elif evt.button == 15:
                    modifier = modifier - 10
                    print(timestamp() + "Total modifier is " + str(modifier) + " points.")
                elif evt.button == 13:
                    modifier = modifier + 10
                    print(timestamp() + "Total modifier is " + str(modifier) + " points.")
        return state_paused()	

class state_write(super_state):
    def state_method(self,l):
        global temp
        print(timestamp() + "Trial accepted. Copying results to permanent file.")
        for pkt in Q.pudp:
            temp.write(str(pkt)) #change this later to actual data, test for now
            if type(pkt) == types.NoneType:
                break
        temp.close()
        shutil.copyfile("temp","result"+sys.argv[1])
    def transition_method(self,l):
        return state_exit()

class state_exit(super_state):
    def state_method(self,l):
        raise NotImplementedError()
    def transition_method(self,l):
        raise NotImplementedError()
	
if __name__=="__main__":
    global temp
    global modifier
    global Q
    global params
    try:
        params = open(("param" + sys.argv[1]),'r')
    except IOError:
        print(timestamp() + "Failed to open specified param file")
        raise
    except IndexError:
        print(timestamp() + "robot_runner.py not invoked with an argument")
        raise
    temp = open("temp",'w')
    Q = qs.QTM(0.01)
    try:
        c = ckbot.logical.Cluster()
    except IOError:
        ckbot.logical.DEFAULT_BUS = ckbot.nobus
        c = ckbot.logical.Cluster()
        print(timestamp() + "Couldn't connect, using virtual modules as standin.")

    c.populate(count=2,fillMissing=True,required=[ 0x01, 0x2 ])
    right = c[1]
    left = c[2]

    pygame.init()
    try:
        ps3 = pygame.joystick.Joystick(0)
        ps3.init()
    except pygame.error:
        print(timestamp() + "Could not find joystick.")
    mystate = state_pretrial_init()
    l = pygame.event.get()
    modifier = 0
    #You may need to press the PS button to get the controller to communicate
    while True:
        try:
            mystate.state_method(l)
            l = pygame.event.get()
            mystate = mystate.transition_method(l)
            if type(mystate) == type(state_exit()):
                break
        except KeyboardInterrupt:
            print("Exiting")
            raise

"""Shut down the connection"""
if Q.isConnected():
    try:
        Q.command("StreamFrames Stop UDP:%d 3DNoLabels" % (Q.udp.getsockname()[1]))
        print(timestamp() + "Sent StreamFrames Stop signal to Qualisys system")
    except:
        print(timestamp() + "Failed to send Qualisys command StreamFrames Stop")
Q.close()