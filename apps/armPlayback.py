#!/usr/bin/env python
from joy import *
from joy.remote import Sink as RemoteSink
import ckbot.dynamixel as dynamixel
import ckbot.logical as logical
from math import copysign, cos, sin, pi, atan2, sqrt
from time import time as now, sleep
from numpy import matrix 

"""
armPlayback: records a set of poses, then interpolates and plays them
             back at the desired rate
"""
class armPlayback( JoyApp ):

    def __init__( self, *args, **kw ):
        """
        Initialize a given number of clusters with the 
        given number of modules
        """        
        #n_clust = input("How many clusters")
        n_clust = 1
        
        ## do something with asking user for usb busses, etc ..

        p = dynamixel.Protocol()
        c = logical.Cluster(p)
        self.num_servos = input("How many modules to .populate()?" )
        c.populate( num_servos )
        self.c = c 
        self.zeros = [0]*len(self.num_servos)
        self.record_time = None
        self.input_pos = None
        self.set_speed_limit( 2.0 )
                                          
    def set_speed_limit( self, lim ):
        """
        Set speed limit for all servos as a safety measure
        """
        for m in self.c.itermodules():
            m.set_speed( lim )
        
    def reset( self ):
        self.input_pose = [[-1] + self.zeros]
        for m in self.c.itermodules():
            m.go_slack()

    def recordPoses( self, normalize=True ):
        """
        Record a given number of poses, then normalize the data 
        """
        self.record_time = 1.0
        input_pose = []
        input_ts = []
        self.reset()
        pose_num = 0
        t0 = now()
        while True:
           resp = raw_input("Press return to capture pose, or q to quit recording")
           if resp is 'q':
             break
           current_pose = [ m.get_pos() for m in c.itermodules() ]
           input_ts.append( now()-t0 )
           input_pose.append(current_pose)
        cnt=0
        # If we ought to normalize then add time stamps to data, else
        # use time stamp data 
        for pose in input_pose:
            if normalize:
                self.input_pose.append([self.record_time*(cnt+1)/len(input_pose)] + pose)
            else:
                self.input_pose.append([input_ts[cnt]] + pose)          
            cnt+=1
        print "Number of knots are: %d" % cnt
        return self.input_pose

    def playback( self, playback_time=1.0, period=1.0 ):
        """
        Playback current pose once if normalize is specified then 
        """

	# playback current pose for a given amount of time and with a given period
	if self.record_time is None:
	    print "No recording"
	    return

	# in order to ensure smooth sequences, double the gait and 
	# only choose the region from phase 0.25 to 0.75 and 
 	# divide the period in half ... hack
	period = period/2
	gait = asarray([self.input_pose[0]]\
			+[[t[0]/2]+t[1:] for t in self.input_pose[1:]]\
		        +[[t[0]/2+0.5]+t[1:] for t in self.input_pose[1:]])

        gaitfun = interp1d( gait[:,0], gait[:,1:].T ) # Gait interpolater function
        t0 = now()
	t1 = t0
	while t1-t0 < playback_time:
	    t1 = now()
 	    phi = (t1-t0)/(period*self.record_time)
	    phi = phi%0.5 + 0.25
	    goal = gaitfun(phi)
	    print "Phi: %f: " % phi
	    print "Goal: " + repr(goal)
            for m, g in zip(self.c.itermodules(), goal):
                m.set_pos( g )
            ### Don't know what this is for sleep(0.01)



