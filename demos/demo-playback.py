#!/usr/bin/env python
# mode is zero and we have to free the motors
'''
FILE demo-playback.py

This program records the positions of the motors and has the ability to play the recording.
This file will simply look for modules and call the PoseRecorder class. Then PoseRecorder class will
provide the command interface on the terminal and functions for each command. PoseRecorder will
capture the position of the motor by using get_pos and later set the motor using set_pos
'''
from ckbot.posable import PoseRecorder, PoseRecorderCLI

if __name__ == "__main__":
    #creates objects for the modules
    import ckbot.logical as L
    c = L.Cluster()
    #number of modules to look for
    n = input("How many modules to .populate()? ")
    #Timeout occurs if the modules are not found by the specific time
    c.populate(n,timeout=n*0.5)
    '''
    It is a concrete class which does all the background stuff for recording and playback such as
    loading and saving CSV file, taking snaps, dropping snaps etc.

    Recording is done in .snap by using getPose function of the module
    The playback will play the recording by calucating the  duration for each frame and plays each frame
    by using set_pos command.

    It is useful for those who want to create their own client which uses all the functions in this class
    '''
    pr = PoseRecorder(c.values())
    '''
    This is a concrete class which provides the cmd interface which is used for recording and 
    playback using a PoseRecorder object
    This class imports cmd to do that and has functions defined for specific commands

    Documented commands (type help <topic>):
    ========================================
    closeLoop  count  duration  help    load  pose  reset  save  target
    cmd        drop   exit      inline  off   quit  run    show
    
    closeLoop - Appends the first frame in the end to return to its initial state
    count - Counts the number of times we want to play the recording
    duration - the duration to play the recording
    load - load a CSV file as a recording
    save - save a CSV file as a recording
    pose - save the current pose of the motors to the recording
    target - specify the target modules
    cmd - Specify command to broadcast to targets
    inline - Show inline code for recording
    off - stops the playback
    show - show the recorded positions
  
    Basic Sequence of operation
    1. Record the postions using pose function or load a CSV file
    2. Set the number of times to play using count
    3. Set the duration by duration command
    4. Use run command to play the recording
    '''
    cli = PoseRecorderCLI(pr,c)
    cli.run()
    pr.off()
