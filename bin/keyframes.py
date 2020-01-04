#!/usr/bin/env python
#mode is zero and we have to free the motors
'''
FILE demo-playback.py

This program records the positions of the motors and has the ability to play the recording.
This file will simply look for modules and call the PoseRecorder class. Then PoseRecorder class will
provide the command interface on the terminal and functions for each command. PoseRecorder will
capture the position of the motor by using get_pos and later set the motor using set_pos
'''
if __name__ != "__main__":
  from sys import stderr,exit
  stderr.write("Run this as a script")
  exit()

from ckbot.posable import PoseRecorder, PoseRecorderCLI
from ckbot import dynamixel as DX, pololu as PO, polowixel as PX
from sys import argv
from argparse import ArgumentParser
parser = ArgumentParser(description="""
    demo-playback -- a commandline interface to posable programming. Use the 'help' command at the prompt to obtain help
""")

parser.add_argument('--arch','-a',action='store',type=str,default='dynamixel',help='Robot bus architecture to use: "dynamixel", "pololu", "polowixel"/"wixel", "simulation"; any prefix valid')
parser.add_argument('--count','-c',action='store',type=int,default=1,help='Number of modules to find on bus')
args = parser.parse_args(argv[1:])

#creates objects for the modules
import ckbot.logical as L
if "dynamixel".startswith(args.arch):
    c = L.Cluster(arch=DX)
elif "pololu".startswith(args.arch):
    c = L.Cluster(arch=PO)
elif "polowixel".startswith(args.arch) or "wixel".startswith(args.arch):
    c = L.Cluster(arch=PX)
elif "simulation".startswith(args.arch):
    raise RuntimeError("Not implemented YET")

#number of modules to look for
n = int(args.count)
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
pr = PoseRecorder(c.values(),c.p.update)

cli = PoseRecorderCLI(pr,c)
try:
    cli.run()
finally:
    pr.off()
