#redesigned by Chad Schaffer 6/2015
#questions to cmschaf@umich.edu

"""Run the robot, collect and log trial data
Feature add list:
-Save data runs as gzipped csv
"""
import subprocess
import time
import datetime
from threading import Timer
import sys
import numpy as np
from optutil import fminIter
from evaluator import evaluate
from speak import say
from math import pi, degrees


try:
        import pygame
except ImportError:
        print(timestamp() + "Can't import pygame")
        say("Can't import pygame")
        raise

#global variables for hexapod contact gait specific runs
minYaw = 23
maxYaw = 28
minRoll = 23
maxRoll = 28

minStance = 1.0
maxStance = 2.0
numStance = 5


def timestamp():
        """Returns a timestamp in format Year-month-day Hour:minute:second"""
        return datetime.datetime.fromtimestamp(time.time()).strftime('%Y-%m-%d %H:%M:%S ')


#calls a subprocess that is the gait and passes it params
#returns the return status of the program
def run(n, pt_num, yaw, roll, stance, dir):

        roborun = subprocess.Popen(args=['python', 'hexapod_runner_dist_edit.py', str(n), str(pt_num), str(yaw), str(roll), str(stance), dir],stdout= None, stderr = None)

        check = None

        while check == None:
                check = roborun.wait()

        return check


def wait():

        print('TRIAL ENDED UNEXPECTEDLY')
        print('fix the robot and return to start point')
        print('press square button to continue')
        print('or x button to exit')

        while True:
                l = pygame.event.get()

                for evt in l:
                        if evt.type == 11:
                                if evt.button == 3:
                                        return
                                elif evt.button == 4:
                                        sys.exit()

                

def trials(pt_num):
        
            file = open('result.csv', 'w')
            file.close()
            #create and alter gait params per run
            yawAmp = np.arange(minYaw, maxYaw, dtype=np.float)
            rollAmp = np.arange(minYaw, maxYaw, dtype=np.float)
            stanceVel = np.linspace(minStance, maxStance, numStance)

            j = 0
            i = 0
            z = 0

            while True:
                    
                    check = run(n, pt_num, yawAmp[j], rollAmp[i], stanceVel[z],'forward')

                    if check != 0:
                            wait()
                            continue

                    
                    check = run(n, pt_num, yawAmp[j], rollAmp[i], stanceVel[z],'back')
                            
                    if check != 0:
                            wait()
                            continue
                                    
                        
                    z = z + 1
                    if z == stanceVel.size:
                            z = 0
                            i = i + 1
                            if i == rollAmp.size:
                                    i = 0
                                    j = j + 1
                                    if j == yawAmp.size:
                                            return 1

def eval():
        result = np.genfromtxt('result.csv', delimiter=',')
        speed = 0
        rad = pi / 180
        
        
        for i in xrange(result.shape[0]):
                holder = result[i][0]/ result[i][1]
                print holder
                if holder > speed:
                        if result[i][2] == 0:
                                speed = holder
                                row = i
                        
        print "Best run forward: %s" % '\n'
        print "speed: %f, rollAmp: %f, yawAmp: %f, stanceVel: %f %s" % (speed, result[row][2] * rad, result[row][3] * rad, result[row][4]* rad, '\n')
        
        for i in xrange(result.shape[0]):
                holder = result[i][0]/ result[i][1]
                print holder
                if holder > speed:
                        if result[i][2] == 1:
                                speed = holder
                                row = i
        
        print "Best run backward: %s" % '\n'
        print "speed: %f, rollAmp: %f, yawAmp: %f, stanceVel: %f %s" % (speed, result[row][2] * rad, result[row][3] * rad, result[row][4]* rad, '\n')



if __name__=="__main__":


        pt_num = input("How many markers? ")
        n = 0

        #empty scorefile
        scorefile = open('score.txt', 'w')
        scorefile.close()

        pygame.init()
        try:
            ps3 = pygame.joystick.Joystick(0)
            ps3.init()
        except pygame.error:
            print(timestamp() + "Could not find joystick.")

        print('press square button to begin a set of trails or x button to exit') 

        doneFlag = 0
        while True:
               l = pygame.event.get()
               
               for evt in l:
                       if evt.type == 11:
                               if evt.button == 3:
                                       doneFlag = trials(pt_num)
                               elif evt.button == 4:
                                       sys.exit()


               if doneFlag == 1:
                        eval()

                                       
