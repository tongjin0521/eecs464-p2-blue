"""Run the robot, collect and log trial data
Feature add list:
-Save data runs as gzipped csv

WRITE = 7 (R1)
PENALIZE = 2 (X)
RUN = 3 (SQUARE)
BACK = 4 (L2)
EXIT = 1 (CIRCLE)

"""
import subprocess
import time
import datetime
from threading import Timer
import sys
from numpy import array
from optutil import fminIter
from evaluator import evaluate
from speak import say



try:
        import pygame
except ImportError:
        print(timestamp() + "Can't import pygame")
        say("Can't import pygame")
        raise
    
def timestamp():
        """Returns a timestamp in format Year-month-day Hour:minute:second"""
        return datetime.datetime.fromtimestamp(time.time()).strftime('%Y-%m-%d %H:%M:%S ')


class super_state( object ):
        def __init__(self):
            self.RRstderr = open('RRstderr','w')
        def state_method( self, l ):
            raise NotImplementedError()
        def transition_method( self, l ):
            raise NotImplementedError()


class state_pretrial_init(super_state):
        def state_method(self,l):
            pass
        def transition_method(self,l):
            return state_pretrial()
            
class state_pretrial(super_state):
        def state_method(self,l):
            pass
        def transition_method(self,l):
            for evt in l:
                if evt.type == 11:
                    if evt.button == 3: #square button
                        return state_run()
                    elif evt.button == 4: # L2
                        return state_back()
                    elif evt.button == 2: # X button
                        say("Trial aborted, exiting.")
                        return state_exit()
            return state_pretrial()


class state_back(super_state):
        def state_method(self, l, n, pt_num):
            
            self.do_Run(n, pt_num)
            
        def transition_method(self,l):          
            return state_choose_init()    


        def do_Run(self, n, pt_num):
           
            """Open hexapod_runner_dist.py as a new process and start the polling to monitor it"""
         
            self.roborun = subprocess.Popen(['python', 'hexapod_runner_dist.py', str(n), str(pt_num), 'back'], stdout=sys.stdout, stderr = self.RRstderr)
            
            self.check_poll()


        def check_poll(self):


            """Uses a timer based system to poll the robot_runner.py process"""
            t = Timer(5,self.check_poll)
            if (self.roborun.poll() == None): #Roborun hasn't terminated, so keep checking
                t.start()
           
            else:
                print("Back to start")
            
class state_run(super_state):
        def state_method(self,l, n, pt_num):
            print("Running trial " + str(n))
            
            
            self.do_Run(n, pt_num)
             
        def transition_method(self,l):          
            return state_choose_init()    


        def do_Run(self, n, pt_num):
            """Open hexapod_runner_dist.py as a new process and start the polling to monitor it"""  
            self.roborun = subprocess.Popen(['python', 'hexapod_runner_dist.py', str(n), str(pt_num), 'forward'], stdout= None, stderr = None)
            
            self.check_poll()


        def check_poll(self):
            """Uses a timer based system to poll the robot_runner.py process"""
            t = Timer(5,self.check_poll)
            if (self.roborun.poll() == None): #Roborun hasn't terminated, so keep checking
                t.start()
            else:
                print ("Run ended")
            
class state_choose_init(super_state):
        def state_method(self, l):
            pass
        def transition_method(self, l):
            return state_choose()


class state_choose(super_state):
        def state_method(self, l):
            pass
        def transition_method(self, l):
            for evt in l:
                if evt.type == 11:
                    if evt.button == 1: # Circle
                         return state_penalize_init()
                    elif evt.button == 7: # R1
                         return state_write_init()
                    elif evt.button == 2:
                         return state_exit()  
                    elif evt.button == 3:
                         return state_run()
                    elif evt.button == 4:
                         return state_back()
            return state_choose()  


class state_write_init(super_state):
        def state_method(self,l):
            pass
        def transition_method(self,l):         
            return state_write()


class state_write(super_state):
        def state_method(self, l):
            pass
        def transition_method(self, l):
            for evt in l:
                if evt.type == 11:
                    if evt.button == 3:
                         return state_run()
                    elif evt.button == 4:
                         return state_back()
                    elif evt.button == 2:
                         return state_exit()


            return state_write()


class state_penalize_init(super_state):
        def state_method(self, l):
            pass
        def transition_method(self, l):
            return state_penalize()


class state_penalize(super_state):
        def state_method(self, l):
            pass
        def transition_method(self, l):
            for evt in l:
                if evt.type == 11:
                    if evt.button == 3:
                         return state_run()
                    elif evt.button == 4:
                         return state_back()
                    elif evt.button == 2:
                         return state_exit()
            return state_penalize()


class state_exit(super_state):
        def state_method(self,l):
            raise NotImplementedError()
        def transition_method(self,l):
            raise NotImplementedError()




def writeparams(params, n):


            if params == None:
                print("Optimize")
                say("Optimize")
            else:         
                fname = "param"+str(n)
                paramfile = open(fname,'w+')
                for i in range (0,len(params)):
                    paramfile.write(str(params[i]) + '\n')
                paramfile.close()








if __name__=="__main__":


        sim0 = array([[.52, .23, 1, 3], [.53, .24, 1.1, 3.1], [.54, .25, 1.2, 3.2], [.55, .26, 1.3, 3.3], [.56, .26, 1.4, 3.4]])
        opt = fminIter(sim0,None,1e-2,1e-2)
        val = opt.next()
        params = opt.next()
        n = 0
        writeparams(params, n)


        pt_num = input("How many markers? ")


        #empty scorefile
        scorefile = open('score.txt', 'w')
        scorefile.close()


        pygame.init()
        try:
            ps3 = pygame.joystick.Joystick(0)
            ps3.init()
        except pygame.error:
            print(timestamp() + "Could not find joystick.")




        mystate = state_pretrial_init()
        l = pygame.event.get()
    
   
        while True:
            try:
                if type(mystate) == type(state_run()):
                    mystate.state_method(l, n, pt_num)
                elif type(mystate) == type(state_back()):
                    mystate.state_method(l, n, pt_num)
                elif type(mystate) == type(state_write_init()):
                    score = evaluate(n)
                    val[-1] = score
                    print val
                    print("Parameters: "+str(params) + ", Score: " + str(score))
                   
                    scorefile = open('score.txt', 'a')
                    scorefile.write(str(score) + '\n')
                    scorefile.close()


                    n = n + 1
                    params = opt.next()
                    writeparams(params, n)
                    
                     
                elif type(mystate) == type(state_penalize_init()):
                    score = 1e9
                    val[-1] = score
                    print val
                    print("Parameters: "+str(params) + ", Score: " + str(score))
                    
                    scorefile = open('score.txt', 'a')
                    scorefile.write(str(score) + '\n')
                    scorefile.close()


                    n = n + 1   
                    params = opt.next()                 
                    writeparams(params, n)
                    
                elif type(mystate) == type(state_exit()):
                    break
                else:
                    mystate.state_method(l)
 
                l = pygame.event.get()
                mystate = mystate.transition_method(l)
 
            except KeyboardInterrupt:
                raise