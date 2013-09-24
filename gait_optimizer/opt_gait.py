from cmd import Cmd
import subprocess
from evaluator import evaluate
import time
import threading
import os
import string
import numpy
import optutil
import sys


"""A Cmd based module for the robot gait optimization"""

class opt_UI( Cmd ):
    """The commandline user interface to the optimization routine"""
			
    def __init__(self):
        Cmd.__init__(self)
        self.prompt = "Optimizer >> "
        #self.RRstdout = open('RRstdout','w')
        self.RRstderr = open('RRstderr','w')
        self.routine_flag = 'once' # this flag is the type of routine. 'once' is a standin for do one iteration (one Robot run, one evaluation, stops after computing new params)
        self.n = 0 #this flag keeps track of the trial number we are on
        self.sim0 = numpy.array([[-5],[5]]) #a dummy initial simplex
        self.opt = optutil.fminIter(self.sim0,None,0,0) #our opt iterator item
        self.val = self.opt.next() #our list to place scores
        self.param0 = self.opt.next() #compute a starting set of parameters
        self.writeparams(self.param0)
		
    def writeparams(self,param_n):
        """"Write gait parameters to the param[n]file"""
        fname = "param"+str(self.n)
        paramfile = open(fname,'w+')
        for i in range (0,len(param_n)):
            paramfile.write(str(param_n[i]))
        paramfile.close()
        self.stdout.write(self.prompt) #These two lines added to properly get the prompt back for aesthetic reasons only
        self.stdout.flush()


    def generate_params(self,line):
        """Evaluate the score of the last trial to generate new gait parameters with fminIter, then call the writeparams function"""
        score = evaluate(self.n)
        self.val[-1] = score
        self.n = self.n + 1 #now we increment trial number. param[n] generates result[n] which yields param[n+1]
        params = self.opt.next()
        print("Parameters: "+str(params) + ", Score: " + str(score))
        self.writeparams(params)
        
		
    def do_Run(self,line):
        """Open robot_runner.py as a new porcess and start the polling to monitor it"""
        self.roborun = subprocess.Popen(['python', 'robot_runner.py', str(self.n)], stdout=sys.stdout, stderr = self.RRstderr)
        self.check_poll()
        #note that while the interpreter is not blocked, it does not display propmt again and 'enter' repeats the command, which aesthetically bothers me
		
    def do_test(self,line):
        """Returns None type if the robot_runner process is running and 0 if it is terminated. Error if it has never been run at all"""
        try:
            print self.roborun.poll()
        except AttributeError:
            print "Process robot_runner.py hasn't yet been run"

    def do_Kill(self,line):
        """Kills the robot_runner.py process"""
        try:
            self.roborun.kill()
        except OSError:
            print "OSError. Process may no longer exist."
        except:
            print "Error. Robot Runner process may not exist."

    def do_exit(self, line):
        """Quit"""
        return True

    def run(self):
        self.cmdloop()
    intro = "Welcome to the BIRDS-lab Robot Gait Optimizer.\nType ? or  help for a list of commands."
	
    def check_results(self):
        """Check that we have a results file that matches our trial number"""
        fname = "result"+str(self.n)
        if (os.path.isfile(fname)):
            self.generate_params(self)

    def check_poll(self):
        """Uses a timer based system to poll the robot_runner.py process"""
        t = threading.Timer(0.010,self.check_poll) #set up a timer
        if (self.roborun.poll() == None): #Roborun hasn't terminated, so keep checking
            t.start()
        else:
            if self.routine_flag == 'once':
                self.check_results()

    def help_exit(self):
        print "Exit this python cmd utility."

    def help_Run(self):
        print "Start the robot runner and data collection utility as a separate process."

    def help_Kill(self):
        print "Kill the robot runner."

    def help_help(self):
        print "Display a list of commands or documentation on individual commands."

## Start Qt event loop unless running in interactive mode.
if __name__ == '__main__':
    opt = opt_UI()
    opt.run()
    