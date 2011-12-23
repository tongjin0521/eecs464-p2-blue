# Unit Testing - Stella Latscha 2010

import random
import unittest
import ckbot.logical
import time
import sys
import yaml

c = ckbot.logical.Cluster() #populates the modules globally
c.populate()


def canodOf( m ):
    m.get_od()
    l = [ (o.name, o.permission, o.type, o.description) for o in m.od.name_table.itervalues() ] #creates a yml file with the OD contents
    l.sort() #alphabetically sorts the information in the OD
    return [list(li) for li in l]

def canodFileName( m ):
    return "ObjDict-%s.yml" % m.code_version #creates the proper filename for the designated module
    
   
values = []
nargv = [sys.argv[0]]
nodeNames = set([ m.name for m in c.itermodules()])
for arg in sys.argv[1:]:
  if len(arg)>4 and arg[2:] in nodeNames: #will only do this if -gNx$$ is entered, not for standard modules. uses the module as an example of a good OD
    # generate od file from node arg[2:]
    m = getattr(c.at,arg[2:]) #finds the Node ID
    m.get_od() #walks the object dictionary
    cod = canodOf(m) #writes a yml file for m
    fn = canodFileName( m ) #creates a filename function assigned to fn for the module ID
    fo = open(fn,"w") #opens the file, 'write' mode
    yaml.safe_dump( cod, stream=fo ) #puts the object dictionary of the module into the newly created yml file
    fo.close() #closes the yml
    print "Wrote object dictionary YAML to ",fn
  elif arg in nodeNames: # regular module command
    values.append(m)   #adds module id to values variable
  else:
    nargv.append(arg) #sends command line IDs to new list
sys.argv[:] = nargv # replace argv with filtered version that had nodes removed

def loadCanonOD( ymlFile ):
    return yaml.safe_load( ymlFile )

def saveCanonOD( canod, ymlFile ):
    yaml.safe_dump( canod, stream=ymlFile )


class TestRB(unittest.TestCase):
           
    def test_ReadWriteEEProm(self):
        #check that the value does change after reset
        global values
        
        for m in values:
            m.get_od() #walks object dictionary
            m.od.set_feedfreq(200) #setting the feedfreq as a variable
            m.reset()
            time.sleep(.5)
            val = m.od.get_feedfreq()
            m.od.set_feedfreq(val - 5) #changing the feedfreq value
            m.reset()
            time.sleep(.5)
            val2 = m.od.get_feedfreq() #sets the current feedfreq as a variable
            if val2 == (val - 5):
              print 'Passed ReadWriteEEProm'
                #asserts that they are equal
    
    def test_ReadOnlyEEProm(self):
        #test that the value hasn't changed after reset
        global values
        
        for m in values:
            #tests that there is no 'set' function for the EEprom
            try:
              m.set_0x1400(-9000)
              return False
            except AttributeError:
              print 'Passed ReadOnlyEEProm'
    
    def test_ObjectDictionary(self):
        global values
        #runs 18 times?
        for m in values:
            f = open( canodFileName( m ), "r" ) #opens the corresponding version of the yml file for this module
            cod = yaml.safe_load( f ) 
            f.close()
            bad = False
            for ai, bi in zip(cod,canodOf(m)): #compares the current module OD with the correct OD
                if tuple(ai) != tuple(bi): #will return false if values aren't equal
                    bad = True
            if bad:
                print 'Failed ObjectDictionary'
            else:
                print 'Passed ObjectDictionary'
            
class TestCalibration(unittest.TestCase):
    
    def test_feedbackValue(self):
        #test that the feedback is pretty accurate
        global values
       
        for m in values:
          m.set_pos(-8000)
          val = m.get_pos()
          if -9000 <= val <= -7000:
            print 'Passed feedbackValue', val
          else:
            print 'Failed feedbackValue', val
        
 
if __name__ == '__main__':
    unittest.main()
    
    
