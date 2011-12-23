from joy import *

class RecordCSV(JoyApp):
    
    def __init__(self, filename, tolerance=50, *args, **kw):
        JoyApp.__init__(self, *args, **kw)

        self.filename = filename
        self.tolerance = tolerance

    def onStart(self):
        self.getters = {}
        self.hist = set()
        self.recs = [] #[{}], list of dictionaries of movements
        self.prev_rec = {}

        if self.robot is None:
            raise RuntimeError("Yer robot is dead, matie")

        for m in self.robot.itermodules():
            print m.name
            self.getters[m.name] = m.get_pos

        print "Start Recording."

    def onEvent(self,evt):
        # Space records one position
        if evt.type==KEYDOWN and evt.key==ord(' '):
            self.recordPos()

        # "Enter" key ends recording
        elif evt.type==KEYDOWN and evt.key==13:
            self.saveCSV(self.filename)
            self.stop()

        # "Backspace" key deletes last recording from list
        elif evt.type==KEYDOWN and evt.key==8:
            self.removePrevPos()

        # "l" key prints current recordings
        elif evt.type==KEYDOWN and evt.key==ord('l'):
            for rec in self.recs:
                print rec

        # "p" key will stiffen all modules except the ones specified
        elif evt.type==KEYDOWN and evt.key==ord('p'):
            mod_str = raw_input("Modules to keep loose (separate by spaces): ")
            mods = mod_str.split(" ")
            self.stiffenModules(mods)

        elif evt.type==KEYDOWN and evt.key==ord('o'):
            mod_str = raw_input("Modules to keep stiff (separate by spaces): ")
            mods = mod_str.split(" ")
            self.loosenModules(mods)
        
    def recordPos(self):
        res = {}
        for mnm,get in self.getters.iteritems():
            pos = get()
            recorded = self.prev_rec.has_key(mnm)

            if recorded and abs(self.prev_rec[mnm]-pos) < self.tolerance:
                continue
            res[mnm] = pos
            self.hist.add(mnm)

    def removePrevPos(self):
        self.recs.pop()

    def stiffenModules(self, mods=None):
        if mods is None:
            mods = []
        for mnm,get in self.getters.iteritems():
            if mnm in mods:
                continue
            pos = get()
            eval('self.robot.at.mnm.set_pos(pos)')  

    def loosenModules(self, mods=None):
        if mods is None:
            mods = []
        for mnm,get in self.getters.iteritems():
            if mnm in mods:
                continue
            eval('self.robot.at.mnm.set_pos(9001)')
        
    def saveCSV(self, filename):
        try:
            f = open(filename, "w")

            hist_list = sorted(self.hist)

            line = ["\"t\""]
            for mnm in hist_list:
                line.append("\"%s.pos\""%mnm)
                
            res = ",".join(line)
            f.write("%s\n"%res)
      
            num = 1
            for rec in self.recs:
                num += 1
                line =  ["%d" % num] + [rec.get(mnm,' ') for mnm in hist_list ]
                f.write("%s\n" % ",".join(line)  )

        except IOError,ioe:
            sys.stderr.write( "ERROR accessing '%s' -- %s" % (filename,str(ioe)))
        finally:
            if f:
                f.close()

if __name__ == "__main__":
    print """
       Demonstration of the Gait Recorder
       ----------------------------------

       Shows the default behaviour of a CKBot Gait Recorder.
       Used to store positions and save them to a .csv file 
       for use in running CKBot Gaits. A CKBot Cluster must
       be connected for this demo to function properly! 

       Implemented Commands:
            <Space>     -  Record position
            <Enter>     -  Save and Quit
            <Backspace> -  Delete Previous Recording
            l        -  List current recordings

       Features coming to a demo near you...:
           - Playback
           - Stiffen selected modules
           - Loosen selected modules
           - Record based on timer events
    """
    recorder = rec_csv("test_rec.csv",walk=1)
    recorder.run()
