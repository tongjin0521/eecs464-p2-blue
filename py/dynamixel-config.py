#!/usr/bin/env python
"""
The Dynamixel Configuration tool is used to configure dynamixel servos to
preset values. It also serves as a tool too locate dynamixel servos and 
change their IDs

-- Typical usage is as follows:
   # Change servo at ID 0x01 to ID 0x05 and use paramters from rx64-servo.yml
   >>> ./dynamixel-configure.py -c 0x01 -n 0x05 rx64-servo.yml 

   ... more examples ...
"""
from sys import argv, exit as sys_exit
from getopt import getopt, GetoptError
from yaml import load, dump
from struct import unpack
from time import time as now, sleep
from ckbot.ckmodule import progress
import ckbot.dynamixel as dynamixel

DEBUG = []

class dynamixelConfigurator:
    """
    SOME USEFUL DESCRIPTION
    """
    
    def __init__( self, argv, cur_nid=0x01, cfg=None ):
        """
        Initialize the dynamixelConfigurator, the configurator reads in 
        arguments and sets servo paramters accordingly 
        """
        self.DEBUG = []
        self.argv = argv
        self.cur_nid = cur_nid
        self.new_nid = None
        self.cfg = cfg        
        self.p = dynamixel.Protocol()
        
    def usage( self ):
        print """
              Usage:
                -h, --help : Returns usage 
                -s, --scan : Scan to find all servos and baudrates on the bus 
                -c, --cur  : Desired servo ID to configure
                -n, --new  : New servo ID
                -f, --yaml : .yml file to read parameters from
                -d, --debug : Set debug flag  
           
              """
        return 

    def parseArgs( self, argv ):
        """
        Parse input arguments, handle if sensible 
        """
        if len(argv) < 1:
            self.usage()
            progress('No arguments passed, see Usage \n')
            return False

        try:
            opts, args = getopt( argv, "hn:c:f:d:sr", ["help", "new", "cur", "yaml", "debug", "scan"])
            
        except GetoptError:
            progress( "Invalid Arguments" )
            self.usage()
            return False

        for opt, arg in opts:
            if opt in ("-h", "--help"):
                self.usage()
                sys_exit()
            elif opt in ( "-n", "--new"):
                self.new_nid = int( arg, 0 )
            elif opt in ("-c", "--cur"):
                self.cur_nid = int( arg, 0 )
            elif opt in ("-b", "--baud"):
                self.baud = int(arg)
                progress("No scan, fixed baudrate: %d " % self.baud)
            elif opt in ("-f", "--yaml"):    
                progress("Loading yaml configuration: %s" % self.cfg )
                self.cfg = load( open( arg ))
            elif opt in ("-s", "--scan"):
                self.scanAll()
                return False
        return True

    def scanAll( self ):
        """ 
        A utility function that will scan all baudrates and return all
        nodes found. 
        Exits upon completion
        """
        nid_str = []
        bauds = self.p.bus.scanBauds(useFirst=False)
        bauds = list(set(bauds))
        for baud in bauds:
            self.p.bus.reconnect(baudrate = baud)
            self.p.reset()
            for nid in self.p.pnas.keys():
                tpe = self.read_type(nid)
                nid_str.append(' ID 0x%02x: %s Module found at baudrate %d ' % (nid, tpe, baud))
        for s in nid_str:
            progress(s)

    def _safe_nid( self, nid):
        """
        Check to ensure that we don't try to overwrite another nid
        """
        if nid is not None:
            if self.p.pnas.has_key(nid):
                progress("ID 0x%02x already exists cannot set new node id" % nid)
                return False
        return True

    def write_nid(self, nid_old, nid_new):
        """
        Write old node id to new node id
        """
        if not self._safe_nid( nid_new ):
            return            
        # Update nid if allowed
        if nid_new == 0x01:
            progress("Warning: NID 0x01 is reserved for configuration")
        if nid_new is None:
            return
        node = self.p.pnas[nid_old]
        progress("Setting old NID 0x%02x to new NID 0x%02x" % (nid_old, nid_new))
        node.mem_write_sync( getattr( node.mm, 'ID'), nid_new)
        return  

    def write_config( self, nid ):
        """
        Write node parameters as defined by config (cfg)
        """
        baud = None
        # For memory address names set those attributes 
        node = self.p.pnas[nid]
        if self.cfg is None:
            progress("No configuration defined")
            return 
        progress('Configuring ID 0x%02x with parameters %s' % (nid, self.cfg))
        node = self.p.pnas[nid]
        for name, val in self.cfg.iteritems():
            # Check for node id setting 
            print name, val
            if name == 'ID':
                sys.exit(" Setting ID through .yml file not allowed, see --help")
            # Check for baudrate change, if so, write baud last 
            if name == 'baud':
                baud = val
                continue
            # This is pulled out of DynamixelModule         
            if hasattr( node.mm, name ):
                print "Writing %s to %s" % (repr(name), repr(val))
                node.mem_write_sync( getattr( node.mm, name ), val )
            else:
                raise KeyError("Unknown address '%s'" % name)
        # Write baudrate last 
        node.mem_write_sync( getattr( node.mm, 'baud' ), baud )

    def read_type(self, nid):
        """
        Read the servo type, returned upon scanAll, ... maybe scan?
        """
        node = self.p.pnas[nid]
        cw_limit = node.mem_read_sync( node.mm.cw_angle_limit) 
        ccw_limit = node.mem_read_sync( node.mm.ccw_angle_limit)
        print "cw_limit: %s" % repr(cw_limit)
        print "ccw_limit: %s" % repr(ccw_limit)
        if cw_limit==0 and ccw_limit==0:
            return 'CR'
        return 'Servo'       

    def scan(self, nid, timeout=0.5, retries=1 ):
        """
        Check if a given node id exists on any baudrates,
        returns nid if successful, None if not 
        
        INPUTS:
        p -- object -- protocol object 
        nid -- int -- node id to scan for
        
        """
        progress("Scanning")
        bauds = [ b[1] for b in self.p.bus.ser.getSupportedBaudrates() if b[1]<=1e6]
        bauds.sort(reverse=True)
        bauds.insert(0, 57600)
        found = []
        for baud in bauds:
            self.p.bus.reconnect(baudrate = baud)
            self.p.reset()
            if self.p.pnas.has_key(nid):
                tpe = self.read_type(nid)
                progress(' ID 0x%02x: %s Module  found at baudrate %d ' % (nid, tpe, baud))
                return
        progress( 'ID 0x%02x not found' % nid )
        return 

    def run( self ):
        """
        Read in and parse arguments, perform necessary configuration 
        """
        cntu = self.parseArgs( self.argv )
        if not cntu:
            return        
        # Find desired nid
        self.scan( self.cur_nid )
        # Check that new nid is safe 
        if not self._safe_nid( self.new_nid ):
            return 
        self.write_config( self.cur_nid )
        # After changing configuration we may be at a new baudrate so rescan
        self.scan( self.cur_nid )
        self.write_nid( self.cur_nid, self.new_nid )
        return 

if __name__ == "__main__":
    """
    Run the configurator
    """
    dc = dynamixelConfigurator( argv[1:] )
    dc.run()

