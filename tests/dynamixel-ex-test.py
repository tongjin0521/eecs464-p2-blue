from serial import Serial
from struct import pack, unpack
from time import time as now, sleep

def chksum( dat ):
  """Checksum as per EX-106 section 3-2 pp. 17
  """
  return 0xFF ^ (0xFF & sum([ ord(c) for c in dat ]))

def mkPkt( nid, cmd, pars ):
  """Build a Dynamixel EX packet, as per EX-106 section 3-2 pp. 16
  """
  body = pack('BB',nid,len(pars)+2)+cmd+pars
  return '\xff\xff'+body+pack("B",chksum(body))

class PacketReader( object ):
  def __init__( self, ser ):
    self.ser = ser
    self.reset()

  def reset( self ):
    self.buf = ''
    self.expect = 6
    self.eSync = 0
    self.count = 0
    self.eChksum = 0
    self.eLen = 0
    self.eID = 0
    self.pktCount = 0
    self.ser.flush()
    # Debug
    self.pkt = ''
    self.prev = ''
    return

  def statsMsg( self ):
    return [
      'bytes in %d' % self.count,
      'packets in %d' % self.pktCount,
      'sync errors %d' % self.eSync,
      'length errors %d' % self.eLen,
      'id errors %d' % self.eID,
      'crc errors %d' % self.eChksum,
      'buffer %d expecting %d' % (len(self.buf),self.expect)
    ]

  def getPacket( self, maxlen=500 ):
    assert self.expect >= 6
    while len(self.buf)+self.ser.inWaiting()>=self.expect:
      # If we don't have at least self.expect bytes --> nothing to do
      #if len(self.buf)+self.ser.inWaiting()<self.expect:
      #  return None
      rd = self.ser.read( self.ser.inWaiting() )
      self.buf += rd
      assert len(self.buf) >= self.expect
      self.count += len(rd)
      # Expecting sync byte
      if not self.buf.startswith('\xFF\xFF'):
        # --> didn't find sync; drop the first byte
        self.buf = self.buf[1:]
        self.eSync += 1
        self.expect = 6
        continue
      assert self.buf.startswith('\xFF\xFF')
      # Make sure that our nid makes sense
      ID = ord(self.buf[2])
      if ID > 0xFD: # Where 0xFD is the maximum ID
        self.eID+=1
        #print "Invalid ID is %02x" % ID
        self.buf = self.buf[1:]
        continue
      # Pull out the length byte and compute total length
      L = 4+ord(self.buf[3])
      if L > maxlen or L < 6: # Where 6 is the minimum possible packet length
        self.eLen+=1
        self.expect = 6
        self.buf = self.buf[1:]
    continue
      if len(self.buf)<L:
        self.expect = L
        continue
      assert len(self.buf)>=L
      chk = chksum( self.buf[2:L-1] )
      #print ">>"," ".join([ "%02x" % (ord(b)) for b in self.buf[:6] ]),"chk",hex(chk),hex(ord(self.buf[L-1])),L
      if ord(self.buf[L-1]) != chk:
        self.eChksum += 1
        self.expect = 6
        self.buf = self.buf[L:]
        continue
      # At this point we have a packet that passed the checksum in
      #   positions buf[:L+1]. We peel the wrapper and return the
      #   payload portion
      pkt = self.buf[2:L-1]

      self.pkt = pkt
      self.prev = self.buf[:L]

      self.buf = self.buf[L:]
      self.pktCount += 1
      return pkt
    # ends parsing loop
    # Function terminates returning a valid packet payload or None
    return None

  def __iter__(self):
    while True:
      pkt = self.getPacket()
      if pkt is None:
        break
      else:
        yield pkt

class DynamixelTest( object ):
  WRITE_DATA = '\x03'# EX-106 sec. 3-2 pp. 16  (Instruction)
  ADDR_LED = '\x19' # EX-106 sec. 3-4 pp. 20

  def __init__(self, nids ):
    # Create on and off packets for each ID
    self.pkts = [ [
      mkPkt( nid, self.WRITE_DATA, self.ADDR_LED + '\x00' ),
      mkPkt( nid, self.WRITE_DATA, self.ADDR_LED + '\x01' )
    ] for nid in nids ]
    self.inv = {}
    for pos,nid in enumerate(nids):
      self.inv[nid]=pos
    # Open interface
    self.ser = Serial('/dev/ttyUSB0',baudrate=1e6)
    assert self.ser.isOpen()
    self.pr = PacketReader( self.ser )
    self.nids = nids
    print "# ready to test:",nids

  def run( self, rounds, n, dt, pause = 0.1 ):
    assert len(self.nids) == len(n) and len(n) == len(dt)
    # Reset everything
    self.pr.reset()
    #
    print "# Testing for %d rounds" % rounds
    print "#\tN:",n
    print "#\tdT:",dt
    total = len(self.pkts[0][0])*sum(n)
    duration = sum([dti*ni for dti,ni in zip(dt,n)])
    print "#\tSending %d bytes in %g seconds -- %g bps" % (total,duration,total*8.0/duration)
    #
    led = [0]*len(self.nids)
    rx = [0]*len(self.nids)
    ####start = now()
    for r in range(rounds):
      if r:
        sleep(pause)
      for nid in range(len(self.nids)):
        t0 = now()
        ###last = t0
        for k in range(1,1+n[nid]):
          t = now()
          # If we are too early --> sleep
          if t-t0<k*dt[nid]:
            sleep(k*dt[nid]-(t-t0))
          # Toggle LED on nid
          self.ser.write( self.pkts[nid][led[nid]] )
          led[nid] ^= 1
          ####print t-start, nid
          ###print "%7.4f" % (t-last),
          ###last = t
        ###print
      while True:
        pkt = self.pr.getPacket(maxlen=6)
        if pkt is None:
          break
        nid,_,err = unpack('BBB',pkt[:3])
        if err != 0:
          print "#>> ID 0x%02x error 0x%02x" % (nid,err)
        ni = self.inv.get(nid,None)
        if ni is None:
          print "#>> Bogus ID 0x%0x" % nid
          continue
        rx[ni] += 1
    print "# PacketReader status"
    print "#\t"+"\n#\t".join(self.pr.statsMsg())
    print "# RX counts"
    print "#  "+"  ".join(["0x%02x" % nid for nid in self.nids] )
    print "# "+" ".join(["%5d" % x for x in rx])
    p =[ float(x)/(rounds*ni) for x,ni in zip(rx,n) ]
    print "#  "+"  ".join(["%3g%%" % int(100*pk) for pk in p ])
    return p,total,duration

if __name__=="__main__":
  T = DynamixelTest([2,1])
  dt = 10**linspace(-10,-9)
  res = []
  for dti in dt:
    ptd = T.run(1000,[10,10],[dti,dti])
    p,t,d = ptd
    print "%6.4f" % dti, "%6g" % (t*8.0/d), "%6e" % p[0], "%6e" % p[1]
    res.append((dti,)+ptd)
    sleep(0.3)
  res.sort()
