from joy import JoyApp, progress
from ckbot.multiprotocol import MultiProtocol
from ckbot.logical import Cluster

class MultiJoyApp(JoyApp):
  def _initRobot(self,robots):
    """
    (private) Initialize a list of robot busses, and allow modules to appear
    on any of them.
    """
    if type(robots) is dict:
        robots = [robots]
    cs = [ self._initNumRobot(n,robot) for n,robot in enumerate(robots) ]
    mp = MultiProtocol(*[c.p for c in cs])
    NIDS = sum([[(nid | ((n+1)<<8))
                for nid in c.keys()]
                    for n,c in enumerate(cs)],[])
    self.robot = Cluster(
       arch=mp,count=len(NIDS),fillMissing=True, required=NIDS
    )
    if self.cfg.robotPollRate:
      self._initPosPolling()
    if self.cfg.minimalVoltage:
      return self._initVoltageSafety()

  def _initNumRobot( self, n, robot ):
    """
    (private) Initialize the ckbot.Cluster robot interface

    INPUT:
      robot -- dict -- Dictionary of settings for robot.populate
    """
    progress("Populating:")
    # Collect names from the cfg
    nn = self.cfg.nodeNames.copy()
    nn.update(robot.get('names',{}))
    robot['names']=nn
    # Show the names in the progress log
    progress("\t# Robot bus #%d" % n)
    for k,v in robot.iteritems():
      progress("\t\t%s=%s" % (k,repr(v)))
    # Check for both protocol= and arch= parameters
    p = robot.get('protocol',None)
    a = robot.get('arch',None)
    if a and p:
      raise ValueError("Do not combine legacy protocol= with arch=")
    # NOTE: A protocol instance can be passed in arch= parameter of Cluster
    if p:
      del robot['protocol']
      a = p
    # Make sure that robot dictionary does not duplicate the arch= parameter
    if a:
      del robot['arch']
    return Cluster(arch=a, **robot)

if __name__=="__main__":
    import ckbot.nobus as NB
    app = MultiJoyApp(
       robot = [
        dict(arch=NB,fillMissing=True,required=[1,2]),
        dict(arch=NB,fillMissing=True,required=[3,4])
    ]  )
    app.run()
