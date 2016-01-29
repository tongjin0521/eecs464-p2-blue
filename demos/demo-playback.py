#!/usr/bin/env python
from ckbot.posable import PoseRecorder, PoseRecorderCLI

if __name__ == "__main__":
    import ckbot.logical as L
    c = L.Cluster()
    n = input("How many modules to .populate()? ")
    c.populate(n,timeout=n*0.5)
    pr = PoseRecorder(c.values())
    cli = PoseRecorderCLI(pr,c)
    cli.run()
    pr.off()

