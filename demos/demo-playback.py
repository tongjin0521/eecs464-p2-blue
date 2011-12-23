#!/usr/bin/env python
from ckbot.posable import PoseRecorder, PoseRecorderCLI

if __name__ == "__main__":
    import ckbot.logical as L
    c = L.Cluster()
    c.populate(input("How many modules to .populate()? "))
    pr = PoseRecorder(c.values())
    cli = PoseRecorderCLI(pr)
    cli.run()
    pr.off()

