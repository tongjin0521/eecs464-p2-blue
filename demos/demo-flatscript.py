"""
file: demo-flatscript.py

This is a demonstration of the "flatscript" feature in JoyApp. It is a simple wrapper that
allows you to define event callback functions to run a JoyApp, and you pretty much
need nothing else.

The flatscript feature
"""
### This part creates "debug" modules because I didn't want to set up physical ones
from ckbot import nobus
from ckbot.ckmodule import  DebugModule
nids = [5,6,7]
for nid in nids:
    nobus.NID_CLASS[nid]=DebugModule

### This is how you use this flatscript feature
from joy.decl import *
## Typically you would use something like this
# ROBOT=dict( count=##number_of_modules## )

### But for debugging we need something more fancy
ROBOT=dict( arch=nobus, fillMissing=True, required=nids, timeout=0.01 )

### here are examples of event handlers
###   anything with name starting with 'on_', and also 'onStart' and 'onStop' will get used
def on_K_SPACE(evt):
  progress("(say) space key pressed")

def on_K_s(evt):
  progress("(say) 3")
  yield 1
  progress("(say) 2")
  yield 1
  progress("(say) 1")
  yield 1
  progress("(say) done")

def on_K_p(evt):
  robot.Nx06.set_pos(5000)
  app.stop()

def onStart():
  progress("(say) starting")

def onStop():
  progress("(say) stopping")

### You need this at the end. It starts the robot and runs your code
from joy import runFlatScript
runFlatScript(locals())
