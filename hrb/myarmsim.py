#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr  1 14:23:10 2020

@author: shrevzen
"""

from numpy import asarray
from p2sim import ArmAnimatorApp

class MyArmSim(ArmAnimatorApp):
    def __init__(self,Tp2ws):
      ###
      ### Student team selection -- transform from workspace coordinates to world
      ###
      Tws2w = asarray([
           [1,0,0,  0],
           [0,1,0, -5],
           [0,0,1,-10],
           [0,0,0,  1]
      ]) 
      ###
      ### Arm specification
      ###
      armSpec = asarray([
              [0,0.02,1,  4,  0],
              [0,   1,0,  4,  0],
              [0,   1,0,  4,  0],
          ]).T
      ArmAnimatorApp.__init__(self,armSpec,Tws2w,Tp2ws)
    
    def onStart(self):
      ArmAnimatorApp.onStart(self)
      ###
      ### TEAM CODE GOES HERE
      ###
      
    def onEvent(self,evt):
      ###
      ### TEAM CODE GOES HERE
      ###    Handle events as you see fit, and return after
      return ArmAnimatorApp.onEvent(self,evt)

if __name__=="__main__":
  # Transform of paper coordinates to workspace
  Tp2ws = asarray([
       [0.7071,0,-0.7071,0],
       [0,     1,      0,0],
       [0.7071,0, 0.7071,0],
       [0,     0,      0,1]
  ])
  app = MyArmSim(Tp2ws)
  app.run()

