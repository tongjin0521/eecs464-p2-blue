#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr  1 14:23:10 2020

@author: shrevzen
"""
from ast import Yield
from joy.plans import Plan
from joy.decl import *
from numpy import asarray
from p2sim import ArmAnimatorApp
import numpy as np
import time

DEBUG = False

class MoveToPoint(Plan):
  def __init__(self,app):
    Plan.__init__(self,app)
    self.app = app
    self.goal_point = "WHERE AM I GOING?"
    self.l1 = self.app.l1
    self.l2 = self.app.l2
    self.l3 = self.app.l3
    self.curr_angles = None
    self.num_its_per_step = 20

  def curr_tool_pos(self):
    theta_0 = self.curr_angles[0,0]
    theta_2 = self.curr_angles[1,0]
    theta_t = self.curr_angles[2,0] - np.pi/2 + theta_2
    p_x = (self.l2 * np.sin(theta_2) + self.l3 * np.cos(theta_t)) * np.cos(theta_0)
    p_y = (self.l2 * np.sin(theta_2) + self.l3 * np.cos(theta_t)) * np.sin(theta_0)
    p_z = self.l1 + self.l2 * np.cos(theta_2) - self.l3 * np.sin(theta_t)
    return np.array([[p_x],[p_y],[p_z]])

  def get_jacobian(self):
    jacobian = np.zeros((3,3))
    theta_0 = self.curr_angles[0,0]
    theta_2 = self.curr_angles[1,0]
    theta_t = self.curr_angles[2,0] - np.pi/2 + theta_2
    M = self.l2 * np.sin(theta_2) + self.l3 * np.cos(theta_t)
    jacobian[0,:] = np.array([-M* np.sin(theta_0),np.cos(theta_0) *(self.l2 * np.cos(theta_2) - self.l3 * np.sin(theta_t)), - self.l3 *np.cos(theta_0) * np.sin(theta_t) ])
    jacobian[1,:] = np.array([M* np.cos(theta_0),np.sin(theta_0) *(self.l2 * np.cos(theta_2) - self.l3 * np.sin(theta_t)), - self.l3 *np.sin(theta_0) * np.sin(theta_t) ])
    jacobian[2,:] = np.array([0,-self.l2 * np.sin(theta_2) - self.l3* np.cos(theta_t), - self.l3 * np.cos(theta_t)])
    return jacobian

  def set_motor_pos(self):
    yield self.app.arm[0].set_pos( int(self.curr_angles[0] / np.pi * 180 * 100))
    yield self.app.arm[2].set_pos( int(self.curr_angles[1] / np.pi * 180 * 100))
    yield self.app.arm[3].set_pos( int(self.curr_angles[2] / np.pi * 180 * 100))
    # while (abs(self.app.arm[0].get_pos() - self.curr_angles[0] / np.pi * 180 * 100) > 100 or abs(self.app.arm[2].get_pos() - self.curr_angles[1] / np.pi * 180 * 100) > 100 or abs(self.app.arm[3].get_pos() - self.curr_angles[2] / np.pi * 180 * 100) > 100):
    #   progress("waiting")
    #   yield self.forDuration(0.1)
    yield self.forDuration(1)

  def behavior(self):
    assert(self.goal_point.any() is not None)
    self.curr_angles = np.array([[self.app.arm[0].get_pos() / 18000 * np.pi],[self.app.arm[2].get_pos() / 18000 * np.pi],[self.app.arm[3].get_pos() / 18000 * np.pi]])
    err_ending = 1
    step_size = 0.01
    it_num = 0
    while (True):
      d_point = self.goal_point - self.curr_tool_pos()
      if (np.linalg.norm(d_point) < err_ending):
        if DEBUG:
          # print_d_angles = d_angles/ np.pi * 180
          print_curr_angles = self.curr_angles / np.pi * 180
          # progress("d_angles: " + str(print_d_angles[0,0]) +", "+ str(print_d_angles[1,0]) +", "+ str(print_d_angles[2,0]))
          progress("curr_angles: " + str(print_curr_angles[0,0]) +", "+ str(print_curr_angles[1,0]) +", "+ str(print_curr_angles[2,0]))
        yield self.set_motor_pos()
        break
      jacobian = self.get_jacobian()
      d_angles = np.linalg.pinv(jacobian) @ d_point
      if (np.linalg.norm(d_angles) > step_size):
        d_angles = d_angles / np.linalg.norm(d_angles) * step_size
      d_angles[np.abs(d_angles) < 5e-4] = 0
      self.curr_angles += d_angles
      if it_num % self.num_its_per_step == 0:
        if DEBUG:
          print(it_num)
          progress("----------------")
          progress("current pos: " + str(self.curr_tool_pos()[0,0]) +", "+ str(self.curr_tool_pos()[1,0]) +", "+ str(self.curr_tool_pos()[2,0]))
          print_d_angles = d_angles/ np.pi * 180
          print_curr_angles = self.curr_angles / np.pi * 180
          progress("d_angles: " + str(print_d_angles[0,0]) +", "+ str(print_d_angles[1,0]) +", "+ str(print_d_angles[2,0]))
          progress("curr_angles: " + str(print_curr_angles[0,0]) +", "+ str(print_curr_angles[1,0]) +", "+ str(print_curr_angles[2,0]))
        yield self.set_motor_pos()
      it_num +=1
    self.goal_point = "WHERE AM I GOING?"

class DrawSquare(Plan):
  def __init__(self,app):
    Plan.__init__(self,app)
    self.app = app   

  def behavior(self):
      line_num = 0
      for line_i in self.app.target_square:
        progress("drawing line" + str(line_num))
        line_num +=1
        for point_i in line_i:
          self.app.moveP.goal_point = point_i
          self.app.moveP.start()
          while (type(self.app.moveP.goal_point).__name__ != 'str'):
            yield self.forDuration(0.5)
      progress("finished")


class MyArmSim(ArmAnimatorApp):
    def __init__(self,Tp2ws,**kw):
      ###
      ### Student team selection -- transform from workspace coordinates to world
      ###
      Tws2w = asarray([
           [1,0,0,  5],
           [0,1,0, -33/2],
           [0,0,1,0],
           [0,0,0,  1]
      ])
      ###
      ### Arm specification
      ###
      self.l1 = 15
      self.l2 = 17
      self.l3 = 23
      self.draw_num_points_per_line = 10.0
      self.draw_zOffset = 0

      armSpec = asarray([
        [0,0.0001,1,0,0], #base with 0 length
        [0,1,0,self.l1,-1.57],  # fisrt joint with fixed orientation
        [0,1,0,self.l2,np.pi /6],
        [0,1,0,self.l3,np.pi /6],
      ]).T
      ArmAnimatorApp.__init__(self,armSpec,Tws2w,Tp2ws,
        simTimeStep=0.1, # Real time that corresponds to simulation time of 0.1 sec
        **kw
      )
      self.moveP = MoveToPoint(self)
      self.drawP = DrawSquare(self)
      self.Tp2w = Tws2w @ Tp2ws

    def show(self,fvp):
      fvp.plot3D([0],[0],[0],'^k',ms=10) # Plot black triangle at origin
      return ArmAnimatorApp.show(self,fvp)
    
    def onStart(self):
      ArmAnimatorApp.onStart(self)


    ###
    ### TEAM event handlers go here
    ###    Handle events as you see fit, and return after
    def on_K_r(self,evt):
      progress("(say) r was pressed")
    
    def onEvent(self,evt):
      # Ignore everything except keydown events
      if evt.type != KEYDOWN:
        return ArmAnimatorApp.onEvent(self,evt)
      ## disable this block (change to 0) to use on_K for these keys
      if 1: 
        # row of 'a' on QWERTY keyboard increments motors
        p = "asdfghjkl".find(evt.unicode)
        if p>=0:
          self.arm[p].set_pos(self.arm[p].get_goal() + 500)
          return
        # row of 'z' in QWERTY keyboard decrements motors
        p = "zxcvbnm,.".find(evt.unicode)
        if p>=0:
          self.arm[p].set_pos(self.arm[p].get_goal() - 500)
          return
        if evt.key == K_w:
          progress("Drawing a square:")
          # square_x = input("x")
          # square_y = input("y")
          # square_s = input("s")
          square_x = 20.32 /2 
          square_y = 27.94 / 2
          square_s = 10
          progress("x: "+str(square_x)+" y: "+str(square_y)+ " z: "+ str(square_s))
          self.discretize_square(square_x,square_y,square_s)
          self.drawP.start()
          return 
        if evt.key == K_p:
          goal_point = np.array([[20],[0],[5]])
          progress("Going to point:")
          progress("x: "+str(goal_point[0,0])+" y: "+str(goal_point[1,0])+ " z: "+ str(goal_point[2,0]))
          self.moveP.goal_point = goal_point
          self.moveP.start()
          return 
      return ArmAnimatorApp.onEvent(self,evt)
      
    def discretize_square(self, x, y, s):
      self.target_square = []
      top_line = []
      right_line = []
      bottom_line = []
      left_line = []
      num_points = self.draw_num_points_per_line
      zOffset = self.draw_zOffset

      for i in range(0, int(num_points+1)):
        top_line.append([x-s+i*2*s/num_points, y+s, zOffset, 1])
        bottom_line.append([x+s-i*2*s/num_points, y-s, zOffset, 1])
        left_line.append([x-s, y-s+i*2*s/num_points, zOffset, 1])
        right_line.append([x+s, y+s-i*2*s/num_points, zOffset, 1])

      top_line_t = []
      bottom_line_t = []
      left_line_t = []
      right_line_t = []

      for p in top_line:
        top_line_t.append(np.array([[(p @ self.Tp2w.T)[0]],[(p @ self.Tp2w.T)[1]],[(p @ self.Tp2w.T)[2]]]))
      for p in bottom_line:
        bottom_line_t.append(np.array([[(p @ self.Tp2w.T)[0]],[(p @ self.Tp2w.T)[1]],[(p @ self.Tp2w.T)[2]]]))
      for p in left_line:
        left_line_t.append(np.array([[(p @ self.Tp2w.T)[0]],[(p @ self.Tp2w.T)[1]],[(p @ self.Tp2w.T)[2]]]))
      for p in right_line:
        right_line_t.append(np.array([[(p @ self.Tp2w.T)[0]],[(p @ self.Tp2w.T)[1]],[(p @ self.Tp2w.T)[2]]]))

      self.target_square.append(top_line_t)
      self.target_square.append(right_line_t)
      self.target_square.append(bottom_line_t)
      self.target_square.append(left_line_t)

if __name__=="__main__":
  # Transform of paper coordinates to workspace

  # rotating around x axis:
  # Tp2ws=asarray([[  1.  ,   0.  ,   0.  ,   0.16],
  #      [  0.  ,   0.71,   0.71,   1.92],
  #      [  0.  ,  -0.71,   0.71,  10.63],
  #      [  0.  ,   0.  ,   0.  ,   1.  ]])

  # rotating around y axis 45 degrees:
  # Tp2ws = asarray([
  #      [0.7071,0,-0.7071,33/2-20.32/2],
  #      [0,     1,      0,33/2-27.94/2],
  #      [0.7071,0, 0.7071,2],
  #      [0,     0,      0,1]
  # ])

  # No rotation:
  # Tp2ws = asarray([
  #      [1,0,0,33/2-20.32/2],
  #      [0,     1,      0,33/2-27.94/2],
  #      [0,0, 1,0],
  #      [0,     0,      0,1]
  # ])

  # rotating around y axis 90 degrees:
  Tp2ws = asarray([
       [0,0,1,33/2-20.32/2],
       [0,     1,      0,33/2-27.94/2],
       [-1,0, 0,30],
       [0,     0,      0,1]
  ])


  app = MyArmSim(Tp2ws,
     ## Uncomment the next line (cfg=...) to save video frames;
     ## you can use the frameViewer.py program to view those
     ## frames in real time (they will not display locally)
     # cfg=dict(logVideo="f%04d.png")
    )
  app.run()
