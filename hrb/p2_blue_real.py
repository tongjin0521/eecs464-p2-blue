#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr  1 14:23:10 2020

@author: shrevzen
"""
from pickle import NONE
from joy.plans import Plan
from joy import JoyApp, progress
from joy.decl import *
from joy.misc import *
from numpy import asarray
import numpy as np

pre = "P2 BLUE - "

class MoveToPoint(Plan):
    def __init__(self,app):
        Plan.__init__(self,app)
        self.app = app
        self.goal_point = "WHERE AM I GOING?"
        self.l1 = self.app.l1
        self.l2 = self.app.l2
        self.l3 = self.app.l3
        self.l4 = self.app.l4
        self.curr_angles = None
        self.num_its_per_step = 20

    def curr_tool_pos(self):
        theta_0 = self.curr_angles[0,0]
        theta_2 = self.curr_angles[1,0]
        theta_t = self.curr_angles[2,0] - np.pi/2 + theta_2
        p_x = (self.l2 * np.sin(theta_2) + self.l3 * np.cos(theta_t)) * np.cos(theta_0) - self.l4 * np.sin(theta_0)
        p_y = (self.l2 * np.sin(theta_2) + self.l3 * np.cos(theta_t)) * np.sin(theta_0) + self.l4 * np.cos(theta_0)
        p_z = self.l1 + self.l2 * np.cos(theta_2) - self.l3 * np.sin(theta_t)
        return np.array([[p_x],[p_y],[p_z]])

    def get_jacobian(self):
        jacobian = np.zeros((3,3))
        theta_0 = self.curr_angles[0,0]
        theta_2 = self.curr_angles[1,0]
        theta_t = self.curr_angles[2,0] - np.pi/2 + theta_2
        M = self.l2 * np.sin(theta_2) + self.l3 * np.cos(theta_t)
        jacobian[0,:] = np.array([-M* np.sin(theta_0) - self.l4 * np.cos(theta_0),np.cos(theta_0) *(self.l2 * np.cos(theta_2) - self.l3 * np.sin(theta_t)), - self.l3 *np.cos(theta_0) * np.sin(theta_t) ])
        jacobian[1,:] = np.array([M* np.cos(theta_0) - self.l4 * np.sin(theta_0),np.sin(theta_0) *(self.l2 * np.cos(theta_2) - self.l3 * np.sin(theta_t)), - self.l3 *np.sin(theta_0) * np.sin(theta_t) ])
        jacobian[2,:] = np.array([0,-self.l2 * np.sin(theta_2) - self.l3* np.cos(theta_t), - self.l3 * np.cos(theta_t)])
        return jacobian

    def set_motor_pos(self):
        for i in range(3):
            self.app.arm[i].set_pos( int(self.curr_angles[i] / np.pi * 180 * 100))
        yield self.forDuration(1)

    def behavior(self):
        assert(self.goal_point.any() is not None)
        self.curr_angles = np.array([[self.app.arm[0].get_pos() / 18000 * np.pi],[self.app.arm[1].get_pos() / 18000 * np.pi],[self.app.arm[2].get_pos() / 18000 * np.pi]])
        err_ending = 1
        step_size = 0.01
        it_num = 0
        while (True):
            d_point = self.goal_point - self.curr_tool_pos()
            if (np.linalg.norm(d_point) < err_ending):
                # # print_d_angles = d_angles/ np.pi * 180
                # print_curr_angles = self.curr_angles / np.pi * 180
                # # progress("d_angles: " + str(print_d_angles[0,0]) +", "+ str(print_d_angles[1,0]) +", "+ str(print_d_angles[2,0]))
                # progress("curr_angles: " + str(print_curr_angles[0,0]) +", "+ str(print_curr_angles[1,0]) +", "+ str(print_curr_angles[2,0]))
                yield self.set_motor_pos()
                break
            jacobian = self.get_jacobian()
            d_angles = np.linalg.pinv(jacobian) @ d_point
            if (np.linalg.norm(d_angles) > step_size):
                d_angles = d_angles / np.linalg.norm(d_angles) * step_size
            d_angles[np.abs(d_angles) < 5e-4] = 0
            self.curr_angles += d_angles
            if it_num % self.num_its_per_step == 0:
                # print(it_num)
                # progress("----------------")
                # progress("current pos: " + str(self.curr_tool_pos()[0,0]) +", "+ str(self.curr_tool_pos()[1,0]) +", "+ str(self.curr_tool_pos()[2,0]))
                # print_d_angles = d_angles/ np.pi * 180
                # print_curr_angles = self.curr_angles / np.pi * 180
                # progress("d_angles: " + str(print_d_angles[0,0]) +", "+ str(print_d_angles[1,0]) +", "+ str(print_d_angles[2,0]))
                # progress("curr_angles: " + str(print_curr_angles[0,0]) +", "+ str(print_curr_angles[1,0]) +", "+ str(print_curr_angles[2,0]))
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
            progress(pre + "drawing line" + str(line_num))
        line_num +=1
        for point_i in line_i:
            self.app.moveP.goal_point = point_i
            self.app.moveP.start()
            while (type(self.app.moveP.goal_point).__name__ != 'str'):
                yield self.forDuration(0.5)
        progress(pre + "finished")


class P2_Blue_App(JoyApp):
    def __init__(self,robot,cfg,square,*arg,**kw):
        ###
        ### Student team selection -- transform from workspace coordinates to world
        ###
        JoyApp.__init__(self,robot=robot, cfg = cfg,*arg, **kw)
        self.Tws2w = asarray([
            [1,0,0,  10],
            [0,1,0, -33/2],
            [0,0,1,8],
            [0,0,0,  1]
        ])
        self.Tp2ws = NONE
        
        ###
        ### Arm specification
        ###
        self.l1 = 26.5 - 10
        self.l2 = 25.3
        self.l3 = 28
        self.l4 = 8.5
        self.s = 26.5

        self.cali_num_points_per_line = 4
        self.draw_num_points_per_line = 10.0
        self.draw_zOffset = 0

        armSpec = asarray([
            [0,0.0001,1,0,0], #base with 0 length
            [0,1,0,self.l1,-1.57],  # fisrt joint with fixed orientation
            [0,1,0,self.l2,np.pi /6],
            [0,1,0,self.l3,np.pi /6],
        ]).T

        self.moveP = MoveToPoint(self)
        self.drawP = DrawSquare(self)
        self.square_param = square
        self.cali_angles = []
        self.rotating_base_fixed = False
        self.arm = [self.robot.at.shoulder, self.robot.at.elbow, self.robot.at.wrist, self.robot.at.rotating_base]
        for motor_i in self.arm:
            motor_i.set_speed(3)

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

    def calculate_Tp2ws(self):
        assert(len(self.cali_angles) == 4 * self.cali_num_points_per_line) 
        motor_0_polarity = 1 # assume CCW is positive
        motor_1_polarity = 1 # assume up is positive
        motor_2_polarity = 1 # assume up is positive
        angle_rotate_about_z = self.cali_angles[0] * motor_0_polarity
        angle_rotate_about_y = self.cali_angles[1] * motor_1_polarity - self.cali_angles[2] * motor_2_polarity
        Tp2ws_z = asarray([
            [np.cos(angle_rotate_about_z), -np.sin(angle_rotate_about_z), 1, 0],
            [np.sin(angle_rotate_about_z),  np.cos(angle_rotate_about_z), 0, 0],
            [0, 0, 1, 0],
            [0,     0,      0,1]
        ])
        Tp2ws_y = asarray([
            [np.cos(angle_rotate_about_y),  0, np.sin(angle_rotate_about_y), 0],
            [0, 1, 0, 0],
            [-np.sin(angle_rotate_about_y), 0, np.cos(angle_rotate_about_y), 0],
            [0,     0,      0,1]
        ])
        Tp2ws = Tp2ws_y @ Tp2ws_z    # change in orientation
        # change in position
        Tp2ws[2,3] = 33/2*np.cos(angle_rotate_about_y) # move in z direction
        arm_proj = self.l2 * np.cos(self.cali_angles[1] * motor_1_polarity) + self.l3 * np.cos(-angle_rotate_about_y)
        Tp2ws[1,3] = 33/2 + arm_proj * np.sin(angle_rotate_about_z) # move in y direction
        Tp2ws[0,3] = arm_proj * np.cos(angle_rotate_about_z) - self.s # move in x direction

        return Tp2ws




    def onStart(self):
        pass

    def onEvent(self,evt):
        ## disable this block (change to 0) to use on_K for these keys
        if evt.type == KEYDOWN: 
            p = "ghjk".find(evt.unicode)
            if p>=0:
                self.arm[p].set_pos(self.arm[p].get_pos() + 500)
                return
            p = "vbnm".find(evt.unicode)
            if p>=0:
                self.arm[p].set_pos(self.arm[p].get_pos() - 500)
                return
            if evt.key == K_r:
                if not self.rotating_base_fixed:
                    return progress(pre + "Pls fix the base first")
                progress(pre + "Recording point~")
                # progress(pre + str(self.app.arm[0].get_pos() / 18000) + " "+ str(self.app.arm[1].get_pos() / 18000) + " "+ str(self.app.arm[2].get_pos() / 18000))
                self.cali_angles.append(np.array([[self.app.arm[0].get_pos() / 18000 * np.pi],[self.app.arm[1].get_pos() / 18000 * np.pi],[self.app.arm[2].get_pos() / 18000 * np.pi]]))
                return progress(self.cali_angles[-1])
            if evt.key == K_s:
                # set rotating base angle
                self.arm[3].set_pos(self.arm[3].get_pos())
                self.rotating_base_fixed = True
                return
            if evt.key == K_d:
                self.Tp2ws = self.calculate_Tp2ws()
                self.Tp2w = self.Tws2w @ self.Tp2ws
                input(pre + "Press Enter when you put the robotic arm in ready pose and ready to draw")
                for i in range(3):
                    self.arm[i].set_pos(self.arm[i].get_pos())
                progress(pre + "Drawing a square~")
                square_x = self.square_param['x']
                square_y = self.square_param['y']
                square_s = self.square_param['s']
                # progress("x: "+str(square_x)+" y: "+str(square_y)+ " z: "+ str(square_s))
                self.discretize_square(square_x,square_y,square_s) # set self.target_square
                self.drawP.start()
                return 
            if evt.key == K_p:
                goal_point = np.array([[20],[0],[5]])
                progress(pre + "Going to point: " + "x: "+str(goal_point[0,0])+" y: "+str(goal_point[1,0])+ " z: "+ str(goal_point[2,0]))
                self.moveP.goal_point = goal_point
                self.moveP.start()
                return
            return JoyApp.onEvent(self,evt)
        return
    
if __name__=="__main__":
    motors_name = {
        0x0C:'shoulder',
        0x28:'elbow',
        0x04:'wrist',
        0x97:'rotating_base',
    }
    robot = {'count':4,'names':motors_name}
    cfg = {'windowSize':[160,120]}
    square = {"x":20.32 /2 , "y":27.94 / 2, "s":10}
    app = P2_Blue_App(robot = robot,cfg=cfg,square = square)
    app.run()
