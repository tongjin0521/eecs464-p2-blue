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
import math

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
        self.num_its_per_step = 7

    def curr_tool_pos(self):
        theta_0 = self.curr_angles[0,0]
        theta_2 = self.curr_angles[1,0]
        theta_t = self.curr_angles[2,0] - np.pi/2 + theta_2
        p_x = (self.l2 * np.sin(theta_2) + self.l3 * np.cos(theta_t)) * np.cos(theta_0) - self.l4 * np.sin(theta_0)
        p_y = (self.l2 * np.sin(theta_2) + self.l3 * np.cos(theta_t)) * np.sin(theta_0) + self.l4 * np.cos(theta_0)
        p_z = self.l1 + self.l2 * np.cos(theta_2) - self.l3 * np.sin(theta_t)
        # progress(np.array([[p_x],[p_y],[p_z]]))
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
        # progress("--------")
        # progress(self.curr_angles /np.pi * 180)
        self.app.arm[0].set_pos( int(self.curr_angles[0] / np.pi * 180 * 100) -3500)
        self.app.arm[1].set_pos( -int(self.curr_angles[1] / np.pi * 180 * 100) - 4500)
        self.app.arm[2].set_pos( int(self.curr_angles[2] / np.pi * 180 * 100) )

    def behavior(self):
        self.curr_angles = np.array([[(self.app.arm[0].get_pos() +3500) % 36000 / 18000 * np.pi],[-(self.app.arm[1].get_pos()+ 4500) % 36000 / 18000 * np.pi],[self.app.arm[2].get_pos() % 36000 / 18000 * np.pi]])
        # progress("----behaviour----")
        # progress(self.curr_angles /np.pi * 180)
        err_ending = 1
        step_size = 0.01
        it_num = 0
        while (True):
            # progress(str(it_num))
            d_point = self.goal_point - self.curr_tool_pos()
            if (np.linalg.norm(d_point) < err_ending):
                # # print_d_angles = d_angles/ np.pi * 180
                # print_curr_angles = self.curr_angles / np.pi * 180
                # # progress("d_angles: " + str(print_d_angles[0,0]) +", "+ str(print_d_angles[1,0]) +", "+ str(print_d_angles[2,0]))
                # progress("curr_angles: " + str(print_curr_angles[0,0]) +", "+ str(print_curr_angles[1,0]) +", "+ str(print_curr_angles[2,0]))
                self.set_motor_pos()
                break
            jacobian = self.get_jacobian()
            d_angles = np.linalg.pinv(jacobian) @ d_point
            # if ((self.curr_angles[2,0] >= 100 /180 * np.pi and d_angles[2,0] >0 ) or (self.curr_angles[2,0] <= -100 /180 * np.pi and d_angles[2,0] <0 )):
            #     d_angles[2,0] = 0
            #     progress("WARNING: ANGLE LIMIT FOR MOTOR 2")
            if (np.linalg.norm(d_angles) > step_size):
                d_angles = d_angles / np.linalg.norm(d_angles) * step_size
            d_angles[np.abs(d_angles) < 5e-4] = 0
            self.curr_angles += d_angles
            if it_num % self.num_its_per_step == 0:
                # progress("setting")
                # print(it_num)
                # progress("----------------")
                # progress("current pos: " + str(self.curr_tool_pos()[0,0]) +", "+ str(self.curr_tool_pos()[1,0]) +", "+ str(self.curr_tool_pos()[2,0]))
                # print_d_angles = d_angles/ np.pi * 180
                # print_curr_angles = self.curr_angles / np.pi * 180
                # progress("d_angles: " + str(print_d_angles[0,0]) +", "+ str(print_d_angles[1,0]) +", "+ str(print_d_angles[2,0]))
                # progress("curr_angles: " + str(print_curr_angles[0,0]) +", "+ str(print_curr_angles[1,0]) +", "+ str(print_curr_angles[2,0]))
                self.set_motor_pos()
            it_num +=1
            yield self.forDuration(0.1)
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
                progress(point_i)
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
        
        ###
        ### Arm specification
        ###
        self.l1 = 27.5
        self.l2 = 24.7
        self.l3 = 41.5
        self.l4 = 8  
        self.s = 26.5

        self.cali_num_points_per_line = 4
        self.draw_num_points_per_line = 10.0
        self.draw_zOffset = 0

        self.moveP = MoveToPoint(self)
        self.drawP = DrawSquare(self)
        self.square_param = square
        self.cali_angles = []
        self.cali_pos = []
        self.min_x = 1000
        self.max_x = -1000
        self.min_y = 1000
        self.max_y = -1000
        self.min_z = 1000
        self.max_z = -1000
        self.rotating_base_fixed = False
        self.arm = [self.robot.at.shoulder, self.robot.at.elbow, self.robot.at.wrist, self.robot.at.rotating_base]
        for motor_i in self.arm:
            motor_i.set_speed(2)
            motor_i.set_mode(2)
        # self.reset_motors()

    def discretize_square(self,x, y, s, recorded_pts):
        left_upper_p,right_lower_p = self.cal_corner_pts(recorded_pts)
        self.target_square = []
        top_line = []
        right_line = []
        bottom_line = []
        left_line = []
        num_points = 10


        # TODO: x,y,z offest for different paper orientations & waypoint
        x_offset = 0
        y_offset = 0
        z_offset = 0
        waypt_x_offset = -5
        waypt_y_offset = 0
        waypt_z_offset = 0

        total_x = 21.5
        total_y = 28.0
        total_z = total_x

        delta_x = (left_upper_p[0,0]-right_lower_p[0,0])/total_x
        delta_y = (left_upper_p[1,0]-right_lower_p[1,0])/total_y
        delta_z = (left_upper_p[2,0]-right_lower_p[2,0])/total_z

        center_x = (left_upper_p[0,0] + right_lower_p[0,0])/2
        center_y = (left_upper_p[1,0] + right_lower_p[1,0])/2
        center_z = (left_upper_p[2,0] + right_lower_p[2,0])/2

        for i in range(0, int(num_points+1)):
            top_line.append(np.array([[center_x+(s+y)*delta_x + x_offset], [center_y+(s-x)*delta_y-i*2*s*delta_y/num_points+y_offset], [center_z + (s+y)*delta_z+z_offset]]))
            right_line.append(np.array([[center_x+(s+y)*delta_x-i*2*s*delta_x/num_points+ x_offset], [center_y+(-s-x)*delta_y+y_offset],  [center_z + (s+y)*delta_z - i*2*s*delta_z/num_points+z_offset]]))
            bottom_line.append(np.array([[center_x+(-s+y)*delta_x+ x_offset], [center_y+(-s-x)*delta_y+i*2*s*delta_y/num_points+y_offset],  [center_z + (-s+y)*delta_z+z_offset]]))
            left_line.append(np.array([[center_x+(-s+y)*delta_x+i*2*s*delta_x/num_points+ x_offset], [center_y+(s-x)*delta_y+y_offset], [center_z + (-s+y)*delta_z + i*2*s*delta_z/num_points+z_offset]]))

        self.target_square.append([np.array([[center_x+s*delta_x +waypt_x_offset  + x_offset ], [center_y+s*delta_y-i*2*s*delta_y/num_points+waypt_y_offset  + y_offset], [center_z + s*delta_z+waypt_z_offset  + z_offset]])])
        self.target_square.append(top_line)
        self.target_square.append(right_line)
        self.target_square.append(bottom_line)
        self.target_square.append(left_line)
        progress(self.target_square)

    def cali_pos_cal(self):
        theta_0 = self.cali_angles[-1][0]
        theta_2 = self.cali_angles[-1][1]
        theta_t = self.cali_angles[-1][2] - np.pi/2 + theta_2
        p_x = (self.l2 * np.sin(theta_2) + self.l3 * np.cos(theta_t)) * np.cos(theta_0) - self.l4 * np.sin(theta_0)
        p_y = (self.l2 * np.sin(theta_2) + self.l3 * np.cos(theta_t)) * np.sin(theta_0) + self.l4 * np.cos(theta_0)
        p_z = self.l1 + self.l2 * np.cos(theta_2) - self.l3 * np.sin(theta_t)

        if p_x < self.min_x:
            self.min_x = p_x
        if p_x > self.max_x:
            self.max_x = p_x
        if p_y < self.min_y:
            self.min_y = p_y
        if p_y > self.max_y:
            self.max_y = p_y
        if p_z < self.min_z:
            self.min_z = p_z
        if p_z > self.max_z:
            self.max_z = p_z
        self.cali_pos.append(np.array([[p_x],[p_y],[p_z]]))

    def cal_corner_pts(self,recorded_pts):
        num_cali_pts_per_line = 4
        recorded_pts = np.array(recorded_pts).reshape(num_cali_pts_per_line * 4,3)
        left_upper_p = np.array([[-1.],[-1.],[-1.]])
        right_lower_p =  np.array([[-1.],[-1.],[-1.]])
        left_upper_p[0,0] = np.mean(recorded_pts[:num_cali_pts_per_line+1,0])
        left_upper_p[1,0] = (np.mean(recorded_pts[-num_cali_pts_per_line:,1]) * num_cali_pts_per_line + recorded_pts[0,1])/(num_cali_pts_per_line +1)
        left_upper_p[2,0] = np.mean(recorded_pts[:num_cali_pts_per_line+1,2])
        right_lower_p[0,0] = np.mean(recorded_pts[2*num_cali_pts_per_line:3*num_cali_pts_per_line+1,0])
        right_lower_p[1,0] = np.mean(recorded_pts[num_cali_pts_per_line:2*num_cali_pts_per_line+1,1])
        right_lower_p[2,0] = np.mean(recorded_pts[2*num_cali_pts_per_line:3*num_cali_pts_per_line+1,2])
        print(left_upper_p)
        print(right_lower_p)
        return left_upper_p,right_lower_p

    def reset_motors(self, no_rotating_base = False):
        self.arm[0].set_pos(-3500)
        self.arm[1].set_pos(-4500 - 3000)
        self.arm[2].set_pos(3000)
        if not no_rotating_base:
            self.arm[3].set_pos(0)
        return

    def onStart(self):
        pass

    def onEvent(self,evt):
        ## disable this block (change to 0) to use on_K for these keys
        if evt.type == KEYDOWN: 
            # progress("------")
            # for motor_i in self.arm:
            #     progress(str(motor_i.get_pos()))
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
                # progress(pre + str(self.app.arm[0].get_pos() / 18000) + " "+ str(self.app.arm[1].get_pos() / 18000) + " "+ str(self.app.arm[2].get_pos() / 18000))
                self.cali_angles.append(np.array([(self.arm[0].get_pos() +3500) % 36000 / 18000 * np.pi,-(self.arm[1].get_pos()+ 4500) % 36000 / 18000 * np.pi,self.arm[2].get_pos() % 36000 / 18000 * np.pi]))
                self.cali_pos_cal()
                progress(pre + "Recording point: " + str(self.cali_pos[-1][0]) +" " +str(self.cali_pos[-1][1]) +" " + str(self.cali_pos[-1][2]) +" ")
                return
            if evt.key == K_s:
                # set rotating base angle
                progress(pre + "Rotating base angle set~")
                self.arm[3].set_pos(self.arm[3].get_pos())
                self.rotating_base_fixed = True
                return
            if evt.key == K_d:
                # input(pre + "Press Enter when you put the robotic arm in ready pose and ready to draw")
                progress(pre + "Drawing a square~")
                square_x = self.square_param['x']
                square_y = self.square_param['y']
                square_s = self.square_param['s']
                # progress("x: "+str(square_x)+" y: "+str(square_y)+ " z: "+ str(square_s))
                self.discretize_square(square_x,square_y,square_s,self.cali_pos) # set self.target_square
                self.drawP.start()
                return 
            if evt.key == K_p:
                goal_point = np.array([[45],[8],[40]])
                progress(pre + "Going to point: " + "x: "+str(goal_point[0,0])+" y: "+str(goal_point[1,0])+ " z: "+ str(goal_point[2,0]))
                self.moveP.goal_point = goal_point
                self.moveP.start()
                return
            if evt.key == K_o:
                self.reset_motors(no_rotating_base=True)
            return JoyApp.onEvent(self,evt)
        return
    
if __name__=="__main__":
    motors_name = {
        0x0C:'shoulder',
        0x28:'elbow',
        0x4A:'wrist',
        0x97:'rotating_base',
    }
    robot = {'count':4,'names':motors_name}
    cfg = {'windowSize':[160,120]}
    square = {"x":0 , "y":0, "s":6}
    app = P2_Blue_App(robot = robot,cfg=cfg,square = square)
    app.run()
