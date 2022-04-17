from numpy import asarray
import numpy as np
import math


def calculate_Tp2w(cali_pos,min_x,min_y,min_z,max_x,max_y,max_z):
        # do fit
        tmp_A = []
        tmp_b = []
        for i in range(len(cali_pos)):
            tmp_A.append([cali_pos[i][0], cali_pos[i][1], 1])
            tmp_b.append(cali_pos[i][2])
        b = np.matrix(tmp_b,dtype='float')
        A = np.matrix(tmp_A,dtype='float')
        print(A.shape)
        print(b.shape)
        fit = (A.T * A).I * A.T * b # fit[0] x + fit[1] y + fit[2] = z
        normal_x = -fit[0]
        normal_y = -fit[1]
        normal_z = 1
        angle_rotate_about_y = -np.arctan(normal_z/math.sqrt(normal_x**2 + normal_y**2)) + 0.5*np.pi
        angle_rotate_about_z = np.arctan(normal_y/normal_x)

        Tp2ws_z = asarray([
            [np.cos(angle_rotate_about_z), -np.sin(angle_rotate_about_z), 0, 0],
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
        # change in position
        # Tp2ws[2,3] = 33/2*np.cos(angle_rotate_about_y) # move in z direction
        # arm_proj = self.l2 * np.cos(self.cali_angles[1] * motor_1_polarity) + self.l3 * np.cos(-angle_rotate_about_y)
        # Tp2ws[1,3] = 33/2 + arm_proj * np.sin(angle_rotate_about_z) # move in y direction
        # Tp2ws[0,3] = arm_proj * np.cos(angle_rotate_about_z) - self.s # move in x direction
        base_x = (min_x + max_x)*0.5
        base_y = (min_y + max_y)*0.5
        base_z = (min_z + max_z)*0.5
        Tp2ws_trans = asarray([
            [1, 0, 0, base_x],
            [0, 1, 0, base_y],
            [0, 0, 1, base_z],
            [0, 0, 0, 1]
        ])

        Tp2ws = Tp2ws_trans @ Tp2ws_y @ Tp2ws_z

        return Tp2ws

def discretize_square(x, y, s, Tp2w):
    target_square = []
    top_line = []
    right_line = []
    bottom_line = []
    left_line = []
    num_points = 10
    zOffset = 0

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
        top_line_t.append(np.array([[(p @ Tp2w.T)[0]],[(p @ Tp2w.T)[1]],[(p @ Tp2w.T)[2]]]).reshape(3,1))
    for p in bottom_line:
        bottom_line_t.append(np.array([[(p @ Tp2w.T)[0]],[(p @ Tp2w.T)[1]],[(p @ Tp2w.T)[2]]]).reshape(3,1))
    for p in left_line:
        left_line_t.append(np.array([[(p @ Tp2w.T)[0]],[(p @ Tp2w.T)[1]],[(p @ Tp2w.T)[2]]]).reshape(3,1))
    for p in right_line:
        right_line_t.append(np.array([[(p @ Tp2w.T)[0]],[(p @ Tp2w.T)[1]],[(p @ Tp2w.T)[2]]]).reshape(3,1))

    zOffset = 3
    z_offset_p = [x-s, y+s, zOffset, 1]
    # target_square.append([np.array([[(z_offset_p @ Tp2w.T)[0]],[(z_offset_p @ Tp2w.T)[1]],[(z_offset_p @ Tp2w.T)[2]]]).reshape(3,1)])
    target_square.append(top_line_t)
    target_square.append(right_line_t)
    target_square.append(bottom_line_t)
    target_square.append(left_line_t)
    return target_square


input =[np.array([[56.837089], [4.05496933],[34.52064423]] ),
np.array([[56.7680284], [-5.92928143] ,[34.88270637]] ),
np.array([[56.69476518], [-15.65090097], [34.05042843]] ),
np.array([[48.36731222], [-16.12965075],[20.51601835]] ),
np.array([[39.32407705], [-15.69578377] ,[12.57454386]]) ,
np.array([[40.41008652], [-5.92701639] ,[14.3572238]] ),
np.array([[38.53679163], [4.33298301] ,[12.91039786]] ),
np.array([[48.82450653], [4.38530711], [24.27425379]])]

tp2w = calculate_Tp2w(input, 39.32407705,-16.12965075,12.91039786,56.837089,4.38530711,34.88270637)
print(tp2w)
target_square = discretize_square(0 , 0, 10.5,tp2w)
print(target_square)