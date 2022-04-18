from numpy import asarray
import numpy as np
import math


def discretize_square(x, y, s, left_upper_p, right_lower_p):
    pass

def cal_corner_pts(recorded_pts):
    recorded_pts = np.array(recorded_pts).reshape(8,3)
    left_upper_p = np.array([[-1.],[-1.],[-1.]])
    right_lower_p =  np.array([[-1.],[-1.],[-1.]])
    left_upper_p[0,0] = np.mean(recorded_pts[:3,0])
    left_upper_p[1,0] = (np.mean(recorded_pts[-2,1]) * 2 + recorded_pts[0,1])/3
    left_upper_p[2,0] = np.mean(recorded_pts[:3,2])
    right_lower_p[0,0] = np.mean(recorded_pts[4:7,0])
    right_lower_p[1,0] = np.mean(recorded_pts[2:5,1])
    right_lower_p[2,0] = np.mean(recorded_pts[4:7,2])
    print(left_upper_p)
    print(right_lower_p)
    return left_upper_p,right_lower_p

input =[np.array([[56.837089], [4.05496933],[34.52064423]] ),
np.array([[56.7680284], [-5.92928143] ,[34.88270637]] ),
np.array([[56.69476518], [-15.65090097], [34.05042843]] ),
np.array([[48.36731222], [-16.12965075],[20.51601835]] ),
np.array([[39.32407705], [-15.69578377] ,[12.57454386]]) ,
np.array([[40.41008652], [-5.92701639] ,[14.3572238]] ),
np.array([[38.53679163], [4.33298301] ,[12.91039786]] ),
np.array([[48.82450653], [4.38530711], [24.27425379]])]

# input_flat =[np.array([60.7929216] [3.77904436] [13.06420982] ),
#  np.array([60.75118511] [-5.77173875] [11.7582237] ),
#  np.array([60.80883501] [-15.34327907] [12.4045296] ),
#  np.array([48.39483061] [-13.49160667] [7.24740825] ),
# np.array([35.10476328] [-12.66006138] [7.92915589] ),
# np.array([34.05238176] [-4.75258677] [9.70908476] ),
# np.array([33.94046446] [4.3657943] [9.47822102] ),
# np.array( [47.40162042] [4.56522962] [9.19554764] )]

# [51.7731837] [7.41314149] [39.33712406] 
# [52.47199006] [-2.80777957] [40.42586854] 
# [51.53216343] [-12.90451412] [43.58888152] 
# [52.70805305] [-14.20486205] [29.39390984] 
#  [52.41910262] [-14.53000485] [16.32140472] 
#  [52.17360664] [-5.35130678] [14.06348087] 
# [51.68851935] [6.22682478] [13.44696394] 
# [51.40509671] [7.41731749] [22.67923076] 



left_upper_p, right_lower_p = cal_corner_pts(input)

# target_square = discretize_square(0 , 0, 10.5,left_upper_p,right_lower_p)
# print(target_square)