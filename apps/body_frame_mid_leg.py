#centers the body of the BIRDS mechapod
#to the mid leg as the y-axis with the orgin
#at the center of the L2 module
#
# 7/9/2015
# Chad Schaffer
# cmschaf@umich.edu

import pylab




def calcDist(x0, y0, x1, y1, x2, y2):
    
    dist = abs((y2 - y1)* x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1)/ pylab.sqrt((y2 - y1)**2 + (x2 - x1)**2)
    

    return dist



if __name__ == "__main__":
 
    x = pylab.loadtxt('/home/your-grace/Birds/twist_test_TSV/2000twist_40spd_1_pos.tsv', delimiter='\t',skiprows=12)

    x.shape = (x.shape[0], x.shape[1]/3, 3)

    mid_leg_x = pylab.np.array([x[...,6,0], x[...,7,0], x[...,15,0], x[...,15,0], x[...,16,0], x[...,17,0], x[...,23,0], x[...,29,0], x[...,30,0], x[...,31,0], x[...,32,0], x[...,38,0]])

    mid_leg_y = pylab.np.array([x[...,6,1], x[...,7,1], x[...,15,1], x[...,15,1], x[...,16,1], x[...,17,1], x[...,23,1], x[...,29,1], x[...,30,1], x[...,31,1], x[...,32,1], x[...,38,1]])

    right_leg = pylab.np.array([x[...,29,0:2], x[...,30,0:2], x[...,31,0:2], x[...,32,0:2]])
    left_leg = pylab.np.array([x[...,14,0:2], x[...,15,0:2], x[...,16,0:2], x[...,17,0:2]])


    right_point = pylab.np.empty([right_leg.shape[1], 2])
    left_point = pylab.np.empty([left_leg.shape[1], 2])

    print left_point.shape

    for n in xrange(right_point.shape[0]):
        right_point[n][0] = pylab.mean(right_leg[..., n, 0])
        right_point[n][1] = pylab.mean(right_leg[..., n, 1])
        left_point[n][0] = pylab.mean(left_leg[..., n, 0])
        left_point[n][1] = pylab.mean(left_leg[..., n, 1])


    framed_x = pylab.empty_like(x)

    for m in xrange(framed_x.shape[0]):
        for n in xrange(framed_x.shape[1]):
                        framed_x[m, n, 0] = calcDist(x[m, n, 0], x[m, n, 1], right_point[m][0], right_point[m][1], left_point[m][0], left_point[m][1])
                        framed_x[m, n, 1] = calcDist(x[m, n, 0], x[m, n, 1], x[m,6,0], x[m,6,1], x[m,7,0], x[m,7,1])

    

    pylab.plot(framed_x[..., ...,1])
    #pylab.plot(x[...,0,0])

    #pylab.plot(x_axis)

    pylab.show()


