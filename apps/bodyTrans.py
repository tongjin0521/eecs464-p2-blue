close('all')
### Generate some data
Base_Shape = asfarray([(0,1), (0, -1), (1,0)])
   
BT = Base_Shape.T
plot(BT[0], BT[1], 'bx', markersize=8)
axis([-2, 2, -2, 2])
##### Shift it through an affine map to create 'measured' data
#Rotation
theta = pi/4.0;

Q_true =  asfarray([(cos(theta), -sin(theta)), (sin(theta), cos(theta))])

#translation
b = array([1.0,1.0])


Aff_shape = dot(Q_true, BT) + b[:,newaxis]*ones(shape(BT)[1])
plot(Aff_shape[0], Aff_shape[1], 'ko', markersize=8)
title("raw data")
grid('on')


#### Corrrection Method
#compute centroid
ac = sum(Aff_shape, axis = 1)/shape(Aff_shape)[1]
bc = sum(BT, axis = 1)/shape(BT)[1]

#Translate to Origin
Atil = Aff_shape - ac[:,newaxis]
Btil = BT - bc[:,newaxis]

figure()
plot(Atil[0], Atil[1],'ko', Btil[0], Btil[1], 'kx', markersize=8)
axis([-2,2,-2,2])
title("centered data")
grid('on')


d,Z, trans = procrustes(Btil, Atil)
M = trans.get('rotation')
#Rotated Data = dot(Atil, M)
plot(Z[0], Z[1], 'ro', markersize=10, alpha=0.2) 

print "x's mark base shape - red circles with alpha  = 0.2 is output of transform"
