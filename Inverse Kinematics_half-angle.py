# inverse kinematics by half-angle tangent formula, i.e. Method 2

from numpy import *
from math import *

def theta_inv_M2(a2,a3,d2,d4,d6,nx,ny,nz,sx,sy,sz,ax,ay,az,px,py,pz):
#3.66
theta1_M2 = 2*atan2(-px+sqrt(pow(px,2)+pow(py,2)-pow(d2,2)), d2+py)
#3.73
g214_M2 = cos(theta1_M2)*px+sin(theta1_M2)*py
g224_M2 = -pz

d_M2 = pow(g214_M2,2) + pow(g224_M2,2) - pow(d4,2) - pow(a3,2) - pow(a2,2)
e_sqr_M2 = 4*pow(a2,2)*pow(a3,2) + 4*pow(a2,2)*pow(d4,2)
d_sqr_M2 = pow(d_M2,2)

theta3_M2 = 2*atan2(2*a2*d4+sqrt(e_sqr_M2-d_sqr_M2), d_M2+2*a2*a3)
#3.78
f_M2 = f
h_sqr_M2 = h_sqr
f_sqr_M2 = f_sqr

theta2_M2 = 2*atan2(d4*cos(theta3_M2)-a3*sin(theta3_M2)+sqrt(h_sqr_M2-f_sqr_M2), f_M2+d4*sin(theta3_M2)+a3*cos(theta3_M2)+a2)

#f = g214 - a2*cos(theta3_M2)
#h_sqr = pow(d4,2) + pow(a3,2)
#theta2_M2 = 2*atan2(d4*cos(theta3_M2)-a3*sin(theta3_M2)+sqrt(pow(d4,2)+pow(a3,2)-pow(f,2)), f+(d4*sin(theta3_M2)+a3*cos(theta3_M2)))

#3.81
theta4_M2 = atan2(-sin(theta1_M2)*a1+cos(theta_M2)*ay, cos(theta2_M2+theta3_M2)*(cos(theta1_M2)*ax+sin(theta1_M2)*ay)-sin(theta2_M1+theta3_M2)*az )
#3.85
theta5_M2 = atan2(cos(theta4_M2)*(cos(theta2_M2+theta3_M2)*(cos(theta1_M2)*ax+sin(theta1_M2)*ay)-sin(theta2_M2+theta3_M2)*az )+sin(theta4_M2)*(-sin(theta1_M2)*ax+cos(theta1_M2)*ay), sin(theta2_M2+theta3_M2)*(cos(theta1_M2)*ax+sin(theta1_M2)*ay )+cos(theta2_M2+theta3_M2)*az)
#3.87
theta6_M2 = atan2(-sin(theta4_M2)*(cos(theta2_M2+theta3_M2)*(cos(theta1_M2)*nx+sin(theta1_M2)*ny)-sin(theta2_M2+theta3_M2)*nz)+cos(theta4_M2)*(-sin(theta1_M2)*nx+cos(theta1_M2)*ny), -sin(theta4_M2)*(cos(theta2_M2+theta3_M2)*(cos(theta1_M2)*sx+sin(theta1_M2)*sy)-sin(theta2_M2+theta3_M2)*sz)+cos(theta4_M2)*(-sin(theta1_M2)*sx+cos(theta1_M2)*sy))

return theta1_M2, theta2_M2, theta3_M2, theta4_M2, theta5_M2, theta6_M2
