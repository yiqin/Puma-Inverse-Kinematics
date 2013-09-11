# Inverse Kinematics by Trigonometric Substitution, i.e. Method 1

from numpy import *
from math import *


def theta_inv_M1(a2,a3,d2,d4,d6,nx,ny,nz,sx,sy,sz,ax,ay,az,px,py,pz):
    theta1_M1 = atan2(py,px)-atan2(d2,sqrt(pow(px,2)+pow(py,2)-pow(d2,2)))

    g214 = cos(theta1_M1)*px+sin(theta1_M1)*py
    g224 = -pz

    d = pow(g214,2) + pow(g224,2) - pow(d4,2) - pow(a3,2) - pow(a2,2)
    e_sqr = 4*pow(a2,2)*pow(a3,2) + 4*pow(a2,2)*pow(d4,2)

    theta3_M1 = atan2(d,sqrt(e_sqr-pow(d,2))) - atan2(a3,d4)

    f = g214 - a2*cos(theta3_M1)
    h_sqr = pow(d4,2) + pow(a3,2)
    
    theta2_M1 = atan2(f, sqrt(h_sqr - pow(f,2)) ) - atan2( d4*sin(theta3_M1)+a3*cos(theta3_M1), d4*cos(theta3_M1) - a3*sin(theta3_M1) )
    
    
    theta4_M1 = atan2( -sin(theta1_M1)*a1+cos(theta_M1)*ay, cos(theta2_M1+theta3_M1)*(cos(theta1_M1)*ax+sin(theta1_M1)*ay)-sin(theta2_M1+theta3_M1)*az )

    theta5_M1 = atan2( cos(theta4_M1)*( cos(theta2_M1+theta3_M1)* (cos(theta1_M1)*ax+sin(theta1_M1)*ay)-sin(theta2_M1+theta3_M1)*az )  + sin(theta4_M1)*(-sin*(theta1_M1)*ax + cos(theta1_M1)*ay), sin(theta2_M1+theta3_M1)*( cos(theta1_M1)*ax+sin(theta1_M1)*ay )+cos(theta2_M1+theta3_M1)*az )

    theta6_M1 = atan2 ( -sin(theta4_M1)*( cos(theta2_M1+theta3_M1)*(cos(theta1_M1)*nx+sin(theta1_M1)*ny) - sin(theta2_M1+theta3_M1)*nz ) + cos(theta4_M1)*(-sin(theta1_M1)*nx+cos(theta1_M1)*ny), -sin(theta4_M1)*(cos(theta2_M1+theta3_M1)*(cos(theta1_M1)*sx+sin(theta1_M1)*sy)-sin(theta2_M1+theta3_M1)*sz)+cos(theta4_M1)*(-sin(theta1_M1)*sx+cos(theta1_M1)*sy))

return theta1_M1, theta2_M1, theta3_M1, theta4_M1, theta5_M1, theta6_M1
