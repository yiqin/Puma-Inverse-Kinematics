## ECE/CS Introducation to Robotics Systems
#   Project 1
#   Due : Tuesday, October 16, 2012
#   Description:  Write a python or C++ program to run in OpenRave to verify the direct
#                         and inverse kinamatics of the PUMA 560 robot shown in Figure 2.11
#                         of your textbook.
#
#   Modules: math - basis matrix operation
#                   numpy - matrix operation
#                   config_IK - global variables
#                   DH_IK - DH transformation matrix
#
#
#   Update Date: Monday, October 16, 2012


from numpy  import *
from math import *
from config_IK import *
from DH import *


# initial conditions
theta1 = 120*pi/180
theta2 = -35*pi/180
theta3 = 90*pi/180
theta4 = 10*pi/180
theta5 = 10*pi/180
theta6 = 10*pi/180



#theta1 = 160*pi/180
#theta2 = -45*pi/180
#theta3 = 90*pi/180
#theta4 = 110*pi/180
#theta5 = 80*pi/180
#theta6 = 10*pi/180


# radius to degree
change = 180/pi

# forward kinematics state vector
q = array([[change*theta1],[change*theta2],[change*theta3],[change*theta4],[change*theta5],[change*theta6]])

## Forward Kinematics
# homogeneous transformation matrix
A01 = DH(theta1, alpha1, a1, d1)
A12 = DH(theta2, alpha2, a2, d2)
A23 = DH(theta3, alpha3, a3, d3)
A34 = DH(theta4, alpha4, a4, d4)
A45 = DH(theta5, alpha5, a5, d5)
A56 = DH(theta6, alpha6, a6, d6)

T1 = A01
T2 = dot(T1,A12)
T3 = dot(T2,A23)
T4 = dot(T3,A34)
T5 = dot(T4,A45)
T6 = dot(T5,A56)

# obtain z3
z3x = T3[0,2]
z3y = T3[1,2]
z3z = T3[2,2]
z3 = array([z3x,z3y,z3z])

# obtain z4
z4x = T4[0,2]
z4y = T4[1,2]
z4z = T4[2,2]
z4 = array([z4x,z4y,z4z])

# obtain y5
y5x = T5[0,1]
y5y = T5[1,1]
y5z = T5[2,1]
y5 = array([y5x,y5y,y5z])

# vectors of hand
# different from matlab.
nx = T6[0,0]
ny = T6[1,0]
nz = T6[2,0]
n = array([nx,ny,nz])

sx = T6[0,1]
sy = T6[1,1]
sz = T6[2,1]
s = array([sx,sy,sz])

ax = T6[0,2]
ay = T6[1,2]
az = T6[2,2]
a = array([ax,ay,az])

px = T6[0,3]
py = T6[1,3]
pz = T6[2,3]
p = array([[px],[py],[pz]])

##A geometric approach to solving the inverse kinematics problem
#Values of the indicators

ARM = sign(-d4*sin(theta2+theta3)-a3*cos(theta2+theta3)-a2*cos(theta2))
ELBOW = ARM*sign(d4*cos(theta3)-a3*sin(theta3))

temp_W1 = dot(s,z4)
temp_W2 = dot(n,z4)
WRIST = sign(temp_W1)
if abs(temp_W1)< 0.0000001:
    WRIST = sign(temp_W2)


print ARM
print ELBOW
print WRIST


#3.103, 3.104, 3.105
S1_M3 = (-ARM*py*sqrt(pow(px,2)+pow(py,2)-pow(d2,2))-px*d2)/(pow(px,2)+pow(py,2))
C1_M3 = (-ARM*px*sqrt(pow(px,2)+pow(py,2)-pow(d2,2))+py*d2)/(pow(px,2)+pow(py,2))
theta1_M3 = atan2(S1_M3, C1_M3)
#3.107, 3.108, 3.109, 3.110, 3.111, 3.112, 3.113, 3.114
R_M3 = sqrt(pow(px,2)+pow(py,2)+pow(pz,2)-pow(d2,2))
r_M3 = sqrt(pow(px,2)+pow(py,2)-pow(d2,2))
Sa_M3 = -pz/R_M3
Ca_M3 = - ARM*r_M3/R_M3
Cb_M3 = (pow(px,2)+pow(py,2)+pow(pz,2)+pow(a2,2)-pow(d2,2)-(pow(d4,2)+pow(a3,2)))/(2*a2*sqrt(pow(px,2)+pow(py,2)+pow(pz,2)-pow(d2,2)))
Sb_M3 = sqrt(1-pow(Cb_M3,2))
S2_M3 = Sa_M3*Cb_M3+(ARM*ELBOW)*Ca_M3*Sb_M3
C2_M3 = Ca_M3*Cb_M3-(ARM*ELBOW)*Sa_M3*Sb_M3
theta2_M3 = atan2(S2_M3, C2_M3)
#3.116, 3.117, 3.118, 3.119, 3.120, 3.121, 3.122
Cc_M3 = (pow(a2,2)+(pow(d4,2)+pow(a3,2))-pow(R_M3,2))/(2*a2*sqrt(pow(d4,2)+pow(a3,2)))
Sc_M3 = ARM*ELBOW*sqrt(1-pow(Cc_M3,2))
Sd_M3 = d4/sqrt(pow(d4,2)+pow(a3,2))
Cd_M3 = abs(a3)/sqrt(pow(d4,2)+pow(a3,2))
S3_M3 = Sc_M3*Cd_M3-Cc_M3*Sd_M3
C3_M3 = Cc_M3*Cd_M3+Sc_M3*Sd_M3
theta3_M3 = atan2(S3_M3, C3_M3)


#3.130
## 3.126
temp = cross(z3,a)
temp_M1 = dot(s, temp)
temp_M2 = dot(n,temp)
M = WRIST*sign(temp_M1)
if abs(temp_M1)< 0.0001:
    M = WRIST*sign(temp_M2)
print M



temp41_M3 = M*(cos(theta1_M3)*ay-sin(theta1_M3)*ax)
temp42_M3 = M*(cos(theta1_M3)*cos(theta2_M3+theta3_M3)*ax+sin(theta1_M3)*cos(theta2_M3+theta3_M3)*ay-sin(theta2_M3+theta3_M3)*az)
theta4_M3 = atan2(temp41_M3, temp42_M3)
#3.132
temp51_M3 = (cos(theta1_M3)*cos(theta2_M3+theta3_M3)*cos(theta4_M3)-sin(theta1_M3)*sin(theta4_M3))*ax+(sin(theta1_M3)*cos(theta2_M3+theta3_M3)*cos(theta4_M3)+cos(theta1_M3)*sin(theta4_M3))*ay-cos(theta4_M3)*sin(theta2_M3+theta3_M3)*az
temp52_M3 = cos(theta1_M3)*sin(theta2_M3+theta3_M3)*ax+sin(theta1_M3)*sin(theta2_M3+theta3_M3)*ay+cos(theta2_M3+theta3_M3)*az
theta5_M3 = atan2(temp51_M3, temp52_M3)
#3.134
temp61_M3 = (-sin(theta1_M3)*cos(theta4_M3)-cos(theta1_M3)*cos(theta2_M3+theta3_M3)*sin(theta4_M3))*nx+(cos(theta1_M3)*cos(theta4_M3)-sin(theta1_M3)*cos(theta2_M3+theta3_M3)*sin(theta4_M3))*ny+(sin(theta4_M3)*sin(theta2_M3+theta3_M3))*nz
temp62_M3 = (-sin(theta1_M3)*cos(theta4_M3)-cos(theta1_M3)*cos(theta2_M3+theta3_M3)*sin(theta4_M3))*sx+(cos(theta1_M3)*cos(theta4_M3)-sin(theta1_M3)*cos(theta2_M3+theta3_M3)*sin(theta4_M3))*sy+(sin(theta4_M3)*sin(theta2_M3+theta3_M3))*sz
theta6_M3 = atan2(temp61_M3, temp62_M3)

if theta2_M3 > pi/2:
    theta2_M3 = theta2_M3 - 2*pi
if theta3_M3 < -pi/2:
    theta3_M3 = theta3_M3 + 2*pi

#  The resulting joint variables of applying the geometric approach, i.e. Method 3
q_M3 = array([[change*theta1_M3],[change*theta2_M3],[change*theta3_M3],[change*theta4_M3],[change*theta5_M3],[change*theta6_M3]])    

#  The results of the Method 1, the Method 2, and the Method 3
print q
print q_M3

