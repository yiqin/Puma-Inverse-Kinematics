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
#
#
#
#   Latest Upadte: Tuesday, September 10, 2013
#   What's new:
#                         SHOULDER indicator is not correct. Dean's solution -ARM2 is correct.
#                         This indicator is used in picking up theta2. In the following definition, ARM2 is assign to "sign1"
#                         sign1  is defined in line 276
#
#   Methond: trangent half-angle formula solution
#

from numpy  import *
from math import *
from config_IK import *
from DH import *

# initial conditions
theta1 = -120*pi/180
theta2 = 0*pi/180
theta3 = 90*pi/180
theta4 = 10*pi/180
theta5 = 0*pi/180
theta6 = 10*pi/180

delta_theta1 = 5*pi/180
delta_theta2 = 5*pi/180
delta_theta3= 5*pi/180

#theta1 = 160*pi/180
#theta2 = -45*pi/180
#theta3 = 90*pi/180
#theta4 = 110*pi/180
#theta5 = 80*pi/180
#theta6 = 260*pi/180

for i in range(-35,35):

    for j in range(-35,35):

        for k in range(-35,35):


            theta1 = delta_theta1*i
            theta2 = delta_theta2*j
            theta3 = delta_theta3*k
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

            # obtain z1
            z1x = T1[0,2]
            z1y = T1[1,2]
            z1z = T1[2,2]
            z1 = array([z1x,z1y,z1z])
            # obtain x1
            x1x = T1[0,0]
            x1y = T1[1,0]
            x1z = T1[2,0]
            x1 = array([x1x,x1y,x1z])

            #obtain z2
            z2x =  T2[0,2]
            z2y = T2[1,2]
            z2z = T2[2,2]
            z2 = array([z2x,z2y,z2z])
            # obtain x2
            x2x = T2[0,0]
            x2y = T2[1,0]
            x2z = T2[2,0]
            x2 = array([x2x,x2y,x2z])


            # obtain z3
            z3x = T3[0,2]
            z3y = T3[1,2]
            z3z = T3[2,2]
            z3 = array([z3x,z3y,z3z])
            # obtain x3
            x3x = T3[0,0]
            x3y = T3[1,0]
            x3z = T3[2,0]
            x3 = array([x3x,x3y,x3z])


            # obtain z4
            z4x = T4[0,2]
            z4y = T4[1,2]
            z4z = T4[2,2]
            z4 = array([z4x,z4y,z4z])
            # obtain x4
            x4x = T4[0,0]
            x4y = T4[1,0]
            x4z = T4[2,0]
            x4 = array([x4x,x4y,x4z])

            # obtain y5
            y5x = T5[0,1]
            y5y = T5[1,1]
            y5z = T5[2,1]
            y5 = array([y5x,y5y,y5z])
            # obtain x4
            x5x = T5[0,0]
            x5y = T5[1,0]
            x5z = T5[2,0]
            x5 = array([x5x,x5y,x5z])

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

            SHOULDER = sign(dot(z1,cross(x1,x2)))
            # Dean solution
            ARM2 = sign( cos(theta2)*(d4*cos(theta3)-a3*sin(theta3))-sin(theta2)*(d4*sin(theta3)+a3*cos(theta3)+a2) )
            
            ##if dot(z1,cross(x1,x2)) == 0:
                
            #print "SHOULDER = "
            #print SHOULDER

            temp_W1 = dot(s,z4)
            temp_W2 = dot(n,z4)
            WRIST = sign(temp_W1)
            if abs(temp_W1)< 0.0000001:
                WRIST = sign(temp_W2)


            #print "ARM = "
            #print ARM
            #print "ELBOW = "
            #print ELBOW
            #print "WRIST = "
            #print WRIST

            #3.130
            ## 3.126
            temp = cross(z3,a)
            temp_M1 = dot(s, temp)
            temp_M2 = dot(n,temp)
            M = WRIST*sign(temp_M1)
            if abs(temp_M1)< 0.0001:
                M = WRIST*sign(temp_M2)
            #print "M = "
            #print M

            ##Half-angle tangent formula, i.e. Method 2
            #  3.66
            ##
            theta1_M2A = 2*atan2(-px+sqrt(pow(px,2)+pow(py,2)-pow(d2,2)), d2+py)
            theta1_M2B = 2*atan2(-px-sqrt(pow(px,2)+pow(py,2)-pow(d2,2)), d2+py)
            # -ARM
            theta1_M2 = 2*atan2(-px-ARM*sqrt(pow(px,2)+pow(py,2)-pow(d2,2)), d2+py)

            if theta1_M2A > pi:
                theta1_M2A = theta1_M2A - 2*pi
            if theta1_M2A < -pi:
                theta1_M2A = theta1_M2A + 2*pi
            if theta1_M2B > pi:
                theta1_M2B = theta1_M2B - 2*pi
            if theta1_M2B < -pi:
                theta1_M2B = theta1_M2B + 2*pi
            if theta1_M2 > pi:
                theta1_M2 = theta1_M2 - 2*pi
            if theta1_M2 < -pi:
                theta1_M2 = theta1_M2 + 2*pi
            #print "theta1_M2A = "
            #print theta1_M2A*change
            #print "theta1_M2B = "
            #print theta1_M2B*change
            #print "theta1_M2 = "
            #print theta1_M2*change


            #  3.73
            g214_M2 = cos(theta1_M2)*px+sin(theta1_M2)*py
            g224_M2 = -pz
            d_M2 = pow(g214_M2,2) + pow(g224_M2,2) - pow(d4,2) - pow(a3,2) - pow(a2,2)
            e_sqr_M2 = 4*pow(a2,2)*pow(a3,2) + 4*pow(a2,2)*pow(d4,2)
            d_sqr_M2 = pow(d_M2,2)
            theta3_M2A = 2*atan2(2*a2*d4+sqrt(e_sqr_M2-d_sqr_M2), d_M2+2*a2*a3)
            theta3_M2B = 2*atan2(2*a2*d4-sqrt(e_sqr_M2-d_sqr_M2), d_M2+2*a2*a3)
            # -ARM*ELBOW
            theta3_M2 = 2*atan2(2*a2*d4-ARM*ELBOW*sqrt(e_sqr_M2-d_sqr_M2), d_M2+2*a2*a3)

            if theta3_M2A > pi:
                theta3_M2A = theta3_M2A - 2*pi
            if theta3_M2A < -pi:
                theta3_M2A = theta3_M2A + 2*pi
            if theta3_M2B > pi:
                theta3_M2B = theta3_M2B - 2*pi
            if theta3_M2B < -pi:
                theta3_M2B = theta3_M2B + 2*pi
            if theta3_M2 > pi:
                theta3_M2 = theta3_M2 - 2*pi
            if theta3_M2 < -pi:
                theta3_M2 = theta3_M2 + 2*pi
            #print "theta3_M2A"
            #print theta3_M2A*change
            #print "theta3_M2B = "
            #print theta3_M2B*change
            #print "theta3_M2 = "
            #print theta3_M2*change


            #  3.78
            g214 = cos(theta1_M2)*px+sin(theta1_M2)*py
            f = g214
            f_sqr = pow(g214,2)
            f_M2 = f
            h_sqr = pow(d4,2) + pow(a3,2) + pow(a2,2) + 2*a2*d4*sin(theta3_M2) + 2*a2*a3*cos(theta3_M2)
            h_sqr_M2 = h_sqr
            f_sqr_M2 = f_sqr
            theta2_M2A = 2*atan2(d4*cos(theta3_M2)-a3*sin(theta3_M2)+sqrt(h_sqr_M2-f_sqr_M2), f_M2+d4*sin(theta3_M2)+a3*cos(theta3_M2)+a2)
            theta2_M2B = 2*atan2(d4*cos(theta3_M2)-a3*sin(theta3_M2)-sqrt(h_sqr_M2-f_sqr_M2), f_M2+d4*sin(theta3_M2)+a3*cos(theta3_M2)+a2)
            # -ELBOW
            sign1= -SHOULDER
            if dot(z1,cross(x1,x2)) == 0:
                sign1 = ARM*ELBOW

            sign1 = ARM2
            
            theta2_M2 = 2*atan2(d4*cos(theta3_M2)-a3*sin(theta3_M2)-sign1*sqrt(h_sqr_M2-f_sqr_M2), f_M2+d4*sin(theta3_M2)+a3*cos(theta3_M2)+a2)

            if theta2_M2A > pi:
                theta2_M2A = theta2_M2A - 2*pi
            if theta2_M2A < -pi:
                theta2_M2A = theta2_M2A + 2*pi
            if theta2_M2B > pi:
                theta2_M2B = theta2_M2B - 2*pi
            if theta2_M2B < -pi:
                theta2_M2B = theta2_M2B + 2*pi
            if theta2_M2 > pi:
                theta2_M2 = theta2_M2 - 2*pi
            if theta2_M2 < -pi:
                theta2_M2 = theta2_M2 + 2*pi
            #print "theta2_M2A = "
            #print theta2_M2A*change
            #print "theta2_M2B = "
            #print theta2_M2B*change
            #print "theta2_M2 = "
            #print theta2_M2*change

                
            ## sign3
            #theta4_M2A = atan2(-sin(theta1_M2)*ax+cos(theta1_M2)*ay, cos(theta2_M2+theta3_M2)*(cos(theta1_M2)*ax+sin(theta1_M2)*ay)-sin(theta2_M2+theta3_M2)*az)
            #theta4_M2B = theta4_M2A + pi

            #theta4_M2 = theta4_M2A
            #  3.85
            #theta5_M2A = atan2(cos(theta4_M2A)*(cos(theta2_M2+theta3_M2)*(cos(theta1_M2)*ax+sin(theta1_M2)*ay)-sin(theta2_M2+theta3_M2)*az)+sin(theta4_M2A)*(-sin(theta1_M2)*ax+cos(theta1_M2)*ay), sin(theta2_M2+theta3_M2)*(cos(theta1_M2)*ax+sin(theta1_M2)*ay)+cos(theta2_M2+theta3_M2)*az)
            #theta5_M2B = atan2(cos(theta4_M2B)*(cos(theta2_M2+theta3_M2)*(cos(theta1_M2)*ax+sin(theta1_M2)*ay)-sin(theta2_M2+theta3_M2)*az)+sin(theta4_M2B)*(-sin(theta1_M2)*ax+cos(theta1_M2)*ay), sin(theta2_M2+theta3_M2)*(cos(theta1_M2)*ax+sin(theta1_M2)*ay)+cos(theta2_M2+theta3_M2)*az)

            #theta6_M2A = atan2(-sin(theta4_M2A)*(cos(theta2_M2+theta3_M2)*(cos(theta1_M2)*nx+sin(theta1_M2)*ny)-sin(theta2_M2+theta3_M2)*nz)+cos(theta4_M2A)*(-sin(theta1_M2)*nx+cos(theta1_M2)*ny), -sin(theta4_M2A)*(cos(theta2_M2+theta3_M2)*(cos(theta1_M2)*sx+sin(theta1_M2)*sy)-sin(theta2_M2+theta3_M2)*sz)+cos(theta4_M2A)*(-sin(theta1_M2)*sx+cos(theta1_M2)*sy))
            #theta6_M2B = atan2(-sin(theta4_M2B)*(cos(theta2_M2+theta3_M2)*(cos(theta1_M2)*nx+sin(theta1_M2)*ny)-sin(theta2_M2+theta3_M2)*nz)+cos(theta4_M2B)*(-sin(theta1_M2)*nx+cos(theta1_M2)*ny), -sin(theta4_M2B)*(cos(theta2_M2+theta3_M2)*(cos(theta1_M2)*sx+sin(theta1_M2)*sy)-sin(theta2_M2+theta3_M2)*sz)+cos(theta4_M2B)*(-sin(theta1_M2)*sx+cos(theta1_M2)*sy))

            #print "theta4_M2A = "
            #print theta4_M2A*change
            #print "theta5_M2A = "
            #print theta5_M2A*change
            #print "theta6_M2A = "
            #print theta6_M2A*change

            #print "theta4_M2B = "
            #print theta4_M2B*change
            #print "theta5_M2B = "
            #print theta5_M2B*change
            #print "theta6_M2B = "
            #print theta6_M2B*change

            theta4_M2 = atan2(M*(-sin(theta1_M2)*ax+cos(theta1_M2)*ay), M*(cos(theta2_M2+theta3_M2)*(cos(theta1_M2)*ax+sin(theta1_M2)*ay)-sin(theta2_M2+theta3_M2)*az))
            theta5_M2 = atan2(cos(theta4_M2)*(cos(theta2_M2+theta3_M2)*(cos(theta1_M2)*ax+sin(theta1_M2)*ay)-sin(theta2_M2+theta3_M2)*az)+sin(theta4_M2)*(-sin(theta1_M2)*ax+cos(theta1_M2)*ay), sin(theta2_M2+theta3_M2)*(cos(theta1_M2)*ax+sin(theta1_M2)*ay)+cos(theta2_M2+theta3_M2)*az)
            theta6_M2 = atan2(-sin(theta4_M2)*( cos(theta2_M2+theta3_M2)*(cos(theta1_M2)*nx+sin(theta1_M2)*ny)-sin(theta2_M2+theta3_M2)*nz )+cos(theta4_M2)*(-sin(theta1_M2)*nx+cos(theta1_M2)*ny), -sin(theta4_M2)*(cos(theta2_M2+theta3_M2)*(cos(theta1_M2)*sx+sin(theta1_M2)*sy)-sin(theta2_M2+theta3_M2)*sz)+cos(theta4_M2)*(-sin(theta1_M2)*sx+cos(theta1_M2)*sy))

#            if theta2_M2 > pi/2:
#               theta2_M2 = theta2_M2 - 2*pi
#            if theta3_M2 < -pi/2:
#                theta3_M2 = theta3_M2 + 2*pi

            #  The resulting joint variables of applying the half-angle tangent formula, i.e. Method 2
            q_M2 = array([[change*theta1_M2],[change*theta2_M2],[change*theta3_M2],[change*theta4_M2],[change*theta5_M2],[change*theta6_M2]])

            #  The results of the Method 1, the Method 2, and the Method 3

            
            if abs(theta2_M2-theta2)>1e-10:
                print q
                print q_M1


