#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

def create_transformation_matrix(alpha, a, d, q):
	trans_mat = Matrix([[ 				cos(q),				-sin(q),			0,			    a],
						[ 	 sin(q)*cos(alpha),	  cos(q)*cos(alpha),  -sin(alpha),	-sin(alpha)*d],
						[ 	 sin(q)*sin(alpha),	  cos(q)*sin(alpha),   cos(alpha),	 cos(alpha)*d],
						[ 					 0,					  0,			0,			    1]])
	return trans_mat
		
def get_angles_of_triangle(len_a,len_b,len_c):
	angle_a = acos((len_b * len_b + len_c * len_c - len_a * len_a)/(2. * len_b * len_c))
	angle_b = acos((len_a * len_a + len_c * len_c - len_b * len_b)/(2. * len_a * len_c))
	angle_c = acos((len_a * len_a + len_b * len_b - len_c * len_c)/(2. * len_a * len_b))
	return angle_a, angle_b, angle_c

def get_euler_angles_from_rot_mat(rot_mat):
	roll = atan2(rot_mat[2,2], -rot_mat[0,2])
	pitch = atan2(sqrt(rot_mat[0,2]*rot_mat[0,2] + rot_mat[2,2]*rot_mat[2,2]), rot_mat[1,2])
	yaw = atan2(-rot_mat[1,1], rot_mat[1,0])
	return roll, pitch, yaw
	
def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') #lengths
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') #offsets
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') #twist angles
        
		#Creating joing angle symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') #twist angles

        # Create Modified DH parameters
        DH_Table = {alpha0:		0,	a0:		0,	d1:	 0.75,	q1:			q1,
        		alpha1:-pi/2.,	a1:	 0.35,	d2:		0,	q2:	-pi/2.+ q2,
        		alpha2:		0,	a2:	 1.25,	d3:		0,	q3:			q3,
        		alpha3:-pi/2.,	a3:-0.054,	d4:	  1.5,	q4:			q4,
        		alpha4: pi/2.,	a4:		0,	d5:		0,	q5:			q5,
        		alpha5:-pi/2.,	a5:		0,	d6:		0,	q6:			q6,
        		alpha6:		0,	a6:		0,	d7:	0.303,	q7:			0}	

        # Create individual transformation matrices
        T0_1 = create_transformation_matrix(alpha0, a0, d1, q1).subs(DH_Table)
        T1_2 = create_transformation_matrix(alpha1, a1, d2, q2).subs(DH_Table)
        T2_3 = create_transformation_matrix(alpha2, a2, d3, q3).subs(DH_Table)
        T3_4 = create_transformation_matrix(alpha3, a3, d4, q4).subs(DH_Table)
        T4_5 = create_transformation_matrix(alpha4, a4, d5, q5).subs(DH_Table)
        T5_6 = create_transformation_matrix(alpha5, a5, d6, q6).subs(DH_Table)
        T6_EE = create_transformation_matrix(alpha6, a6, d7, q7).subs(DH_Table)

	
        # Extract rotation matrices from the transformation matrices
        T0_EE =  T0_1*T1_2*T2_3*T3_4*T4_5*T5_6*T6_EE
	
        # Compensate for rotation discrepancy between DH parameters and Gazebo
        rl, ph, yw = symbols('rl ph yw')
        rot_mat_x = Matrix([[1,		  0,		0],
        					[0,	cos(rl), -sin(rl)],
        					[0,	sin(rl),  cos(rl)]]) #for roll
        					
        rot_mat_y = Matrix([[  cos(ph),	0,  sin(ph)],
        					[		 0,	1, 		  0],
        					[ -sin(ph),	0,  cos(ph)]]) #for pitch
        					
        rot_mat_z = Matrix([[	cos(yw),   -sin(yw), 0],
        					[	sin(yw),	cos(yw), 0],
        					[		  0,	      0, 1]]) #for yaw
        					
        rot_mat_EE = rot_mat_z*rot_mat_y*rot_mat_x
        
        rot_mat_error = rot_mat_z.subs(yw, radians(180)) * rot_mat_y.subs(ph, radians(-90))
	
        rot_mat_EE = rot_mat_EE * rot_mat_error	
		
        psi = atan2(0.054,1.5) #refer notes
        side_A = sqrt(1.5*1.5 + 0.054*0.054) # sqrt(d4*d4, a3*a3)
        side_C = 1.25
		
        ###

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
            # Compensate for rotation discrepancy between DH parameters and Gazebo
            rot_mat_EE = rot_mat_EE.subs({'rl': roll, 'ph': pitch, 'yw': yaw})
            EE_pos = Matrix([[px],
            		 [py],
            		 [pz]])
	
            WC = EE_pos - 0.303 * rot_mat_EE[:,2] #r - n*d
	
            # Calculate joint angles using Geometric IK method
            theta1 = atan2(WC[1],WC[0])
            WC_xy = sqrt( WC[0]*WC[0] + WC[1]*WC[1])

            
            side_B = sqrt((WC_xy - 0.35)*(WC_xy - 0.35) + (WC[2]-0.75)*(WC[2]-0.75)) #refer notes
     
            angle_a, angle_b, angle_c = get_angles_of_triangle(side_A,side_B,side_C)

            theta2 = pi/2. - angle_a - atan2(WC[2]-0.75, WC_xy - 0.35)
            theta3 = pi/2. - angle_b - psi
	
            R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3] 
            R0_3 = R0_3.evalf(subs = {q1:theta1, q2: theta2, q3: theta3})
            R3_6 = R0_3.inv("LU") * rot_mat_EE
            theta4, theta5, theta6 = get_euler_angles_from_rot_mat(R3_6)
			
            ###

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
