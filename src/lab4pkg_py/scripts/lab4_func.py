#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
from lab4_header import *
from ModernRobotics import *
"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for a1~6 and q1~6, as well as the M matrix
	#M = np.eye(4)
	#S = np.zeros((6,6))
	#M is relative to world frame
	M = np.array([[0,-1,0,390],[0,0,-1,401],[1,0,0,215.5],[0,0,0,1]])
		#find the S matricies
	#S1
	w1 = np.array([ 0, 0,1 ])
	q1 = np.array([ -150, 150, 162])
	v1 = np.cross(-w1,q1)
	s1 = np.append(w1,v1)

	#S2
	w2 = np.array([0 ,1 ,0 ])
	q2 = np.array([ -150,270 , 162])
	v2 = np.cross(-w2,q2)
	s2 = np.append(w2,v2)
	#S3
	w3 = np.array([ 0, 1,0 ])
	q3 = np.array([ 94, 240, 162])
	v3 = np.cross(-w3,q3)
	s3 = np.append(w3,v3)
	#S4
	w4 = np.array([ 0, 1,0 ])
	q4 = np.array([ 307,177 ,162 ])
	v4 = np.cross(-w4,q4)
	s4 = np.append(w4,v4)
	#S5
	w5 = np.array([1 ,0 ,0 ])
	q5 = np.array([ 307,260 ,162 ])
	v5 = np.cross(-w5,q5)
	s5 = np.append(w5,v5)
	#S3
	w6 = np.array([ 0, 1, 0])
	q6 = np.array([390 , 260, 162])
	v6 = np.cross(-w6,q6)
	s6 = np.append(w6,v6)
	S = np.array([s1,s2,s3,s4,s5,s6])   # might need to trannypose ?

	l1 = 152
	l2 = 120
	l3 = 244
	l4 = 93
	l5 = 213
	l6 = 83
	l7 = 83
	l8  = 82
	l9 = 53.5
	l10 = 59
	
	###########################













	# ==============================================================#
	return M, s1,s2,s3,s4,s5,s6


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	print("Foward kinematics calculated:\n")


	# =================== Your code starts here ====================#
	theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
	T = np.eye(4)
	
	M,s1,s2,s3,s4 ,s5,s6 = Get_MS()

	T = expm(VecTose3(s1)*theta1) @ expm(VecTose3(s2)*theta2)@ expm(VecTose3(s3)*theta3) @ expm(VecTose3(s4)*theta4) @ expm(VecTose3(s5)*theta5) @ expm(VecTose3(s6)*theta6) @ M



	# ==============================================================#

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6
	print(T)
	return return_value




	# ==============================================================#




"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	# =================== Your code starts here ====================#
	xWgrip = xWgrip + .150
	yWgrip = yWgrip - .150
	zWgrip = zWgrip - .010
	links = np.array([-1000, 152, 120, 244, 93, 213, 83, 83, 82, 53.5, 59])/1000
	print(f'Links: {links}')
	x_center = xWgrip - 0.0535*np.sin(np.radians(yaw_WgripDegree))
	y_center = yWgrip - 0.0535*np.cos(np.radians(yaw_WgripDegree))
	z_center = zWgrip

	theta1 = np.arctan2(y_center, x_center) - np.arcsin((0.110) / np.sqrt(x_center**2 + y_center**2))
	theta6 = (theta1 + np.pi/2) - np.radians(yaw_WgripDegree)

	rotation = np.array([[-np.cos(theta1),  np.sin(theta1)],
					  	 [-np.sin(theta1), -np.cos(theta1)]])
	
	# x_3end = x_center + links[7] * -1*np.cos(theta1) + (links[6] - 0.027)*np.sin(theta1))
	# y_3end = y_center + links[7] *  -1*np.sin(theta1) + (links[6] - 0.027)*-1*np.cos(theta1))

	offset_dist = np.array([[links[7]],
						    [links[6] + 0.027]])
	
	pos_3end = np.array([[x_center],[y_center]]) + (rotation @ offset_dist)
	
	x_3end, y_3end = pos_3end.reshape(2,).tolist()
	z_3end = z_center + links[10] + links[8]

	# c_line = np.sqrt((x_3end - (-0.150))**2 + (z_3end - (0.010 + links[1]))**2)
	c_line = np.sqrt((x_3end - 0)**2 + (y_3end - 0)**2 + (z_3end - links[1])**2) # not ours PLEASE REMOVE
	c_line = np.sqrt(x_3end**2 + y_3end**2 + (z_3end - links[1])**2)
	print(f'Cl_line: {c_line}')

	theta3 = (np.pi) - (np.arccos((links[5]**2 + links[3]**2 - c_line**2)/(2*links[3]*links[5])))
	print(f'ratio: {(links[5]**2 + links[3]**2 - c_line**2)/(2*links[3]*links[5])}')

	theta2small = np.arccos(np.sqrt(x_3end**2 + y_3end**2)/c_line)

	theta2big = -1 * np.arccos((links[3]**2 + c_line**2 - links[5]**2) / (2*links[3]*c_line))
	
	#theta2 = theta2small + theta2big
	theta2 = -theta2small - theta2big
	theta4 = -theta2 - theta3
	theta5 = -np.pi/2

	print(f'Theta 1: {np.degrees(theta1)}')
	print(f'Theta 2: {np.degrees(theta2)}')
	print(f'Theta 3: {np.degrees(theta3)}')
	print(f'Theta 4: {np.degrees(theta4)}')
	print(f'Theta 5: {np.degrees(theta5)}')
	print(f'Theta 6: {np.degrees(theta6)}')

	# ==============================================================#
	return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)
