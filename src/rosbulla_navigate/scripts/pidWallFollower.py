#!/usr/bin/env python

'''
source-
https://github.com/vipulkumbhar/AuE893_Autonomy_Science_and_Systems/blob/master/catkin_ws/src/assignment4/src/turtlebot3_wallfollowing.py
'''
import rospy
import numpy as np
from numpy import inf
from geometry_msgs.msg  import Twist, Vector3
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

class WallFollower():

	def __init__(self):
		self.side_scanstartangle = 20 
		self.side_scanrange      = 60          
		self.front_scanrange     = 16
		self.distancefromwall    = 0.4

		self.x   = np.zeros((360))
		self.s_d = 0 # front wall distance
		self.y_l = 0 # left wall distance
		self.y_r = 0 # right wall distance

		# PID parameters
		self.kp = 4
		self.kd = 450
		self.ki = 0

		self.k1 = self.kp + self.ki + self.kd
		self.k2 = -self.kp - 2*self.kd
		self.k3 = self.kp
		self.stop = False


		# Initialize parameters used in PID controller
		self.prev_PID_output = 0
		self.prev_error      = 0
		self.prev_prev_error = 0
		self.y_l
		self.y_r
		self.x
		self.s_d

		rospy.init_node('wallfollowing_control', anonymous=True)
		self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.scan_subscriber    = rospy.Subscriber('/scan', LaserScan, self.callback)
		self.stop_wall_sub    = rospy.Subscriber('/stopwall', Bool, self.callbackwallstart)
		self.rate               = rospy.Rate(10)                  # 20hz

	def callback(self, data):

		x  = list(data.ranges)
		for i in range(360):
			if x[i] == inf:
				x[i] = 7
			if x[i] == 0:
				x[i] = 6

			# store scan data 
		self.y_l= min(x[self.side_scanstartangle:self.side_scanstartangle+self.side_scanrange])          # left wall distance
		self.y_r= min(x[360-self.side_scanstartangle-self.side_scanrange:360-self.side_scanstartangle])  # right wall distance
		self.s_d= min(min(x[0:int(self.front_scanrange/2)],x[int(360-self.front_scanrange/2):360])) # front wall distance

	def callbackwallstart(self, data):
		if self.stop is not data.data:
			self.stop = data.data
			print(self.stop)
			if not self.stop:
				self.nextWall()
			else:
				self.velocity_publisher.publish(Twist(Vector3(0,0,0), Vector3(0,0,0)))
				print('Turtlebot stopped')
		rospy.loginfo("TEST")

	def nextWall(self):
		while not self.stop:

			delta = self.distancefromwall-self.y_r   # distance error 

			#PID controller
			# PID_output = prev_PID_output + k1*delta + k2*prev_error + k3*prev_prev_error

			#PD controller
			PID_output  = self.kp*delta + self.kd*(delta-self.prev_error)

			#stored states
			prev_error      = delta
			prev_prev_error = self.prev_error
			prev_PID_output = PID_output

			#clip PID output
			angular_zvel = np.clip(PID_output,-1.2,1.2)

			linear_vel   = np.clip((self.s_d-0.35),-0.1,0.4)

			#if linear_vel <0:
			#	angular_zvel = angular_zvel*-1 

			#if s_d < distancefromwall/3 and y_l >3:			
			#	PID_output = 0.1
			
			
			#check IOs
			print('distance from right wall in cm =',format(int(self.y_r*100)),'/',format(self.distancefromwall*100))
			print('distance from front wall in cm =',format(self.s_d*100))
			print('linear_vel=',format(linear_vel),' angular_vel=',format(angular_zvel))
			rospy.loginfo('\n') 

			#publish cmd_vel
			vel_msg = Twist(Vector3(linear_vel,0,0), Vector3(0,0,angular_zvel))
			self.velocity_publisher.publish(vel_msg)
			self.rate.sleep()

		

	
	

if __name__ == '__main__':
	try:
		#start turtllebot
		WallFollower()
		WallFollower().nextWall()
		rospy.spin()


	except rospy.ROSInterruptException: pass