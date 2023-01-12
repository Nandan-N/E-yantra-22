#!/usr/bin/env python3

'''
Team ID: 2222
Author List: Pratham Patil, Nandan N, Sriram Radhakrishna, Samuel Thomas
Filename: waypoint_navigation.py
Theme: eYRC Sentinel Drone
Class: Edrone

This python file runs a ROS-node of name drone_control which holds the position of e-Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll
					
								

Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

# Importing the required libraries

from cmath import pi
from edrone_client.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time


class Edrone():
	"""docstring for Edrone"""
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z,yaw_value]
		self.drone_position = [0.0, 0.0, 0.0]

		# [x_setpoint, y_setpoint, z_setpoint,]
		self.setpoint = [2, 2, 20] # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly


		#Declaring a cmd of message type edrone_msgs and initializing values
		self.cmd = edrone_msgs()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		#self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500


		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters
		'''self.Kp = [0.69, 0.69, 8.3]
		self.Ki = [0.0011, 0.001, 0.1]
		self.Kd = [600, 900, 2002]'''
		self.Kp = [0.6, 0.7, 8.3]
		self.Ki = [0.0011, 0.001, 0.1]
		self.Kd = [500, 500, 2002]
		'''self.Kp = [28.8,19.68,11.88]
		self.Ki = [0.064,0.08,0.072]
		self.Kd = [0,0,0]'''
		

		#-----------------------Add other required variables for pid here ----------------------------------------------
		#self.error = self.setpoint - self.drone_position
		self.prev_error = [0] * 3
		#self.integral = [0,0,0,0] 
		#self.derivative = [0,0,0,0]
		self.max_values = [2000] * 3
		self.min_values = [1000] * 3
		self.error_sum = [0] * 3
		self.isfrstflght = True



		# Hint : Add variables for storing previous errors in each axis, like self.prev_error = [0,0,0] where corresponds to [pitch, roll, throttle]		#		 Add variables for limiting the values like self.max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
		#													self.min_values = [1000,1000,1000] corresponding to [pitch, roll, throttle]
		#																	You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------

		# # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the simulation step time is 50 ms
		self.sample_time = 0.060 # in seconds







		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error
		self.command_pub = rospy.Publisher('/drone_command', edrone_msgs, queue_size=1)
		#------------------------Add other ROS Publishers here-----------------------------------------------------
		self.command_pub_ae = rospy.Publisher('/alt_error',Float64, queue_size=1)
		self.command_pub_pe = rospy.Publisher('/pitch_error',Float64, queue_size=1)
		self.command_pub_re = rospy.Publisher('/roll_error',Float64, queue_size=1)
		#self.command_pub_yawe = rospy.Publisher('/yaw_error',Float64, queue_size=1)





		#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('/whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		#-------------------------Add other ROS Subscribers here----------------------------------------------------
		rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
		#rospy.Subscriber('/pid_tuning_yaw', PidTune, self.yaw_set_pid)
		#rospy.Subscriber('/drone_yaw', Int16, self.yaw_callback)


		#------------------------------------------------------------------------------------------------------------

		self.arm() # ARMING THE DRONE


	# Disarming condition of the drone
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)
		#rospy.signal_shutdown("disarmed")

	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
	def arm(self):

		self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(2)



	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x
		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
		p = msg.poses[0]
		self.drone_position[1:3] = [p.position.y, p.position.z]
		'''
		self.drone_position[1] = msg.poses[1].position.y
		self.drone_position[2] = msg.poses[2].position.z
		'''



		
		#---------------------------------------------------------------------------------------------------------------



	# Callback function for /pid_tuning_altitude
	# This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp * 0.01 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[2] = alt.Ki * 0.0001
		self.Kd[2] = alt.Kd * 1

	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------
	def pitch_set_pid(self, pitch):
		self.Kp[1] = pitch.Kp * 0.01
		self.Ki[1] = pitch.Ki * 0.0001
		self.Kd[1] = pitch.Kd * 1

	def roll_set_pid(self,roll):
		self.Kp[0] = roll.Kp * 0.01
		self.Ki[0] = roll.Ki * 0.0001
		self.Kd[0] = roll.Kd * 1


	#def yaw_set_pid(self, yaw):
	#	self.Kp[3] = yaw.Kp * 0.1 
	#	self.Ki[3] = yaw.Ki * 0.001
	#	self.Kd[3] = yaw.Kd * 1

	#def yaw_callback(self, yaw):
	#	self.drone_position[3] = yaw.data



	def run(self):
			#while (len(self.setpoint)>=3):
			#print(len(self.setpoint),self.setpoint[0], self.setpoint[1], self.setpoint[2], self.setpoint[3])
			if len(self.setpoint) == 3:
				l=[(0,0,23),(2,0,23),(2,2,23),(-2,2,23),(-2,-2,23),(2,-2,23),(2,0,23),(0,0,23)]
															
				for i in l:
					#go_to_goal(i,j,k)
					#self.setpoint[0] = i;self.setpoint[1] = j;self.setpoint[2] = k
					while not (max(max(self.prev_error),-min(self.prev_error)) < 0.2 and not self.isfrstflght):
						self.isfrstflght = False
						self.setpoint[0:3] = i
						self.pid()
					#print("next stop")
					self.isfrstflght = True
			'''while not (max(max(self.prev_error),-min(self.prev_error)) < 0.2 and not self.isfrstflght):
				self.isfrstflght = False
				self.pid()
			self.setpoint.pop(0)'''
			#self.isfrstflght = True
			#rospy.spin()
			'''self.cmd.rcRoll = 1500
			self.cmd.rcYaw = 1500
			self.cmd.rcPitch = 1500
			self.cmd.rcThrottle = 1500
			self.cmd.rcAUX4 = 1500
			print("out of while loop")
			self.command_pub.publish(self.cmd)'''

			#rospy.sleep(5)
			self.disarm()
			#print("disarming")






	#----------------------------------------------------------------------------------------------------------------------


	def pid(self):
	#-----------------------------Write the PID algorithm here--------------------------------------------------------------

	# Steps:
	# 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
		errors = [dp - setp for dp, setp in zip(self.drone_position, self.setpoint)]
		print(self.drone_position , self.setpoint)
		self.command_pub_re.publish(errors[0])
		self.command_pub_pe.publish(errors[1])
		self.command_pub_ae.publish(errors[2])
		#self.command_pub_yawe.publish(errors[3])
	
	#	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
		der_err = [ce - pr_e for ce, pr_e in zip(errors, self.prev_error)]
		new_error_sum = [ce + es for ce, es in zip(errors, self.error_sum)]
	
	#	3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
		out_vals = [sum(p*q for p, q in zip(a, b)) for a, b in zip(zip(errors, new_error_sum, der_err), zip(self.Kp, self.Ki, self.Kd))]
	
	#	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
		out_vals = [1500 + val for val in out_vals]
	
	#	6. Limit the output value and the final command value between the maximum(2000) and minimum(1000)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
		def clamp(n, minn, maxn): return max(min(maxn,n), minn)
		out_vals = [clamp(val, self.min_values[i], self.max_values[i]) for i, val in enumerate(out_vals)]
						
		self.cmd.rcPitch = int(out_vals[0])
		#rospy.spin()
		self.cmd.rcRoll = int(out_vals[1])
		#rospy.spin()
		self.cmd.rcThrottle = int(out_vals[2])
		#self.cmd.rcYaw = int(out_vals[3])
	#	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
		self.prev_error = errors
		print(self.prev_error)

	#	8. Add error_sum
		self.error_sum = new_error_sum

	#	5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
		time.sleep(e_drone.sample_time)

	#publish command to drone
		self.command_pub.publish(self.cmd)







	#------------------------------------------------------------------------------------------------------------------------


		
	




if __name__ == '__main__':

	e_drone = Edrone()

	time.sleep(2)
	r = rospy.Rate(15) #specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	while not rospy.is_shutdown():
		#e_drone.pid()
		e_drone.run()
		r.sleep()
		#e_drone.disarm()
#self.setpoint = [[0,0,23],[2,0,23],[2,2,23],[-2,2,23],[-2,-2,23],[2,-2,23],[2,0,23],[0,0,23]]