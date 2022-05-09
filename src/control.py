#!/usr/bin/env python
import rospy
from final.msg import gap_info
from ackermann_msgs.msg import AckermannDrive
from collections import deque

class Controller():

	def __init__(self, servo_offset=0.0):
		# initialize node
		rospy.init_node('controller', anonymous=True)

		# This code can input desired velocity from the user.
		# velocity must be between [0,100] to move forward. 
		# The following velocity values correspond to different speed profiles.
		# 15: Very Slow (Good for debug mode)
		# 25: Slow and steady
		# 35: Nice Autonomous Pace
		# > 40: Careful, what you do here. Only use this if your autonomous steering is very reliable.
		# this is used as max velocity for dynamic velocity
		self.max_velocity = 30 # float(input("Enter desired maximum velocity: "))
		self.min_velocity = 15

		# zero correction offset in case servo is misaligned and has a bias in turning.
		self.servo_offset = servo_offset

		# initialize subscriber and publisher (do this after parameters have been set)
		rospy.Subscriber('/car7/gap_info', gap_info, self.control)
		self.command_pub = rospy.Publisher('/car_7/offboard/command', AckermannDrive, queue_size = 1)

		rospy.spin()

	def control(self, data):
		angle = 0

		#----------------control logic goes here-------------------
		#Based on what I understand here of what I'm getting, the calculations are being done in 
		#Gapfinder, with the angle being fed in relative to the car, so therefore all that needs to be
		#done here is setting the angle. It is also 5am, I am running on very little sleep, and just 
		#want today to kind of end
                # -120 to 120 is the angle range, so divide by 1.2 to be between -100 and 100
		angle = data.angle*2
		
		# An empty AckermannDrive message is created. You will populate the steering_angle and the speed fields.
		command = AckermannDrive()

		# Make sure the steering value is within bounds [-100,100]
		command.steering_angle = max(-100, min(angle, 100))

		# Make sure the velocity is within bounds [0,100]
		MIN_DEPTH = 1
		MAX_DEPTH = 4
		if data.depth < MIN_DEPTH:
			velocity = self.min_velocity
		elif data.depth < MAX_DEPTH:
			velocity = self.min_velocity + (data.depth-MIN_DEPTH)/(MAX_DEPTH-MIN_DEPTH)*(self.max_velocity-self.min_velocity)
		else:
			velocity = self.max_velocity
		command.speed = max(0, min(velocity, 100))
		rospy.loginfo("commanding speed " + str(command.speed) + " and angle " + str(command.steering_angle))
		# Move the car autonomously
		self.command_pub.publish(command)

if __name__ == '__main__':
	Controller()
