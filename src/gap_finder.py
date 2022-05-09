#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan
from final.msg import gap_info
import numpy as np

class GapFinder():

	def __init__(self):
		print("Hokuyo LIDAR node started")
		
		# initialize node, subscriber, and publisher
		rospy.init_node('gap_finder', anonymous = True)
		rospy.Subscriber("/car_7/scan", LaserScan, self.callback)
		self.pub = rospy.Publisher('/car7/gap_info', gap_info, queue_size=10)

		# Some useful variable declarations.
		self.ANGLE_RANGE = 240			# Hokuyo 4LX has 240 degrees FoV for scan
		self.CAR_LENGTH = 0.50			# Traxxas Rally is 20 inches or 0.5 meters
		self.safety_radius = 0.25
		self.disparity_threshold = 0.25
		
		rospy.spin()
	

	def getRanges(self, data):
		"""data: single message from topic /scan
		outputs angles in degrees with 0 degrees directly in front, positive values to the left
		Outputs length in meters to object with angle in lidar scan field of view\n
		Cleans NaNs etc"""

		ranges = []
		angles = []

		for i in range(len(data.ranges)):
			if (data.ranges[i] and data.ranges[i] < data.range_max and data.ranges[i] > data.range_min):
				ranges.append(data.ranges[i])
				angles.append(math.degrees(data.angle_min + i*data.angle_increment))
		return np.array(ranges), np.array(angles)


	def disparityExtenderDeepest(self, ranges, angles, angle_increment):
		start = np.argmin(np.abs(angles+70))
		end = np.argmin(np.abs(angles-70))
		sub_ranges, sub_angles, = ranges[start:end], angles[start:end]
		disparities = []
		for gap_start in range(1, len(sub_ranges)):
			if abs(sub_ranges[gap_start] - sub_ranges[gap_start-1]) > self.disparity_threshold:
				disparity_index = gap_start if sub_ranges[gap_start] < sub_ranges[gap_start-1] else gap_start-1
				delta_i = int((self.safety_radius/(sub_ranges[disparity_index]+self.CAR_LENGTH/2))/angle_increment)
				disparities.append((disparity_index, delta_i))
		disparity_angles = [sub_angles[disparity[0]] for disparity in disparities]
		
		# for i in range(len(sub_angles)):
		# 	print("angle: " + str(sub_angles[i]) + "\trange: " + str(sub_ranges[i]))
		rospy.loginfo("Seeing " + str(len(disparities)) + " disparities at angles:\n" + str(disparity_angles))
		
		for disparity_index, delta_i in disparities:
			for j in range(max(0, disparity_index-delta_i), min(len(sub_ranges), disparity_index+delta_i+1)):
				sub_ranges[j] = 0

		# find deepest gap, closest to straight ahead as tiebreaker
		candidate_indices = (sub_ranges==np.max(sub_ranges)).nonzero()[0]
		deepest_index = candidate_indices[np.argmin(abs(sub_angles[candidate_indices]))]		# lol
		
		# find start and end of gap
		gap_start = deepest_index
		while gap_start>=0 and sub_ranges[gap_start] != 0:
			gap_start -= 1
		gap_end = deepest_index+1
		while gap_end<len(sub_ranges) and sub_ranges[gap_end] != 0:
			gap_end += 1
		
		# aim for the center of the gap, calculate width
		target_index = gap_start + int((gap_end-gap_start)/2)
		width=(gap_end-gap_start)*math.degrees(angle_increment)
		if width < 60:
			return sub_angles[target_index], width, sub_ranges[deepest_index]
		else:
			return sub_angles[deepest_index], width, sub_ranges[deepest_index]


	def depthCounter(self, ranges, angles, angle_increment):
		start = np.argmin(np.abs(angles+90))
		end = np.argmin(np.abs(angles-90))
		sub_ranges, sub_angles, = ranges[start:end], angles[start:end]
		
		disparities = []
		depth_counter = 0
		depths = [depth_counter]
		
		for gap_start in range(1, len(sub_ranges)):
			if abs(sub_ranges[gap_start] - sub_ranges[gap_start-1]) > self.disparity_threshold:
				if sub_ranges[gap_start] < sub_ranges[gap_start-1]:
					disparity_index = gap_start
					depth_counter -= 1
				else:
					disparity_index = gap_start-1
					depth_counter += 1
				delta_i = int((self.safety_radius/(sub_ranges[disparity_index]+self.CAR_LENGTH/2))/angle_increment)
				disparities.append((disparity_index, delta_i))
			depths.append(depth_counter)
		
		disparity_angles = [sub_angles[disparity[0]] for disparity in disparities]
		
		# for i in range(len(sub_angles)):
		# 	print("angle: " + str(sub_angles[i]) + "\trange: " + str(sub_ranges[i]))
		rospy.loginfo("Seeing " + str(len(disparities)) + " disparities at angles:\n" + str(disparity_angles))
		
		for disparity_index, delta_i in disparities:
			for j in range(max(0, disparity_index-delta_i), min(len(sub_ranges), disparity_index+delta_i+1)):
				sub_ranges[j] = 0

		max_depth = max(depths)
		for i in range(len(sub_ranges)):
			if depths[i] != max_depth:
				sub_ranges[i] = 0
		
		candidate_indices = []
		for i in range(1, len(sub_ranges)-1):
			if (sub_ranges[i] != 0) and (sub_ranges[i-1] == 0 or sub_ranges[i+1] == 0):
				candidate_indices.append(i)

		target_index = candidate_indices[np.argmax(sub_ranges[candidate_indices])]
		return sub_angles[target_index], 0, sub_ranges[target_index]

	def callback(self, data):

		#-------------------gap-finding logic goes here------------------------
		# rospy.loginfo("raw angles are " + str(data.angle_min) + " to " + str(data.angle_max))
		ranges, angles = self.getRanges(data)
		start = np.argmin(np.abs(angles+90))
		end = np.argmin(np.abs(angles-90))
		ranges, angles, = ranges[start:end], angles[start:end]
		# rospy.loginfo("angles are " + str(angles[0]) + " to " + str(angles[-1]))

		msg = gap_info()		# An empty msg is created of the type gap_info
		# msg.angle = ...			# position of the center of the selected gap
		# msg.width = ...			# width of the selected gap
		# msg.depth = ...			# depth of the selected gap
		msg.angle, msg.width, msg.depth = self.disparityExtenderDeepest(ranges, angles, data.angle_increment)
		# for i in range(len(angles)):
		# 	print("angle: " + str(angles[i]) + "\trange: " + str(ranges[i]))
		# print(len(angles))
		rospy.loginfo("Aiming for gap at " + str(msg.angle) + " with width " + str(msg.width) + " and depth" + str(msg.depth))
		self.pub.publish(msg)

if __name__ == '__main__':
	GapFinder()