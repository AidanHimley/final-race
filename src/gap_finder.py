#!/usr/bin/env python
from copy import copy
import rospy
import math
from sensor_msgs.msg import LaserScan
from final.msg import gap_info
import numpy as np

class GapFinder():

	def __init__(self):
		print("Hokuyo LIDAR node started")
		
		# initialize node, subscriber, and publisher
		rospy.init_node('gap_finder', anonymous = True, disable_signals=True)
		rospy.Subscriber("/car_7/scan", LaserScan, self.callback)
		self.pub = rospy.Publisher('/car7/gap_info', gap_info, queue_size=10)

		# Some useful variable declarations.
		self.ANGLE_RANGE = 240			# Hokuyo 4LX has 240 degrees FoV for scan
		self.CAR_LENGTH = 0.50			# Traxxas Rally is 20 inches or 0.5 meters
		self.safety_radius = 0.35
		self.disparity_threshold = 0.75
		self.turning_radius = 1.2		# used to calculate if we can make a turn or not
		
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
		start = np.argmin(np.abs(angles+110))
		end = np.argmin(np.abs(angles-110))
		sub_ranges, sub_angles, = ranges, angles   # ranges[start:end], angles[start:end]
		
		disparities = []
		depth_counter = 0
		safety_bubbles = set()
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
		
		if len(disparities) == 0:
			print("not seeing any disparities")
			return sub_angles[np.argmax(sub_ranges)], 0, np.max(sub_ranges)
		
		disparity_angles = [sub_angles[disparity[0]] for disparity in disparities]
		print("Seeing " + str(len(disparities)) + " disparities at angles:\n" + str(disparity_angles))
		
		for disparity_index, delta_i in disparities:
			for j in range(max(0, disparity_index-delta_i), min(len(sub_ranges), disparity_index+delta_i+1)):
				safety_bubbles.add(j)

		max_depth, min_depth = max(depths), min(depths)
		try_depth = max_depth
		while (try_depth > min_depth):

			candidate_indices = []
			candidate_dists = []
			candidate_radii = []
			for i in range(1, len(sub_ranges)-1):
				if (i not in safety_bubbles and depths[i] == try_depth):
					if i-1 in safety_bubbles:
						search_index = i-1
						is_candidate = False
						while (not is_candidate and search_index >= 0 and search_index in safety_bubbles):
							if depths[search_index] < try_depth:
								is_candidate = True
								if sub_angles[search_index] < 0 or \
										sub_ranges[search_index] > 2*self.turning_radius*math.sin(math.radians(sub_angles[search_index])):
									candidate_indices.append(i)
									candidate_dists.append(sub_ranges[search_index+1] - sub_ranges[search_index])
									candidate_radii.append(sub_ranges[search_index]/(2*math.sin(math.radians(abs(sub_angles[search_index])))))
								else:
									print("rejecting candidate for turning radius")
							search_index -= 1
					if i+1 in safety_bubbles:
						search_index = i+1
						is_candidate = False
						while (not is_candidate and search_index < len(sub_ranges) and search_index in safety_bubbles):
							if depths[search_index] < try_depth:
								is_candidate = True
								if sub_angles[search_index] > 0 or \
										sub_ranges[search_index] > 2*self.turning_radius*math.sin(math.radians(-1*sub_angles[search_index])):
									candidate_indices.append(i)
									candidate_dists.append(sub_ranges[search_index-1] - sub_ranges[search_index])
									candidate_radii.append(sub_ranges[search_index]/(2*math.sin(math.radians(abs(sub_angles[search_index])))))
								else:
									print("rejecting candidate for turning radius")
							search_index += 1

			if len(candidate_indices) == 0:
				print("no candidates yet, finding eaten gaps")
				for disparity, _ in disparities:
					if depths[disparity+1] == try_depth:
						lower_i, upper_i = disparity, disparity+1
						while upper_i < len(depths)-1 and depths[upper_i] >= try_depth:
							upper_i += 1
						x1, y1 = sub_ranges[lower_i]*math.cos(math.radians(sub_angles[lower_i])), sub_ranges[lower_i]*math.sin(math.radians(sub_angles[lower_i]))
						x2, y2 = sub_ranges[upper_i]*math.cos(math.radians(sub_angles[upper_i])), sub_ranges[upper_i]*math.sin(math.radians(sub_angles[upper_i]))
						dist = math.sqrt((x1-x2)**2 + (y1-y2)**2)
						if upper_i < len(depths)-1 and  dist > self.disparity_threshold:
							print("Accepting secondary disparity at angles " + str(sub_angles[lower_i]) +
									" to " + str(sub_angles[upper_i]) + " with ranges " + str(sub_ranges[lower_i]) +
									" and " + str(sub_ranges[upper_i]))
							if sub_ranges[lower_i] < sub_ranges[upper_i]:
								if sub_angles[lower_i] < 0 or \
										sub_ranges[lower_i] > 2*self.turning_radius*math.sin(math.radians(sub_angles[lower_i])):
									candidate_indices.append(lower_i + int((self.safety_radius/(sub_ranges[lower_i]+self.CAR_LENGTH/2))/angle_increment))
							else:
								if sub_angles[upper_i] > 0 or \
										sub_ranges[upper_i] > 2*self.turning_radius*math.sin(math.radians(-1*sub_angles[upper_i])):
									candidate_indices.append(upper_i - int((self.safety_radius/(sub_ranges[upper_i]+self.CAR_LENGTH/2))/angle_increment))
							candidate_dists.append(dist)
						else:
							print("Rejecting secondary disparity at angles " + str(sub_angles[lower_i]) +
									" to " + str(sub_angles[upper_i]) + " with ranges " + str(sub_ranges[lower_i]) +
									" and " + str(sub_ranges[upper_i]))

			if len(candidate_indices) != 0:
				print("Seeing " + str(len(candidate_indices)) + " candidate indices")
				print(sub_angles[candidate_indices])
				candidate_index = np.argmax(candidate_dists)
				target_index = candidate_indices[candidate_index]
				return sub_angles[target_index], 0, sub_ranges[target_index]
			try_depth -= 1
			print("moving down to depth " + str(try_depth))

		print("NOT SEEING ANY CANDIDATES!!!")
		for disparity_index, delta_i in disparities:
			for j in range(max(0, disparity_index-delta_i), min(len(sub_ranges), disparity_index+delta_i+1)):
				sub_ranges[j] = 0
		return sub_angles[np.argmax(sub_ranges)], 0, np.max(sub_ranges)


	def notDepthCounter(self, ranges, angles, angle_increment):
		start = np.argmin(np.abs(angles+110))
		end = np.argmin(np.abs(angles-110))
		sub_ranges, sub_angles, = ranges, angles   # ranges[start:end], angles[start:end]
		
		disparities = []
		safety_bubbles = set()
		
		for gap_start in range(1, len(sub_ranges)):
			if abs(sub_ranges[gap_start] - sub_ranges[gap_start-1]) > self.disparity_threshold:
				if sub_ranges[gap_start] < sub_ranges[gap_start-1]:
					disparity_index = gap_start
				else:
					disparity_index = gap_start-1
				delta_i = int((self.safety_radius/(sub_ranges[disparity_index]+self.CAR_LENGTH/2))/angle_increment)
				disparities.append((disparity_index, delta_i))
		
		if len(disparities) == 0:
			print("not seeing any disparities")
			return sub_angles[np.argmax(sub_ranges)], 0, np.max(sub_ranges)
		
		disparity_angles = [sub_angles[disparity[0]] for disparity in disparities]
		print("Seeing " + str(len(disparities)) + " disparities at angles:\n" + str(disparity_angles))
		
		for disparity_index, delta_i in disparities:
			for j in range(max(0, disparity_index-delta_i), min(len(sub_ranges), disparity_index+delta_i+1)):
				safety_bubbles.add(j)

		candidate_indices = []
		candidate_dists = []
		for i in range(1, len(sub_ranges)-1):
			if (i not in safety_bubbles):
				if i-1 in safety_bubbles:
					search_index = i-1
					while (search_index > 0 and search_index in safety_bubbles):
						search_index -= 1
					if sub_ranges[i] - sub_ranges[search_index] > self.disparity_threshold:
						if sub_angles[search_index] < 0 or \
								sub_ranges[search_index] > 2*self.turning_radius*math.sin(math.radians(sub_angles[search_index])):
							candidate_indices.append(i)
							candidate_dists.append(sub_ranges[search_index+1] - sub_ranges[search_index])
						else:
							print("rejecting candidate at angle " + sub_angles[i] + " for turning radius")
					else:
						print("reject disparity at angle " + str(sub_angles[i]) + " for too close: only " + str(sub_ranges[i] - sub_ranges[search_index]) + " apart")
				if i+1 in safety_bubbles:
					search_index = i+1
					while (search_index < len(sub_ranges)-1 and search_index in safety_bubbles):
						search_index += 1
					if sub_ranges[i] - sub_ranges[search_index] > self.disparity_threshold:
						if sub_angles[search_index] > 0 or \
								sub_ranges[search_index] > 2*self.turning_radius*math.sin(math.radians(-1*sub_angles[search_index])):
							candidate_indices.append(i)
							candidate_dists.append(sub_ranges[search_index-1] - sub_ranges[search_index])
						else:
							print("rejecting candidate at angle " + str(sub_angles[i]) + " for turning radius")
					else:
						print("reject disparity at angle " + str(sub_angles[i]) + " for too close: only " + str(sub_ranges[i] - sub_ranges[search_index]) + " apart")

		if len(candidate_indices) != 0:
			print("Seeing " + str(len(candidate_indices)) + " candidate indices")
			print(sub_angles[candidate_indices])
			target_index = candidate_indices[np.argmax(candidate_dists)]
			return sub_angles[target_index], 0, sub_ranges[target_index]

		print("NOT SEEING ANY CANDIDATES!!!")
		for disparity_index, delta_i in disparities:
			for j in range(max(0, disparity_index-delta_i), min(len(sub_ranges), disparity_index+delta_i+1)):
				sub_ranges[j] = 0
		return sub_angles[np.argmax(sub_ranges)], 0, np.max(sub_ranges)


	def callback(self, data):
		#-------------------gap-finding logic goes here------------------------
		# rospy.loginfo("raw angles are " + str(data.angle_min) + " to " + str(data.angle_max))
		ranges, angles = self.getRanges(data)
		start = np.argmin(np.abs(angles+90))
		end = np.argmin(np.abs(angles-90))
		# ranges, angles, = ranges[start:end], angles[start:end]
		# rospy.loginfo("angles are " + str(angles[0]) + " to " + str(angles[-1]))

		msg = gap_info()		# An empty msg is created of the type gap_info
		# msg.angle = ...			# position of the center of the selected gap
		# msg.width = ...			# width of the selected gap
		# msg.depth = ...			# depth of the selected gap
		msg.angle, msg.width, msg.depth = self.depthCounter(ranges, angles, data.angle_increment)
		# if msg.angle < 0:
		# 	msg.depth = 0
		# 	self.pub.publish(msg)
		# 	rospy.signal_shutdown("going to the right")
		# for i in range(len(angles)):
		# 	print("angle: " + str(angles[i]) + "\trange: " + str(ranges[i]))
		# print(len(angles))
		rospy.loginfo("Aiming for gap at " + str(msg.angle) + " with width " + str(msg.width) + " and depth" + str(msg.depth))
		self.pub.publish(msg)

if __name__ == '__main__':
	GapFinder()