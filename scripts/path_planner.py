#!/usr/bin/env python

import rospy

import numpy as np
from scipy.spatial.transform import Rotation as R
import math


from nav_msgs.msg import MapMetaData
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

import cv2



class Planner:
	def __init__(self):
		# Occupency Grid
		#
		# Note: we consider only 2d case, so we only take only into account x/y
		# Also orientation is not considered because the robot is considered spherical
		self.oGrid = None 
		self.oGridOrigin = None
		self.oGridRes = None
		self.oGridWidth = None
		self.oGridHeight = None

		self.startPoint = None
		self.startOrientation = None

		self.endPoint = None
		self.endOrientation = None

		self.robotFootprint = 0.160
		#self.robotFootprint = 2

		#RosCallback
		rospy.init_node('jetbrain_path_planner', anonymous=True)
		rospy.Subscriber('map', OccupancyGrid, self.callback_OGrid)
		rospy.Subscriber('start', PoseStamped, self.callback_startPos)
		rospy.Subscriber('end', PoseStamped, self.callback_endPos)
		rospy.spin() 

	def isPlannerExecutable(self):
		returnBool = True

		# Checking if we're lacking data to execute planner
		if startPoint is None:
			returnBool = False
			rospy.log(rospy.get_caller_id() + ": Can't execute planner, start point is lacking")
		if endPoint is None:
			returnBool = False
			rospy.log(rospy.get_caller_id() + ": Can't execute planner, end point is lacking")
		if oGrid is None:
			returnBool = False
			rospy.log(rospy.get_caller_id() + ": Can't execute planner, map is lacking")

		#  @ Check if start or end point outside dimension of current map

		return returnBool


	def callback_startPos(self, msg):
		self.startPoint = np.array([msg.pose.position.x, msg.pose.position.y])
		log = ": start point updated, point : " + str(self.startPoint) 
		
		# Quaternion utilization defined "" just in case "".
		# Because the robot is spherical, it's not used in the planenr code thoo
		try:
			self.startOrientation =  R.from_quat([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
			log += ", quaternion : " + str(self.startOrientation.as_quat())
		except ValueError:
			self.startOrientation = None
			rospy.logwarn(rospy.get_caller_id() + ": Invalid quaternion given on topic end, quaternion set to None")
			log += ", quaternion : " + str(self.startOrientation)

		rospy.loginfo(rospy.get_caller_id() + log)

	def callback_endPos(self, msg):
		self.endPoint = np.array([msg.pose.position.x, msg.pose.position.y])
		log = ": end point updated, point : " + str(self.endPoint) 

		# Quaternion utilization defined "" just in case "".
		# Because the robot is spherical, it's not used in the planner code thoo
		try:
			self.endOrientation =  R.from_quat([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
			log += ", quaternion : " + str(self.endOrientation.as_quat())
		except ValueError:
			self.endOrientation = None
			rospy.logwarn(rospy.get_caller_id() + ": Invalid quaternion given on topic end, quaternion set to None")
			log += ", quaternion : " + str(self.endOrientation)

		rospy.loginfo(rospy.get_caller_id() + log)

	def callback_OGrid(self, msg):
		# Generating Ogrid Image through OpenCV + diltation for path-planning
		# Inspired by this library: https://github.com/jnez71/lqRRT

		self.oGrid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
		self.oGridOrigin = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
		
		self.oGridCPM = 1 / msg.info.resolution
		self.oGridWidth = msg.info.width
		self.oGridHeight = msg.info.height

		#if self.oGrid is not None: 
		# Get opencv-ready image from current ogrid (255 is occupied, 0 is clear)
		oGridThreeshold = 90
		occImg = 255*np.greater(self.oGrid, oGridThreeshold).astype(np.uint8)

		# Dilate the image
		robotPix = int(self.oGridCPM*self.robotFootprint)
		robotPix += robotPix%2
		occImgDil = cv2.dilate(occImg, np.ones((robotPix, robotPix), np.uint8))

		#Flip the image (Might be Uncessary, did it to have the same orientation as Rviz vizualization)
		occImgDilFlip = cv2.flip(occImgDil, 1)

		cv2.imshow("Map", occImg)
		cv2.imshow("Dilated map", occImgDil)
		cv2.imshow("Dilated map Flipped", occImgDilFlip)
		cv2.waitKey(0)


		height, width  = occImg.shape 

		rospy.loginfo(rospy.get_caller_id() + ": Occupency Grid updated, shape : " + str(self.oGrid.shape) + ", origin : " + str(self.oGridOrigin) )
		rospy.loginfo(rospy.get_caller_id() + ": Image shape : " + str(height) + ", " + str(width) )


		"""
		# Construct the initial sample space and get bounds in pixel coordinates
		ss = self.behavior.gen_ss(self.next_seed, self.goal)
		pmin = self.intup(self.oGridCPM * ([ss[0][0], ss[1][0]] - self.ogrid_origin))
		pmax = self.intup(self.oGridCPM * ([ss[0][1], ss[1][1]] - self.ogrid_origin))

		# Other quantities in pixel coordinates
		seed = self.intup(self.oGridCPM * (self.next_seed[:2] - self.ogrid_origin))
		goal = self.intup(self.oGridCPM * (self.goal[:2] - self.ogrid_origin))
		step = int(self.oGridCPM * params.ss_step)

		# Make sure seed and goal are physically meaningful
		try:
			occ_img_dial[seed[1], seed[0]]
			occ_img_dial[goal[1], goal[0]]
		except IndexError:
			print("Goal and/or seed out of bounds of occupancy grid!")
			return(0, escape.gen_ss(self.next_seed, self.goal, 1), np.copy(self.goal), False)"""




if __name__ == '__main__':
	test = Planner()
