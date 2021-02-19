#!/usr/bin/env python

import rospy

import numpy as np
from scipy.spatial.transform import Rotation as R
import math

from nav_msgs.msg import MapMetaData
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped



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



		#RosCallback
		rospy.init_node('jetbrain_path_planner', anonymous=True)
		rospy.Subscriber('map', OccupancyGrid, self.callback_OGrid)
		rospy.Subscriber('start', PoseStamped, self.callback_startPos)
		rospy.Subscriber('end', PoseStamped, self.callback_endPos)
		rospy.spin() 


	def callback_startPos(self, msg):
		self.startPoint = np.array([msg.pose.position.x, msg.pose.position.y])
		log = ": start point updated, point : " + str(self.startPoint) 
		
		# Quaternion utilization defined "" just in case "".
		# Because the robot is spherical,it's not used in the code thoo
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
		# Because the robot is spherical,it's not used in the code thoo
		try:
			self.endOrientation =  R.from_quat([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
			log += ", quaternion : " + str(self.endOrientation.as_quat())
		except ValueError:
			self.endOrientation = None
			rospy.logwarn(rospy.get_caller_id() + ": Invalid quaternion given on topic end, quaternion set to None")
			log += ", quaternion : " + str(self.endOrientation)

		rospy.loginfo(rospy.get_caller_id() + log)

	def callback_OGrid(self, msg):
		self.oGrid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
		self.oGridOrigin = np.array([msg.info.origin.position.x, msg.info.origin.position.y])

		self.oGridRes = msg.info.resolution
		self.oGridWidth = msg.info.width
		self.oGridHeight = msg.info.height

		rospy.loginfo(rospy.get_caller_id() + ": Occupency Grid updated, shape : " + str(self.oGrid.shape) + ", origin : " + str(self.oGridOrigin) )


if __name__ == '__main__':
	test = Planner()

