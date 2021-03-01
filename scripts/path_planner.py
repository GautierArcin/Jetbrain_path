#!/usr/bin/env python

import rospy

from nav_msgs.msg import MapMetaData
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

from scipy.spatial.transform import Rotation as R
import cv2
import numpy as np
import math

import tf

from jetbrain_path import PRM


class Planner:

	def __init__(self):
		# Occupency Grid
		#
		# Note: we consider only 2d case, so we only take only into account x/y
		# Also orientation is not considered because the robot is considered spherical

		self.oGrid = None 
		self.oGridOrigin = None
		self.oGridCPM = None
		self.oGridWidth = None
		self.oGridHeight = None
		self.image=  None

		self.startPoint = None
		self.startOrientation = None

		self.endPoint = None
		self.endOrientation = None

		self.robotFootprint = 0.160

		#Config on planner
		self.path = None
		self.plannerNSample = 500 
		self.plannerEdgeFromeOneSamplePoint=20
		self.maxEdgeLength=10.0


		#RosCallback
		rospy.init_node('jetbrain_path_planner', anonymous=True)

		rospy.Subscriber('map', OccupancyGrid, self.callback_OGrid)
		rospy.Subscriber('start', PoseStamped, self.callback_startPos)
		rospy.Subscriber('end', PoseStamped, self.callback_endPos)
		self.pub = rospy.Publisher('/path', Path, latch=True)
		# https://stackoverflow.com/questions/40508651/writing-a-ros-node-with-both-a-publisher-and-subscriber
		timer = rospy.Timer(rospy.Duration(0.5), self.callback_timer)
		rospy.spin()
		timer.shutdown()



	def isPlannerExecutable(self):
		returnBool = True

		# Checking if we're lacking data to execute planner
		if self.startPoint is None:
			returnBool = False
			rospy.loginfo("Can't execute planner, start point is lacking")
		if self.endPoint is None:
			returnBool = False
			rospy.loginfo("Can't execute planner, end point is lacking")
		if self.oGrid is None:
			returnBool = False
			rospy.loginfo("Can't execute planner, map is lacking")

		# @todo Check if start or end point outside dimension of current map

		return returnBool



	def executePlanner(self):
		if(self.isPlannerExecutable()):

			sx = self.startPoint[0]  # [m]
			sy = self.startPoint[1]  # [m]
			gx = self.endPoint[0]  # [m]
			gy = self.endPoint[1]  # [m]
			robotSize = self.robotFootprint / 2  # [m]

			messageToLog =  "calling Prm planner with : "
			messageToLog += "Sp : " + str(sx) + ", " + str(sy)
			messageToLog += ", Gp : " + str(gx) + ", " + str(gy)
			messageToLog += ", Rz : " + str(robotSize) 
			messageToLog += ", CPM : " + str(round(self.oGridCPM))
			rospy.loginfo(messageToLog)

			beforeTime = rospy.Time.now()
			Planner = PRM(self.image , round(self.oGridCPM), sx, sy, gx, gy, robotSize, self.plannerNSample, self.plannerEdgeFromeOneSamplePoin, self.maxEdgeLength)
			rx, ry = Planner.startPlanner()
			afterTime = rospy.Time.now()

			difference = afterTime.secs - beforeTime.secs
			difference += (afterTime.nsecs - beforeTime.nsecs) * 0.000000001

			if rx is not None:
				Planner.saveToVideo(rx, ry, True)
				npRx = np.asarray(rx)
				npRy = np.asarray(ry)
				stacked = np.stack((npRx,npRy),-1) 

				msg = Path()
				msg.header.frame_id = "/path"
				msg.header.stamp = rospy.Time.now()
				for i in range(len(stacked)-1,-1,-1):
					print(stacked[i])
					pose = PoseStamped()

					#Adapting points to frame of Rviz
					pose.pose.position.x = (self.oGridWidth / (self.oGridCPM)) - stacked[i][0]
					pose.pose.position.y = (self.oGridHeight / (self.oGridCPM)) - stacked[i][1]


					pose.pose.position.z = 0
					pose.pose.orientation.x = 0
					pose.pose.orientation.y = 0
					pose.pose.orientation.z = 0
					pose.pose.orientation.w = 1
					msg.poses.append(pose)
				self.path = msg
				message = "goal found ! in " + str( difference ) +  "s"
				rospy.loginfo(message)
			else:
				message = "goal not found in " + str( difference) + "s... Try to change points or parameter of the planner."
				rospy.logwarn(message)





	def callback_timer(self, msg):

		if self.oGrid is not None:
			br = tf.TransformBroadcaster()
			#br.sendTransform(( - self.oGridOrigin[0], - self.oGridOrigin[1], 0.0), (0.0, 0.0, 0.0, 1), rospy.Time.now(), "map", "/path")
			br.sendTransform(( 0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), "map", "/path")

		if(self.path != None):
			self.pub.publish(self.path)

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
		self.executePlanner()


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
		self.executePlanner()


	def callback_OGrid(self, msg):
		# Generating Ogrid Image through OpenCV + diltation for path-planning
		# Inspired by this library: https://github.com/jnez71/lqRRT

		self.oGrid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
		self.oGridOrigin = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
		
		self.oGridCPM = 1/ msg.info.resolution
		self.oGridWidth = msg.info.width
		self.oGridHeight = msg.info.height

		#if self.oGrid is not None: 
		# Get opencv-ready image from current ogrid (255 is occupied, 0 is clear)
		oGridThreeshold = 90
		occImg = 255*np.greater(self.oGrid, oGridThreeshold).astype(np.uint8)

		#Flip the image (Might be Uncessary, did it to have the same orientation as Rviz vizualization)
		occImgFlip = cv2.flip(occImg, 1)

		#cv2.imshow("Map", occImg)
		#cv2.imshow("Map Flipped", occImgFlip)
		#cv2.waitKey(0)

		self.image = occImgFlip

		height, width  = occImg.shape 

		rospy.loginfo(rospy.get_caller_id() + ": Occupency Grid updated, shape : " + str(self.oGrid.shape) + ", origin : " + str(self.oGridOrigin) + ", res : " + str(msg.info.resolution) )
		rospy.loginfo(rospy.get_caller_id() + ": Image shape : " + str(height) + ", " + str(width) )


if __name__ == '__main__':
	try:
		Planner()
	except rospy.ROSInterruptException:
		pass
