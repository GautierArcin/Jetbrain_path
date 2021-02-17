#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

class Planner:
    def __init__(self):

	# Occupency Grid
	#
	# Note: we consider only 2d case, so we only take only into account x/y
	# Also orientation is not considered because the robot is considered spherical
	self.oGrid = None 
	self.oGridOrigin = None
	self.oGridRes = None

        rospy.init_node('jetbrain_path_planner', anonymous=True)
        rospy.Subscriber('map', OccupancyGrid, self.callback_OGrid)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin() 


    def callback_OGrid(self, msg):
        self.oGrid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.oGridOrigin = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
        self.oGridRes = msg.info.resolution
        #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
        rospy.loginfo(rospy.get_caller_id() + ": Occupency Grid updated, shape : " + str(self.oGrid.shape) + ", origin : " + str(self.oGridOrigin) )


if __name__ == '__main__':
    test = Planner()
