# Path planning in ROS

## Task description

Suppose you are developing path planning module of the robot. You will receive a test map files, both yaml and pgm. The goal for this task is to implement and test on the provided map global path planning algorithm, that will take into account obstacles on the map and will be capable of constructing feasible route from point A to point B.

## Rules

1. Deliverable is a ROS/ROS2 package, that is capable of constructing a route from point A to point B, avoiding obstacles from the map.
2. Assume that the robot has circular footprint of diameter 160 mm.
3. You should subscribe to

	* /start - start position of the route, geometry_msgs/PoseStamped
	* /end   - end position of the route, geometry_msgs/PoseStamped
	 
	Route should be constructed from last received poses on topics above

4. You should publish found route to
	* /path - found path, nav_msgs/Path.
  
5. The solution must contain C++ or Python code.
6. You are not allowed to use 3rd party packages or simple algorithms like A*, Djisktra for path planning.
