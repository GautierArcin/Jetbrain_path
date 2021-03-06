# Video:

[![Alt text](https://img.youtube.com/vi/khMNjzpGrQ0/0.jpg)](https://www.youtube.com/watch?v=khMNjzpGrQ0)

# Running the script:

In a first terminal:

`cd ~/catkin_ws/`

`source devel/setup.bash`

`roslaunch jetbrain_path jetbrain-path.launch`

(launch map + rviz)

In a second one:

`cd ~/catkin_ws/`

`source devel/setup/.bash`

`rosrun jetbrain_path path_planner.py`

<br>

# Option of the script

- --robotSize: Diameter of the robot, default=0.160 \[ m \]
- --nSample: Number of point to sample in Planner, default=500
- --edgeMax: Maximum edge from one sample point, default= 10
- --maxEdgeLength: Max length of edge in planner, default= 10.0 \[ m \]
- --saveImage: Save planning to video + gif after having realized it, default=false

<br>

# Optional copy/paste to publish on start/end

Totally optional paste. In order to give start/end point in an easy way.

<br>

## Publish on /start

```
rostopic pub /start geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: '/point'
pose:
  position:
    x: 16.6
    y: 1.6
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0"
```

<br>

## Publish on /end

```
rostopic pub /end geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: '/point'
pose:
  position:
    x: 13.2
    y: 11.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0"
```
