# kinect-openrave-calibration
A ROS project to calibrate a scene in OpenRave using Kinect

# Running the module
- Start Kinect bridge
```
roslaunch kinect2_bridge kinect2_bridge.launch
```

- Launch AprilTag Publisher
```
roslaunch kinect-openrave-calibration apriltag.launch
```

- Run the OpenRave manager script
```
python kinect-openrave-calibration/scripts/SimTrackPerception.py
```
