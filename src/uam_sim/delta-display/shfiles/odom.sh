rostopic pub /world/quadrotor nav_msgs/Odometry "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'map'
child_frame_id: 'quadrotor'
pose:
  pose:
    position:
      x: 1.0
      y: 1.0
      z: 0.5
    orientation:
      x: 0.0
      y: 0.0
      z: 0.707
      w: 0.707"

