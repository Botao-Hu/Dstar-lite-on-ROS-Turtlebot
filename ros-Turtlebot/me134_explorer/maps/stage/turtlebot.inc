# define kinect ranger
# (
#   sensor
#   (
#     range_max 6.5
#     fov 58.0
#     samples 640
#   )
#   # generic model properties
#   color "black"
#   size [ 0.06 0.15 0.03 ]
# )

define hokuyolaser ranger
(
  sensor( 			
  # laser-specific properties
  # factory settings for URG-04LX-UG01  240 degrees 0.36 degrees per sample
  # Our laser has about a 180 degree field of view and 512 samples
  range [ 0.0  5.6 ]
  fov 180.0
  samples 512
  )
  # generic model properties
  color "blue"
  size [ 0.07 0.07 0.05 ] # dimensions from LMS200 data sheet	
)


define turtlebot position
(
  pose [ 0.0 0.0 0.0 0.0 ]

  # odom_error [0.03 0.03 999999 999999 999999 0.02]
  # odom_error [0.0 0.0 999999 999999 999999 0.0]

  size [ 0.2552 0.2552 0.40 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "diff"
  color "gray"
  
  # kinect(pose [ -0.1 0.0 -0.11 0.0 ])
  # stage seems to default to a z value of 0.4 so 0.23-0.4=-0.17 to match the tf transform
  hokuyolaser(pose [ 0.06 0.0 -0.17 0.0 ])
  # pose [ x y z orientation]

  #br_lidar.sendTransform((0.06, 0.0, 0.23), (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), "laser", "base_link")
)
