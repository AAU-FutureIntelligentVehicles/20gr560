define laser ranger
(
  sensor
  (
    range_max 6.5
    fov 58.0
    samples 640
  )
  # generic model properties
  color "black"
  size [2.4 1.2 1.0]
)

#
# Robot model:
# footprint (clockwise): [-1.2, -0.6], [-1.2, 0.6], [1.2, 0.6], [1.2, -0.6]
# center of rotation: [0.0, 0.33]
# wheelbase: 1.6

define carlike_robot position
(
  pose [ 0.0 0.0 0.0 0.0 ]

  odom_error [0.03 0.03 999999 999999 999999 0.02]

  size [ 2.4 1.2 1.0 ] # this models the footprint (rectangular), but shifted such that the bottom-left corner is in [0,0]. The center of rotation now here at [0.3, 0.125, 0.2]
  # correct center of rotation:
  origin [ 0.0 0.33 0.0 0.0]
  gui_nose 1
  color "red"

  # kinematics
  drive "car"
  wheelbase 1.6 # distance between both axles
  
  # spawn sensors
  laser(pose [ 0.0 0.0 0.0 0.0 ])
)
