# COMS Odometry

Odometry publisher for the COMS vehicle.

## Parameters

- `wheel_diameter`: wheel's diameter in meters
- `counts_per_rotation`: pulse counts from the encoder in one wheel rotation
- `encoder_to_axis_center`: distance in meters from the encoder to the center
  of the shaft
- `base_frame`: TF frame name of the base link
- `odom_frame`: TF frame name of the odometry

## Subscribed topics

- `imu`: `sensor_msgs/Imu` message containing the current angular velocity of the vehicle
- `encoder`: `coms_msgs/ComsEncoder` message containing the current pulse
  count from the rotary encoder

## Published topics

- `odom`: `nav_msgs/Odometry` containing the current pose and velocity of the
  vehicle

## TF Transforms

- `odom` to `base_link`

## Caveats

Currently only considers 2D (i.e. x, y, and yaw). Needs modifications in order
to work in a 3D environment.

See the following URL for more details:

- http://wiki.ros.org/hector_slam/Tutorials/SettingUpForYourRobot
- http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom

## License

MIT License

## Author

Naoki Mizuno
