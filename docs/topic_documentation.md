# Published topics

## Realsense topics

- Internal IMU
  - `/accel/imu_info`
  - `/accel/metadata`
  - `/accel/sample`
  - `/gyro/imu_info`
  - `/gyro/metadata`
  - `/gyro/sample`
  - `/imu`

- Color camera images
  - `/color/camera_info`
  - `/color/image_raw`
  - `/color/image_raw/compressed`
  - `/color/image_raw/compressedDepth`
  - `/color/image_raw/theora`
  - `/color/metadata`

- Depth images
  - `/depth/camera_info`
  - `/depth/image_rect_raw`
  - `/depth/image_rect_raw/compressed`
  - `/depth/image_rect_raw/compressedDepth`
  - `/depth/image_rect_raw/theora`
  - `/depth/metadata`

- Infrared images
  - `/infra1/camera_info`
  - `/infra1/image_rect_raw`
  - `/infra1/image_rect_raw/compressed`
  - `/infra1/image_rect_raw/compressedDepth`
  - `/infra1/image_rect_raw/theora`
  - `/infra1/metadata`

- Camera extrinsics
  - `/extrinsics/depth_to_accel`
  - `/extrinsics/depth_to_color`
  - `/extrinsics/depth_to_depth`
  - `/extrinsics/depth_to_gyro`
  - `/extrinsics/depth_to_infra1`

## Apriltag detections

- Raw detections
  - `/detections`

## Object tracker

Applies a moving average filter to the Apriltag detections of the cube

- `/object_pose`
- `/odometry/unfiltered`
- `/time_since_last_seen`

## Grasp selector node

Decides where and when to grasp the object

- Information about the object tracking and predicted intercept point
  - `/grasp_selector/current_target_point`
  - `/grasp_selector/estimated_angle_of_incidence`
  - `/grasp_selector/estimated_grasp_point`
  - `/grasp_selector/estimated_velocity`
- Commands sent to robot
  - `/px100/desired_eef_pose`
  - `/px100/grasp`

## PX100 arm interface

- `/px100/robot_description`
- `/px100/joint_states`
- `/px100/commands/joint_group`
- `/px100/commands/joint_single`
- `/px100/commands/joint_trajectory`

## Other

- `/parameter_events`
- `/rosout`
- `/tf`
- `/tf_static`
