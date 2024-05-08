## Calibration procedure

To get the calibration of the camera location:

- Uncomment the `base_tag` in `apriltag.yml` and comment out the `base_tag` to `camera_link` static transform publisher in `vision.launch.py`
- Build
- Run `ros2 launch micro_g_vision vision.launch.py`
- (Separate terminal) run `ros2 run tf2_ros tf2_echo base_tag camera_link`
- Record the output in the base_tag to camera_link static transform publisher
- Re-comment `base_tag` in `apriltag.yml` and uncomment the static transform publisher
- Re-build
