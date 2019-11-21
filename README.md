# visual_marker_6d_position_estimation

This program detects the 6D position of aruco visual markers.

1. It calibrates the camera to gets camera matrix and distortion coefficients.
2. Gets predefined aruco dictionary markersof type DICT_4X4_100.
3. Load, find aruco markers and draw the 3D axis.


Call: ./visual_marker_6d_position_estimation -path calibration_images -path marker_images

