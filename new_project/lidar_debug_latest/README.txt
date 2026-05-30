This directory is one lidar/FAST-LIO/FC debug session.

Files:
- lidar_summary.csv: one row per /livox/lidar frame, raw radar statistics.
- lidar_xy.csv: sampled raw radar point coordinates from /livox/lidar.
- fastlio.csv: /Odometry, /livox/imu, /relative_pose and localization health.
- fc_frames.csv: actual frames published by fc_bridge after serial write.

Copy latest session back to Mac:
scp -r sunrise@172.20.10.2:/home/sunrise/project/debug_logs/lidar_fc/latest ./lidar_debug_latest

Important columns:
- wall_ms: board system time in milliseconds since epoch.
- stamp_sec: ROS message timestamp in seconds.
- lidar_xy.csv x/y/z: raw point coordinates in Livox lidar frame.
- fc_frames.csv raw_message: frame type, cm values, yaw, hex bytes, and reason.
- fastlio.csv cause: quick diagnosis computed from current window.
