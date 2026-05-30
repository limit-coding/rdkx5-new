ros2 topic echo /Odometry | awk '
/stamp:/ {in_stamp=1}
/^ *sec:/ && in_stamp {ros_sec=$2}
/nanosec:/ && in_stamp {
  ros_nsec=$2
  in_stamp=0
  if (ros_sec < 1704067200) {
    printf "TIME_ERROR SYS=%s  ROS=%s.%09d  raw_sec=%d\n", strftime("%Y-%m-%d %H:%M:%S"), strftime("%Y-%m-%d %H:%M:%S", ros_sec), ros_nsec, ros_sec
  }
}'
