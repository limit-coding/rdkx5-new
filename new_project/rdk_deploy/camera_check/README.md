# RDK X5 Camera Check Snapshot

Date captured: 2026-05-06
Remote host: `sunrise@172.20.10.2`

## Latest USB Camera Result

The USB camera is working.

Detected device:

- USB ID: `32e6:9221`
- Name: `openaicam openaicam`
- Video node: `/dev/video0`
- Extra metadata node: `/dev/video1`
- Media node: `/dev/media0`

Supported useful formats on `/dev/video0`:

- MJPEG `1920x1080@30fps`
- MJPEG `1280x720@30fps`
- YUYV `1920x1080@5fps`
- YUYV `1280x720@10fps`

Successful checks:

- `ffmpeg` captured `/tmp/usb_cam_test.jpg` from `/dev/video0`.
- `v4l2-ctl --stream-mmap --stream-count=10` captured a short MJPEG stream.
- `hobot_usb_cam` launched successfully with explicit device parameters and published `/image` and `/camera_info`.

Working ROS launch command:

```bash
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
ros2 launch hobot_usb_cam hobot_usb_cam.launch.py \
  usb_video_device:=/dev/video0 \
  usb_image_width:=1280 \
  usb_image_height:=720 \
  usb_pixel_format:=mjpeg \
  usb_framerate:=30
```

Important: the package default is `/dev/video8`, so pass `usb_video_device:=/dev/video0`.

## Previous MIPI Conclusion

The onboard MIPI camera stack is installed, but the MIPI camera does not currently initialize successfully.

Observed results:

- No `/dev/video*` or `/dev/media*` nodes are present.
- `media-ctl -p` cannot enumerate `/dev/media0`.
- USB sample reports `No USB camera found.`
- MIPI ROS launch can identify `X5_RDK` and supported hosts `0` and `2`, but both `mipi_channel:=0` and `mipi_channel:=2` fail at capture initialization:
  - `[mipi_cam]: [init]->cap capture init failture.`
  - `[mipi_node]: [init]->mipinode init failure.`
- Board ID is `302`; `/etc/board_config.json` maps cameras to:
  - `i2c_bus: 6`, `mipi_host: 0`
  - `i2c_bus: 4`, `mipi_host: 2`
- I2C scans did not show an obvious camera sensor response on those buses.
- Kernel logs include I2C timeout messages, suggesting the MIPI camera control path is not currently healthy.

Likely next checks:

1. Confirm the MIPI ribbon cable orientation and connector.
2. Confirm the camera is connected to host 0 or host 2 on this board.
3. Confirm the camera sensor model matches the configured driver/launch profile.
4. Power-cycle the board after reseating the camera.
5. If time is tight, use a UVC USB camera and retest with the USB camera sample or `/dev/video0`.

## Files

- `01_device_overview.txt`: hostname, kernel, camera device nodes, USB devices, available tools, user groups.
- `02_board_config.txt`: board ID, board config, Hobot config, boot config.
- `03_video_subsystem.txt`: video4linux sysfs, `media-ctl`, loaded camera/video modules.
- `04_i2c_scan.txt`: `/dev/i2c*` and `i2cdetect` results.
- `05_mipi_channel0_test.txt`: MIPI ROS launch test with `mipi_channel:=0`.
- `06_mipi_channel2_test.txt`: MIPI ROS launch test with `mipi_channel:=2`.
- `07_usb_camera_sample.txt`: USB camera sample result.
- `08_camera_dmesg_tail.txt`: camera/MIPI/I2C-related kernel log tail.
- `09_related_files_index.txt`: remote camera-related package/sample file index.
- `10_hobot_usb_cam_success.log`: successful ROS USB camera launch log.
- `usb_cam_test.jpg`: captured USB camera test frame.
- `usb_cam_stream.mjpg`: short USB camera stream captured with `v4l2-ctl`.
- `remote_files/mipi_cam_launch/`: copied MIPI ROS launch files.
- `remote_files/mipi_cam_config/`: copied MIPI calibration/config files.
- `remote_files/mipi_camera.py`: copied Python MIPI sample.
