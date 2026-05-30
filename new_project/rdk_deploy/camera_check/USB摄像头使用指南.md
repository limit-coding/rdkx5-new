# RDK X5 USB 摄像头使用指南

适用环境：RDK X5 机载电脑，Ubuntu/ROS 2 Humble/TROS，USB UVC 摄像头。

当前验证过的摄像头：

- 设备名：`openaicam openaicam`
- USB ID：`32e6:9221`
- 主视频节点：`/dev/video0`
- 附加节点：`/dev/video1`
- Media 节点：`/dev/media0`

## 1. 硬件连接

1. 将 USB 摄像头插到 RDK X5 的 USB 口。
2. 建议使用短线或固定牢靠的 USB 线，避免无人机震动导致瞬断。
3. 上电后等待 3 到 5 秒，再检查设备枚举。

## 2. 检查摄像头是否识别

SSH 登录机载电脑：

```bash
ssh sunrise@172.20.10.2
```

查看 USB 设备：

```bash
lsusb
```

正常情况下应能看到类似：

```text
Bus 001 Device 004: ID 32e6:9221 openaicam openaicam
```

查看视频节点：

```bash
ls -l /dev/video* /dev/media*
v4l2-ctl --list-devices
```

正常结果类似：

```text
openaicam: openaicam (usb-xhci-hcd.2.auto-1.1):
    /dev/video0
    /dev/video1
    /dev/media0
```

主要使用 `/dev/video0`。

## 3. 查看支持的分辨率和格式

```bash
v4l2-ctl -d /dev/video0 --list-formats-ext
```

当前摄像头支持：

```text
MJPEG 1920x1080 @ 30fps
MJPEG 1280x720  @ 30fps
YUYV  1920x1080 @ 5fps
YUYV  1280x720  @ 10fps
```

推荐无人机竞赛先使用：

```text
1280x720, MJPEG, 30fps
```

理由：画面足够清楚，帧率稳定，USB 带宽和 CPU 压力比 1080p 更小。

## 4. 单张抓图测试

用 `ffmpeg` 抓一帧：

```bash
ffmpeg -hide_banner -loglevel info \
  -f v4l2 \
  -input_format mjpeg \
  -video_size 1280x720 \
  -i /dev/video0 \
  -frames:v 1 \
  /tmp/usb_cam_test.jpg
```

检查图片：

```bash
file /tmp/usb_cam_test.jpg
ls -lh /tmp/usb_cam_test.jpg
```

正常结果应显示 JPEG 图片，例如：

```text
/tmp/usb_cam_test.jpg: JPEG image data, 1280x720
```

把图片拷贝回本地电脑：

```bash
scp sunrise@172.20.10.2:/tmp/usb_cam_test.jpg ./usb_cam_test.jpg
```

本次测试图片已保存到本地：

```text
remote_snapshot/camera_check/usb_cam_test.jpg
```

## 5. 连续取流测试

用 `v4l2-ctl` 连续抓 10 帧：

```bash
v4l2-ctl -d /dev/video0 \
  --stream-mmap \
  --stream-count=10 \
  --stream-to=/tmp/usb_cam_stream.mjpg
```

检查输出：

```bash
ls -lh /tmp/usb_cam_stream.mjpg
file /tmp/usb_cam_stream.mjpg
```

本次测试流已保存到本地：

```text
remote_snapshot/camera_check/usb_cam_stream.mjpg
```

## 6. ROS 2 启动摄像头

加载 ROS/TROS 环境：

```bash
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
```

启动 USB 摄像头节点：

```bash
ros2 launch hobot_usb_cam hobot_usb_cam.launch.py \
  usb_video_device:=/dev/video0 \
  usb_image_width:=1280 \
  usb_image_height:=720 \
  usb_pixel_format:=mjpeg \
  usb_framerate:=30
```

注意：`hobot_usb_cam` 默认设备是 `/dev/video8`，本机实际是 `/dev/video0`，所以必须显式传：

```text
usb_video_device:=/dev/video0
```

## 7. 验证 ROS 图像话题

另开一个 SSH 终端，加载环境：

```bash
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
```

查看话题：

```bash
ros2 topic list
```

正常应出现：

```text
/image
/camera_info
```

查看一帧图像消息：

```bash
ros2 topic echo /image --once
```

正常情况下会看到：

```text
format: jpeg
data:
- 255
- 216
...
```

`255, 216` 是 JPEG 文件头，说明 ROS 已经收到压缩图像。

## 8. 推荐接入参数

比赛初期调试推荐：

```text
设备：/dev/video0
格式：MJPEG
分辨率：1280x720
帧率：30fps
ROS 图像话题：/image
ROS 相机信息话题：/camera_info
```

如果算法需要更高细节，可以改为：

```bash
ros2 launch hobot_usb_cam hobot_usb_cam.launch.py \
  usb_video_device:=/dev/video0 \
  usb_image_width:=1920 \
  usb_image_height:=1080 \
  usb_pixel_format:=mjpeg \
  usb_framerate:=30
```

但 1080p 会增加解码、传输和算法处理压力，建议确认帧率稳定后再用。

## 9. 常见问题

### 找不到 `/dev/video0`

检查 USB 是否识别：

```bash
lsusb
dmesg | grep -Ei "usb|uvc|video|camera" | tail -100
```

如果没有 `openaicam` 或其他摄像头名称，优先检查 USB 线、接口、供电和插拔状态。

### 出现 `/dev/video0`，但 ROS 没有图像

确认启动命令指定了正确设备：

```text
usb_video_device:=/dev/video0
```

不要使用默认 `/dev/video8`。

### `usb_camera_snap.py` 报 `No USB camera found`

本机测试中，`/app/pydev_demo/02_usb_camera_sample/usb_camera_snap.py` 仍然报：

```text
No USB camera found.
```

但 `ffmpeg`、`v4l2-ctl` 和 `hobot_usb_cam` 均已验证成功。因此这个 Python sample 不作为最终判断标准，优先使用 `/dev/video0`、`v4l2-ctl` 和 ROS 节点验证。

### 帧率低

优先使用 MJPEG，不要使用 YUYV：

```text
MJPEG 1280x720  @ 30fps
YUYV  1280x720  @ 10fps
```

如果算法端压力大，先降到 720p，不要一开始就跑 1080p。

### 设备编号变化

如果插拔后 `/dev/video0` 变成其他编号，可以查看稳定路径：

```bash
ls -l /dev/v4l/by-id/
ls -l /dev/v4l/by-path/
```

当前稳定软链接类似：

```text
/dev/v4l/by-id/usb-openaicam_openaicam_2505121128-video-index0
```

后续也可以在 launch 中使用该路径替代 `/dev/video0`，减少设备编号变化的影响。

## 10. 已保存的本地材料

本地目录：

```text
remote_snapshot/camera_check/
```

关键文件：

- `usb_cam_test.jpg`：USB 摄像头抓图结果。
- `usb_cam_stream.mjpg`：短视频流测试结果。
- `10_hobot_usb_cam_success.log`：ROS USB 摄像头成功启动日志。
- `01_device_overview.txt`：设备枚举概览。
- `README.md`：本次摄像头排查快照。

## 11. 最终结论

当前 USB 摄像头可用，建议后续无人机竞赛视觉链路基于 USB 摄像头推进。

推荐默认启动命令：

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
