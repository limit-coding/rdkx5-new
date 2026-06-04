#!/bin/bash
# =============================================================================
# 全链路统一启动脚本 (开机自启)
# 启动顺序:
#   1. USB摄像头 (hobot_usb_cam)
#   2. 视觉识别 (mission_vision: QR + ONNX分类)
#   3. 桥接节点 (mission_bridge)
#   4. 任务状态机 (task_state_machine → /task_status)
#   5. 飞控串口 (fc_bridge → 收发统一，含原uart功能)
#   6. 舵机投放控制 (servo_drop → 订阅 /pr_select 控制舵机)
#   7. 雷达链路 [仅eth0有192.168.1.x时] (livox → FAST-LIO → mid360_xy → relative_pose)
#   8. watchdog (节点健康监控，fc_bridge崩溃自动重启)
# =============================================================================

LOG_DIR="/tmp/flight_logs"
mkdir -p "$LOG_DIR"

log() { echo "[$(date '+%H:%M:%S')] $1" | tee -a "$LOG_DIR/startup.log"; }

# 收到 systemd/手动停止信号时杀掉所有子进程。
# 不在普通 EXIT 上 cleanup，避免 ros2 run 包装进程退出时误杀已经拉起的节点。
cleanup() {
    log "退出，正在停止所有节点..."
    kill $(jobs -p) 2>/dev/null
    wait 2>/dev/null
}
trap cleanup SIGINT SIGTERM

log "=========================================="
log "  全链路启动"
log "=========================================="

# ── 杀掉残留进程 ──────────────────────────────
pkill -f "hobot_usb_cam"      2>/dev/null
pkill -f "mission_vision"     2>/dev/null
pkill -f "mission_bridge"     2>/dev/null
pkill -f "task_state_machine" 2>/dev/null
pkill -f "fc_bridge"          2>/dev/null
pkill -f "servo_drop"         2>/dev/null
pkill -f "communication.*uart" 2>/dev/null
sleep 2
log "[清理] 残留进程已清理"

# ── 加载环境 ──────────────────────────────────
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
source /home/sunrise/lidar_ws/install/setup.bash
source /home/sunrise/project/install/setup.bash
export PYTHONPATH="/home/sunrise/project/install/main/lib/python3.10/site-packages:$PYTHONPATH"
log "[环境] 加载完成"

# ── 串口初始化 ────────────────────────────────
log "[串口] 等待 /dev/ttyFC..."
for i in {1..30}; do
    if [ -e "/dev/ttyFC" ]; then
        chmod 666 /dev/ttyFC 2>/dev/null || true
        stty -F /dev/ttyFC 115200 raw -echo -crtscts 2>/dev/null || true
        log "[串口] /dev/ttyFC 就绪 (-> $(readlink -f /dev/ttyFC))"
        break
    fi
    [ "$i" -eq 30 ] && log "[串口] 警告: /dev/ttyFC 未出现，继续..."
    sleep 1
done

# ── 1. USB 摄像头 ─────────────────────────────
log "[摄像头] 启动 hobot_usb_cam (/dev/video0 640x480)..."
ros2 run hobot_usb_cam hobot_usb_cam \
    --ros-args \
    -p video_device:=/dev/video0 \
    -p image_width:=640 \
    -p image_height:=480 \
    > "$LOG_DIR/usb_cam.log" 2>&1 &
log "[摄像头] pid=$!"

log "[摄像头] 等待 /image topic..."
for i in {1..20}; do
    ros2 topic list 2>/dev/null | grep -q "^/image$" && break
    sleep 1
done
ros2 topic list 2>/dev/null | grep -q "^/image$" \
    && log "[摄像头] /image 就绪" \
    || log "[摄像头] 警告: /image 未出现，继续..."

# ── 2. 视觉识别 ───────────────────────────────
log "[视觉] 启动 mission_vision (QR + ONNX)..."
ros2 run camera mission_vision \
    --ros-args \
    -p image_topic:=/image \
    -p cls_model_path:=/home/sunrise/project/camera/resource/cifar100_cls.onnx \
    -p cls_names_path:=/home/sunrise/project/camera/resource/cifar100_names.txt \
    -p target_rank_k:=20 \
    -p target_rank_min_score:=0.004 \
    > "$LOG_DIR/mission_vision.log" 2>&1 &
log "[视觉] pid=$!"
sleep 3

# ── 3. 桥接节点 ───────────────────────────────
log "[桥接] 启动 mission_bridge..."
python3 /home/sunrise/project/mission_bridge.py \
    > "$LOG_DIR/bridge.log" 2>&1 &
log "[桥接] pid=$!"
sleep 1

# ── 4. 任务状态机 ─────────────────────────────
log "[状态机] 启动 task_state_machine..."
python3 /home/sunrise/project/main/main/task_state_machine.py \
    --ros-args -p qr_confirm_frames:=1 -p target_confirm_frames:=1 \
    > "$LOG_DIR/task_sm.log" 2>&1 &
log "[状态机] pid=$!"
sleep 1

# ── 5. 飞控串口桥接 ───────────────────────────
log "[飞控] 启动 fc_bridge (0x01位置帧 + 0x02任务帧)..."
python3 /home/sunrise/project/main/main/fc_bridge.py \
    > "$LOG_DIR/fc_bridge.log" 2>&1 &
log "[飞控] pid=$!"
sleep 1

# ── 6. 舵机投放控制 ──────────────────────────
log "[舵机] 启动 servo_drop..."
python3 /home/sunrise/project/servo_drop.py \
    > "$LOG_DIR/servo_drop.log" 2>&1 &
log "[舵机] pid=$!"
sleep 1

# ── 7. 雷达链路 (有雷达才启) ─────────────────
if ip addr show eth0 2>/dev/null | grep -q "192.168.1."; then
    log "[雷达] 检测到雷达网段，启动雷达链路..."

    ros2 launch livox_ros_driver2 msg_MID360_launch.py \
        > "$LOG_DIR/livox.log" 2>&1 &
    log "[雷达] livox pid=$!"
    sleep 2

    ros2 launch fast_lio mapping.launch.py rviz:=false \
        > "$LOG_DIR/fast_lio.log" 2>&1 &
    log "[雷达] fast_lio pid=$!"
    sleep 4

    ros2 run lidar_fc_cpp mid360_xy_cpp \
        > "$LOG_DIR/mid360_xy.log" 2>&1 &
    log "[雷达] mid360_xy pid=$!"

    ros2 run lidar_fc_cpp relative_pose_cpp \
        > "$LOG_DIR/relative_pose.log" 2>&1 &
    log "[雷达] relative_pose pid=$!"
else
    log "[雷达] 未检测到 192.168.1.x，跳过雷达链路"
fi

# ── 8. watchdog (节点健康监控) ────────────────
log "[watchdog] 启动节点健康监控..."
bash /home/sunrise/project/watchdog.sh \
    > "$LOG_DIR/watchdog.log" 2>&1 &
log "[watchdog] pid=$!"

log "=========================================="
log "  全部节点已启动，日志目录: $LOG_DIR"
log "  tail -f $LOG_DIR/fc_bridge.log       飞控帧"
log "  tail -f $LOG_DIR/fc_tx.log           机载电脑发送给飞控的帧"
log "  tail -f $LOG_DIR/mission_vision.log  视觉识别"
log "  tail -f $LOG_DIR/task_sm.log         任务状态"
log "  tail -f $LOG_DIR/servo_drop.log      舵机投放"
log "  cat  $LOG_DIR/health.log             节点健康"
log "=========================================="

while true; do
    sleep 5
done
