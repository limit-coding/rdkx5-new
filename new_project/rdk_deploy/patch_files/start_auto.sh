#!/bin/bash
# =============================================================================
# 自动飞行数据链路启动脚本
# 功能：开机自启动时拉起整条数据链路
#   Livox MID360 -> FAST_LIO -> mid360_xy -> relative_pose -> fc_bridge(串口)
# =============================================================================

set -e

# 日志输出带时间戳
log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1"
}

log "=========================================="
log "  Auto Flight Data Link 启动"
log "=========================================="

# --- 1. 等待网络就绪 ---
log "[网络] 检查 eth0 网络状态..."
for i in {1..30}; do
    if ip addr show eth0 | grep -q "192.168.1."; then
        log "[网络] eth0 已就绪: $(ip addr show eth0 | grep 'inet ' | awk '{print $2}')"
        break
    fi
    if [ "$i" -eq 30 ]; then
        log "[网络] 警告: eth0 未获取到 192.168.1.x 地址，继续尝试启动..."
    fi
    sleep 1
done

# --- 2. 等待系统时间有效 ---
# FAST_LIO 依赖雷达/IMU 时间连续。RDK 上电时可能先是 2000-01-01，
# 等热点/NTP 同步后时间跳到真实年份，会导致里程计瞬间发散。
MIN_VALID_EPOCH=1704067200  # 2024-01-01 00:00:00 UTC
log "[时间] 等待系统时间同步到有效年份..."
for i in {1..180}; do
    now_epoch=$(date +%s)
    if [ "$now_epoch" -ge "$MIN_VALID_EPOCH" ]; then
        log "[时间] 系统时间有效: $(date '+%Y-%m-%d %H:%M:%S')"
        break
    fi
    if [ "$i" -eq 180 ]; then
        log "[时间] 错误: 系统时间仍无效: $(date '+%Y-%m-%d %H:%M:%S')，退出等待 systemd 重启"
        exit 1
    fi
    sleep 1
done

# --- 3. 加载 ROS2 Humble ---
log "[环境] 加载 ROS2 Humble..."
source /opt/ros/humble/setup.bash

# --- 4. 加载 lidar_ws（livox + fast_lio）---
log "[环境] 加载 lidar_ws..."
source /home/sunrise/lidar_ws/install/setup.bash

# --- 5. 加载 project ---
log "[环境] 加载 project..."
source /home/sunrise/project/install/setup.bash

# --- 6. 等待并初始化串口设备 ---
log "[串口] 等待 /dev/ttyFC 就绪..."
udevadm settle --timeout=15 || true
for i in {1..60}; do
    if [ -e "/dev/ttyFC" ]; then
        log "[串口] 检测到 /dev/ttyFC -> $(readlink -f /dev/ttyFC)"
        chmod 666 /dev/ttyFC 2>/dev/null || true
        stty -F /dev/ttyFC 115200 raw -echo -crtscts 2>/dev/null || true
        break
    fi
    if [ "$i" -eq 60 ]; then
        log "[串口] 错误: /dev/ttyFC 60 秒内未出现，退出等待 systemd 重启"
        exit 1
    fi
    sleep 1
done

# --- 7. 启动 launch 文件 ---
log "[启动] 启动 auto_flight.launch.py..."
log "[启动] 链路: Livox -> FAST_LIO -> mid360_xy -> relative_pose -> fc_bridge"
exec ros2 launch /home/sunrise/project/auto_flight.launch.py
