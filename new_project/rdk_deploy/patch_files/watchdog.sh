#!/bin/bash
# 节点健康监控 watchdog
# 每5秒检查各进程是否存活，写入 health.log
# fc_bridge 崩溃时自动重启（无状态节点）
# 其他节点崩溃只告警不重启（有状态，重启会丢失任务信息）

LOG_DIR="/tmp/flight_logs"
HEALTH_LOG="$LOG_DIR/health.log"

log_health() { echo "[$(date '+%H:%M:%S')] $1" | tee -a "$HEALTH_LOG"; }

# 等待主链路启动完毕
sleep 15

log_health "=========================================="
log_health "  watchdog 启动"
log_health "=========================================="

# 记录 fc_bridge 重启所需环境和命令（与 start_all.sh 保持一致）
FC_BRIDGE_CMD="python3 /home/sunrise/project/main/main/fc_bridge.py"

restart_fc_bridge() {
    log_health "[重启] fc_bridge 已崩溃，正在重启..."
    source /opt/ros/humble/setup.bash
    source /opt/tros/humble/setup.bash
    source /home/sunrise/project/install/setup.bash
    export PYTHONPATH="/home/sunrise/project/install/main/lib/python3.10/site-packages:$PYTHONPATH"
    $FC_BRIDGE_CMD >> "$LOG_DIR/fc_bridge.log" 2>&1 &
    log_health "[重启] fc_bridge 新 pid=$!"
}

while true; do
    STATUS=""
    ALL_OK=true

    # ── 检查各节点 ──────────────────────────────
    check_node() {
        local name="$1"
        local pattern="$2"
        if pgrep -f "$pattern" > /dev/null 2>&1; then
            STATUS="${STATUS}  ✓ ${name}\n"
        else
            STATUS="${STATUS}  ✗ ${name} [DOWN]\n"
            ALL_OK=false
            log_health "[警告] ${name} 进程不存在！"
            # 写入 startup.log 让主日志也能看到
            echo "[$(date '+%H:%M:%S')] [watchdog警告] ${name} 已崩溃" >> "$LOG_DIR/startup.log"
        fi
    }

    check_node "hobot_usb_cam"    "hobot_usb_cam"
    check_node "mission_vision"   "mission_vision"
    check_node "mission_bridge"   "mission_bridge"
    check_node "task_state_machine" "task_state_machine"
    check_node "fc_bridge"        "fc_bridge.py"

    # ── 写入 health.log ─────────────────────────
    {
        echo "[$(date '+%H:%M:%S')] ── 节点状态 ──"
        echo -e "$STATUS"
        if $ALL_OK; then
            echo "[$(date '+%H:%M:%S')] 全部节点正常"
        fi
        echo ""
    } >> "$HEALTH_LOG"

    # ── fc_bridge 自动重启 ───────────────────────
    if ! pgrep -f "fc_bridge.py" > /dev/null 2>&1; then
        restart_fc_bridge
        sleep 3
    fi

    sleep 5
done
