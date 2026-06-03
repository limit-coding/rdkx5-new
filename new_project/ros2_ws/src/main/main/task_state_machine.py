import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Int32MultiArray, String


class TaskStateMachine(Node):
    """Mission state machine for the QR -> YOLO competition flow only."""

    def __init__(self):
        super().__init__("task_state_machine")

        self.declare_parameter("qr_confirm_frames", 1)
        self.declare_parameter("target_confirm_frames", 3)
        self.declare_parameter("target_disappear_frames", 5)
        self.declare_parameter("target_max_count", 4)
        self.declare_parameter(
            "ignored_labels",
            ["ring", "landing_h", "red_light", "blue_light", "obstacle", "cloud", "keyboard"],
        )

        self.qr_confirm_frames = int(self.get_parameter("qr_confirm_frames").value)
        self.target_confirm_frames = int(self.get_parameter("target_confirm_frames").value)
        self.target_disappear_frames = int(self.get_parameter("target_disappear_frames").value)
        self.target_max_count = int(self.get_parameter("target_max_count").value)
        self.ignored_labels = {
            str(label).strip().lower()
            for label in self.get_parameter("ignored_labels").value
            if str(label).strip()
        }

        self.pic_enable_pub = self.create_publisher(Int32, "/pic_enable", 10)
        self.qr_enable_pub = self.create_publisher(Int32, "/qr_enable", 10)
        self.task_status_pub = self.create_publisher(Int32MultiArray, "/task_status", 10)

        self.qr_text_sub = self.create_subscription(
            String,
            "/qr_code/text",
            self.qr_text_callback,
            10,
        )
        self.vision_detections_sub = self.create_subscription(
            String,
            "/vision/detections",
            self.vision_detections_callback,
            10,
        )
        self.pr_select_sub = self.create_subscription(
            Int32,
            "/pr_select",
            self.pr_select_callback,
            10,
        )

        self.task_state = 0x01
        self.landing_state = 0x01
        self.target_classes = set()
        self.landing_side = ""
        self.qr_task_ready = False
        self._yolo_active = False
        self._current_fc_cmd = 0  # 当前飞控命令 2/3/4/5，决定发哪个 task_state
        self._cmds_done = set()   # 已经发出结果的命令，重复收到直接忽略

        self._qr_candidate_text = ""
        self._qr_candidate_count = 0

        self.target_candidate_label = ""
        self.target_candidate_count = 0
        self.locked_target_label = ""
        self.locked_target_missing_count = 0
        self.recognized_target_count = 0

        self.task_status_timer = self.create_timer(0.1, self.publish_task_status)

        self.publish_qr_enable(True)
        self.publish_yolo_enable(False)
        self.publish_task_status()
        self.get_logger().info(
            "task state machine ready: task_state=0x01, landing_state=0x01"
        )

    def publish_task_status(self):
        msg = Int32MultiArray()
        msg.data = [int(self.task_state), int(self.landing_state)]
        self.task_status_pub.publish(msg)

    def set_task_status(self, task_state=None, landing_state=None):
        changed = False
        if task_state is not None and self.task_state != int(task_state):
            self.task_state = int(task_state)
            changed = True
        if landing_state is not None and self.landing_state != int(landing_state):
            self.landing_state = int(landing_state)
            changed = True
        if changed:
            self.publish_task_status()
            self.get_logger().info(
                f"task status updated: task_state=0x{self.task_state:02X}, "
                f"landing_state=0x{self.landing_state:02X}"
            )

    def publish_qr_enable(self, enabled):
        msg = Int32()
        msg.data = 1 if enabled else 0
        self.qr_enable_pub.publish(msg)

    def publish_yolo_enable(self, enabled):
        msg = Int32()
        msg.data = 1 if enabled else 0
        self.pic_enable_pub.publish(msg)

    def parse_qr_task_text(self, text):
        value = text.strip()
        if not value:
            return None

        replacements = {
            "，": ",",
            "、": ",",
            "；": ";",
            "：": ":",
            ";": ",",
            "|": ",",
            "/": ",",
            "\\": ",",
            "[": " ",
            "]": " ",
            "{": " ",
            "}": " ",
            "(": " ",
            ")": " ",
            '"': " ",
            "'": " ",
            "=": " ",
            ":": " ",
        }
        for old, new in replacements.items():
            value = value.replace(old, new)

        ignored_tokens = {
            "class",
            "class1",
            "class2",
            "target",
            "target1",
            "target2",
            "side",
            "landing",
            "landing_side",
            "land",
            "qr",
            "task",
        }

        tokens = []
        for raw in value.replace(",", " ").split():
            token = raw.strip().lower()
            if token and token not in ignored_tokens:
                tokens.append(token)

        landing_side = ""
        classes = []
        for token in tokens:
            if token in ("left", "right"):
                landing_side = token
            else:
                classes.append(token)

        if len(classes) < 2 or not landing_side:
            return None
        return classes[0], classes[1], landing_side

    def update_qr_task_candidate(self, text):
        task = self.parse_qr_task_text(text)
        if task is None:
            self.get_logger().warning(f"invalid QR task text: {text}")
            return

        normalized = f"{task[0]},{task[1]},{task[2]}"
        if normalized == self._qr_candidate_text:
            self._qr_candidate_count += 1
        else:
            self._qr_candidate_text = normalized
            self._qr_candidate_count = 1

        if self._qr_candidate_count < self.qr_confirm_frames:
            return

        if (
            self.qr_task_ready
            and self.target_classes == {task[0], task[1]}
            and self.landing_side == task[2]
        ):
            return

        self.target_classes = {task[0], task[1]}
        self.landing_side = task[2]
        self.qr_task_ready = True

        landing_state = 0x02 if self.landing_side == "left" else 0x03
        self.set_task_status(task_state=0x02, landing_state=landing_state)
        self.publish_qr_enable(False)
        self.get_logger().info(
            f"QR task confirmed: target_classes={sorted(self.target_classes)}, "
            f"landing_side={self.landing_side}, 等待飞控触发识别命令"
        )

    def pr_select_callback(self, msg):
        cmd = int(msg.data)
        if cmd not in (2, 3, 4, 5):
            return
        if not self.qr_task_ready:
            self.get_logger().warn(f'飞控发命令 {cmd} 但二维码尚未确认，忽略')
            return
        if self._yolo_active:
            return
        if cmd in self._cmds_done:  # 已经发过结果，忽略重复命令
            return
        if self._yolo_active:
            return
        self.clear_target_lock()
        self._current_fc_cmd = cmd
        self._yolo_active = True
        self.publish_yolo_enable(True)
        self.get_logger().info(f'飞控命令 {cmd} → 开始识别第 {cmd - 1} 个靶子')

    def qr_text_callback(self, msg):
        self.update_qr_task_candidate(msg.data)

    def vision_detections_callback(self, msg):
        if not self.qr_task_ready:
            return
        if not self._yolo_active:
            return

        try:
            result = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning(f"invalid vision detections JSON: {msg.data}")
            return

        detections = result.get("detections", [])
        if not isinstance(detections, list):
            self.get_logger().warning(f"invalid vision detections payload: {msg.data}")
            return

        self.update_yolo_task_state(detections)

    def pick_best_task_detection(self, detections):
        candidates = []
        for det in detections:
            if not isinstance(det, dict):
                continue
            class_name = str(det.get("class_name", "")).strip().lower()
            if not class_name or class_name in self.ignored_labels:
                continue
            candidates.append(det)

        if not candidates:
            return None

        return max(
            candidates,
            key=lambda det: (
                float(det.get("score", 0.0)),
                int(det.get("area", 0)),
            ),
        )

    def update_yolo_task_state(self, detections):
        best = self.pick_best_task_detection(detections)
        if best is None:
            self.handle_no_task_target()
            return

        label = str(best.get("class_name", "")).strip().lower()
        if not label:
            self.handle_no_task_target()
            return

        is_target = self.label_matches_target(label)

        if self.locked_target_label:
            locked_is_target = self.label_matches_target(self.locked_target_label)
            if label == self.locked_target_label:
                self.locked_target_missing_count = 0
                return
            if is_target and not locked_is_target:
                self.correct_locked_target(label)
                return
            self.locked_target_missing_count += 1
            if self.locked_target_missing_count < self.target_disappear_frames:
                return
            self.clear_target_lock()

        if label == self.target_candidate_label:
            self.target_candidate_count += 1
        else:
            self.target_candidate_label = label
            self.target_candidate_count = 1

        if self.target_candidate_count < self.target_confirm_frames:
            return

        self.confirm_new_task_target(label)

    def handle_no_task_target(self):
        self.target_candidate_label = ""
        self.target_candidate_count = 0
        if not self.locked_target_label:
            return
        self.locked_target_missing_count += 1
        if self.locked_target_missing_count >= self.target_disappear_frames:
            self.clear_target_lock()

    def clear_target_lock(self):
        self.locked_target_label = ""
        self.locked_target_missing_count = 0
        self.target_candidate_label = ""
        self.target_candidate_count = 0

    def label_matches_target(self, label):
        label = str(label).strip().lower()
        if not label:
            return False
        return any(
            target == label or target in label or label in target
            for target in self.target_classes
        )

    def confirm_new_task_target(self, label):
        if self.recognized_target_count >= self.target_max_count:
            return

        self.recognized_target_count += 1
        is_target = self.label_matches_target(label)
        # cmd=2→03/04, cmd=3→05/06, cmd=4→07/08, cmd=5→09/0A
        base = 0x03 + (self._current_fc_cmd - 2) * 2
        task_state = base + (1 if is_target else 0)

        self.set_task_status(task_state=task_state)
        self.locked_target_label = label
        self.locked_target_missing_count = 0
        self.target_candidate_label = ""
        self.target_candidate_count = 0
        self._yolo_active = False
        self._cmds_done.add(self._current_fc_cmd)
        self.publish_yolo_enable(False)
        self.get_logger().info(
            f"YOLO target confirmed: index={self.recognized_target_count}, "
            f"label={label}, match={is_target}, task_state=0x{task_state:02X}, 等待飞控下一个命令"
        )

    def correct_locked_target(self, label):
        if self.recognized_target_count <= 0:
            self.confirm_new_task_target(label)
            return

        base = 0x03 + (self._current_fc_cmd - 2) * 2
        task_state = base + 1  # 修正为匹配
        self.set_task_status(task_state=task_state)
        old_label = self.locked_target_label
        self.locked_target_label = label
        self.locked_target_missing_count = 0
        self.target_candidate_label = ""
        self.target_candidate_count = 0
        self._yolo_active = False
        self._cmds_done.add(self._current_fc_cmd)
        self.publish_yolo_enable(False)
        self.get_logger().info(
            f"YOLO target corrected: index={self.recognized_target_count}, "
            f"old_label={old_label}, label={label}, task_state=0x{task_state:02X}, 等待飞控下一个命令"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TaskStateMachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
