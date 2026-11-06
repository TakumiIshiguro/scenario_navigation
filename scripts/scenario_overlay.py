#!/usr/bin/env python3
"""Draw the scenario executor's progress from /scenario_status as an image."""
import json
import threading

import numpy as np
import rospy
from cv_bridge import CvBridge
from PIL import Image, ImageDraw, ImageFont
from sensor_msgs.msg import Image as RosImage
from std_msgs.msg import String


FONT_PATH = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
WIDTH = 640
BACKGROUND = (255, 255, 255)
TEXT = (30, 30, 30)
MUTED = (130, 130, 130)
FAINT = (232, 232, 232)
ACTIVE = (35, 75, 170)
OK = (35, 150, 70)
WARN = (225, 90, 30)
STALE = (200, 40, 40)

ACTIONS = {
    "go_straight": "直進",
    "turn_right": "右折",
    "turn_left": "左折",
    "stop": "停止",
}
TYPES = {
    "straight_road": "直線",
    "3_way": "三叉路",
    "dead_end": "行き止まり",
    "crossroads": "十字路",
    "end": "突き当り",
    "corner": "角",
    "corridor": "通路",
}
DIRECTIONS = {"left": "左", "right": "右", "front": "前", "back": "後"}


def _font(size):
    try:
        return ImageFont.truetype(FONT_PATH, size)
    except OSError:
        return ImageFont.load_default()


def step_text(step):
    """Return one scenario step in the wording of the scenario files."""
    action = ACTIONS.get(step["action"], step["action"])
    if step["type"] == "nothing":
        return action
    target = TYPES.get(step["type"], step["type"])
    if step["direction"] != "nothing":
        target = DIRECTIONS.get(step["direction"], step["direction"]) + "の" + target
    if step["order"] > 1:
        target = f'{step["order"]}つ目の' + target
    return f"{target}まで{action}"


def _bar(draw, top, label, value, maximum, color, markers=()):
    left, right = 24, WIDTH - 24
    span = right - left
    draw.text((left, top), label, fill=TEXT, font=_font(17))
    draw.rectangle([left, top + 28, right, top + 46], fill=FAINT)
    ratio = min(value / maximum, 1.0) if maximum > 0.0 else 0.0
    if ratio > 0.0:
        draw.rectangle(
            [left, top + 28, left + int(span * ratio), top + 46], fill=color
        )
    for position, text in markers:
        x = left + int(span * position / maximum)
        draw.line([x, top + 24, x, top + 50], fill=TEXT, width=2)
        draw.text((x - 20, top + 52), text, fill=MUTED, font=_font(13))


def render_scenario_status(status, stale=False):
    """Return the overlay for one /scenario_status message, or a wait screen."""
    steps = status["steps"] if status else []
    height = 300 + 46 * len(steps)
    canvas = Image.new("RGB", (WIDTH, height), BACKGROUND)
    draw = ImageDraw.Draw(canvas)
    if status is None:
        draw.text((24, 20), "/scenario_status 待ち", fill=TEXT, font=_font(30))
        return canvas

    mode = status["mode"]
    if stale:
        draw.text((24, 58), "STALE", fill=STALE, font=_font(16))
    if status["care_active"]:
        draw.rectangle([WIDTH - 130, 18, WIDTH - 24, 56], fill=WARN)
        draw.text((WIDTH - 104, 20), "CARE", fill="white", font=_font(24))

    top = 88
    for index, step in enumerate(steps):
        finished = index < status["step"] or mode == "COMPLETE"
        current = index == status["step"] and mode != "COMPLETE"
        if current:
            draw.rectangle([16, top, WIDTH - 16, top + 40], fill=ACTIVE)
        color = "white" if current else MUTED if finished else TEXT
        mark = "✓" if finished else f"{index + 1}"
        draw.text((30, top + 4), mark, fill=color, font=_font(21))
        draw.text((64, top + 4), step_text(step), fill=color, font=_font(21))
        if current and step["type"] != "nothing":
            draw.text(
                (WIDTH - 130, top + 4),
                f'{status["order"]} / {step["order"]}回',
                fill=color,
                font=_font(21),
            )
        top += 46

    top += 10
    current_node = status["current_node"]
    labels = status["target_labels"]
    matched = current_node in labels
    draw.text((24, top), "通路", fill=MUTED, font=_font(17))
    draw.text(
        (84, top - 6),
        current_node,
        fill=OK if matched else TEXT,
        font=_font(26),
    )
    top += 36
    draw.text((24, top), "受理", fill=MUTED, font=_font(17))
    draw.text(
        (84, top + 2),
        ", ".join(labels) if labels else "-",
        fill=TEXT,
        font=_font(14),
    )
    top += 36

    if status["target_guard"] and mode != "TURNING":
        _bar(
            draw,
            top,
            f'目標の無視 {status["target_guard_travelled_m"]:.2f} / '
            f'{status["target_guard_distance_m"]:.2f} m',
            status["target_guard_travelled_m"],
            status["target_guard_distance_m"],
            WARN,
        )
    else:
        maximum = status["turn_max_angle_deg"]
        _bar(
            draw,
            top,
            f'旋回角度 {status["turn_angle_deg"]:.0f} / {maximum:g} deg',
            status["turn_angle_deg"],
            maximum,
            ACTIVE,
        )
    return canvas


class ScenarioOverlay:
    def __init__(self):
        self.stale_seconds = float(rospy.get_param("~stale_seconds", 1.0))
        self.lock = threading.Lock()
        self.status = None
        self.stamp = None
        self.bridge = CvBridge()
        self.publisher = rospy.Publisher(
            rospy.get_param("~output_topic", "/scenario_navigation/overlay"),
            RosImage,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param("~status_topic", "/scenario_status"),
            String,
            self._status_callback,
            queue_size=1,
        )

    def _status_callback(self, message):
        status = json.loads(message.data)
        with self.lock:
            self.status = status
            self.stamp = rospy.get_time()

    def publish(self):
        with self.lock:
            status = self.status
            stamp = self.stamp
        stale = (
            status is not None
            and status["mode"] != "COMPLETE"
            and rospy.get_time() - stamp > self.stale_seconds
        )
        image = render_scenario_status(status, stale=stale)
        self.publisher.publish(
            self.bridge.cv2_to_imgmsg(np.asarray(image), encoding="rgb8")
        )


def main():
    rospy.init_node("scenario_overlay")
    node = ScenarioOverlay()
    rate = rospy.Rate(float(rospy.get_param("~rate", 4.0)))
    while not rospy.is_shutdown():
        node.publish()
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
