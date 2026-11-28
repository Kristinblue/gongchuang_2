#!/usr/bin/env python3
"""配对同一帧的检测与图像，再交给原感知模块处理。"""
from array import array
from collections import OrderedDict

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image

from robot_interfaces.msg import Object, Objects, SafetyRegions, State
from vision_pkg.detect.get_objs import GetObjs
from vision_pkg.detect.objs import bottom_center

COLORS = {'green': (65,190,80), 'orange': (0,140,255), 'black': (160,100,230),
          'blue': (255,140,50)}


def orange_at_claw(frame, box):
    """低头时，触及画面底边的大橙块框需要有实际橙色像素。"""
    height, width = frame.shape[:2]
    if box.y2 < .99*height or box.y2-box.y1 < .25*height:
        return True
    x1, y1 = max(0, int(box.x1)), max(0, int(box.y1))
    x2, y2 = min(width, int(box.x2)+1), min(height, int(box.y2)+1)
    roi = frame[y1:y2:4, x1:x2:4]
    if not roi.size:
        return True
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (5, 75, 55), (28, 255, 255))
    return cv2.countNonZero(mask) >= .04 * mask.size


class PerceptionNode(Node):
    """用原工程的 GetObjs 过滤物块、跟踪目标并划分安全区。"""
    def __init__(self):
        super().__init__('perception_node')
        # 把 ROS 参数整理成 GetObjs 原来接收的配置结构。
        self.config = {
            'background_filter': {
                'enabled': self.declare_parameter('background_filter.enabled', True).value,
                'horizontal_fov_deg': self.declare_parameter('background_filter.horizontal_fov_deg', 70.).value,
                'horizon_margin_px': self.declare_parameter('background_filter.horizon_margin_px', 16.).value,
            },
            'tracking': {
                'max_dist_px': self.declare_parameter('tracking.max_dist_px', 150).value,
                'lost_frames': self.declare_parameter('tracking.lost_frames', 6).value,
            },
        }
        self.settle = self.declare_parameter('camera_settle_ms', 400).value / 1000
        self.processor = None
        self.size = None
        self.robot = State()
        self.frames = OrderedDict()
        self.detections = OrderedDict()
        self.safety = OrderedDict()
        self.last_key = None
        self.pair_safety = self.declare_parameter('pair_safety', False).value
        qos = QoSProfile(depth=4, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.preview = self.create_publisher(Image, 'vision/preview', qos)
        self.objects = self.create_publisher(Objects, 'vision/objects', qos)
        self.regions = self.create_publisher(SafetyRegions, 'vision/safety_regions', qos)
        # callback：开了推理就分别收原图和检测；只开预览时把相机图直接转发。
        if self.declare_parameter('detect', False).value:
            self.create_subscription(Image, 'vision/inference_image',
                                     lambda image: self.pair(image, self.frames), qos)
            self.create_subscription(Objects, 'vision/detections',
                                     lambda det: self.pair(det, self.detections), qos)
            if self.pair_safety:
                self.create_subscription(SafetyRegions, 'vision/safety_regions_raw',
                                         lambda zones: self.pair(zones, self.safety), qos)
        else:
            self.create_subscription(Image, 'camera/image', self.preview.publish, qos)
        self.create_subscription(State, 'robot/state', self.robot_state,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def robot_state(self, state):
        """相机角度或队伍变化时，清掉依赖旧视角或旧队伍的感知状态。"""
        if self.processor and (state.camera_changed != self.robot.camera_changed or
                               state.team != self.robot.team):
            self.processor.reset()
        self.robot = state

    def pair(self, message, pending):
        """用时间戳配对原图、物块检测和 OpenCV 安全区。"""
        header = message.header
        key = (header.stamp.sec, header.stamp.nanosec, header.frame_id)
        if self.last_key is not None and key <= self.last_key:
            return
        pending[key] = message
        # 另一条话题可能丢帧，只保留最近四条，免得永远等不到的消息越攒越多。
        if len(pending) > 4:
            pending.popitem(last=False)
        if key in self.frames and key in self.detections and (not self.pair_safety or key in self.safety):
            self.last_key = key
            self.process(self.frames.pop(key), self.detections.pop(key),
                         self.safety.pop(key, SafetyRegions(header=message.header)))

    def process(self, image, detections, zones):
        width, height = image.width, image.height
        if (image.encoding != 'bgr8' or not width or not height or image.step < width * 3 or
                len(image.data) != image.step * height):
            return
        if self.size != (width,height):
            self.processor = GetObjs(self.config, width=width, height=height)
            self.size = (width,height)
        processor = self.processor
        frame = np.frombuffer(image.data, np.uint8).reshape(height,image.step)
        frame = frame[:, :width * 3].reshape(height,width,3).copy()
        # 用拍摄时间和舵机指令时间相减，判断拍下这帧时相机是否已经稳定。
        ready = (not self.robot.camera_known or
                 (Time.from_msg(image.header.stamp) - Time.from_msg(self.robot.camera_changed)).nanoseconds >= self.settle * 1e9)
        angle = self.robot.camera_deg if self.robot.camera_known else None
        # 把 ROS 消息拆回旧算法的三个输入：框坐标、类别编号和置信度。
        boxes, classes, scores = [], [], []
        for obj in detections.items:
            if obj.name in COLORS and obj.name in processor.labels:
                if ready and angle == 45 and obj.name == 'orange' and not orange_at_claw(frame, obj):
                    continue
                boxes.append((obj.x1,obj.y1,obj.x2,obj.y2))
                classes.append(processor.labels.index(obj.name))
                scores.append(obj.score)
        objects, _ = processor.process(frame, boxes, classes, scores, self.robot.team, ready, angle)
        polygons = [np.asarray([(p.x,p.y) for p in region.polygon], dtype=np.float32)
                    for region in zones.regions if len(region.polygon) >= 3]

        output = Objects(header=image.header)
        for obj in objects:
            x1,y1,x2,y2 = map(float,obj['box'])
            in_zone = any(cv2.pointPolygonTest(polygon, bottom_center(obj), False) >= 0
                          for polygon in polygons)
            output.items.append(Object(name=obj['name'], id=obj['id'], score=obj['score'],
                                       x1=x1,y1=y1,x2=x2,y2=y2, observed=obj['observed'],
                                       in_safe_zone=in_zone))
            if obj['observed'] and not in_zone:
                color = COLORS.get(obj['name'], (65,190,80))
                cv2.rectangle(frame, (int(x1),int(y1)), (int(x2),int(y2)), color, 2)
                cv2.putText(frame, obj['name'], (int(x1),max(15,int(y1)-4)),
                            cv2.FONT_HERSHEY_SIMPLEX,.6,color,1)
        cutoff = processor.background_filter.last['cutoff_y']
        if cutoff is not None and 0 < cutoff < height:
            cv2.line(frame, (0,int(cutoff)), (width-1,int(cutoff)), (0,255,255), 1)
        for region in zones.regions:
            # 供搜寻阶段评分的标志来自 OpenCV，物块排除只使用上面的多边形。
            output.items.append(Object(name=region.name, observed=True))
        for polygon in polygons:
            cv2.polylines(frame, [polygon.astype(np.int32)], True, (255,210,40), 2)

        # 动作节点读物块，安全区单独发布，HTTP 预览用画好框和线的图。
        self.objects.publish(output)
        self.regions.publish(zones)
        image.step = width * 3
        # ROS 的 uint8[] 直接接收 array，避免按像素检查整个预览帧。
        image.data = array('B', frame.tobytes())
        self.preview.publish(image)


def main():
    rclpy.init()
    node = None
    try:
        node = PerceptionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
