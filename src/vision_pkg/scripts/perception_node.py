#!/usr/bin/env python3
"""配对同一帧的检测与图像，再交给原感知模块处理。"""
from collections import OrderedDict

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point32
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image

from robot_interfaces.msg import Object, Objects, Region, SafetyRegions, State
from vision_pkg.detect.get_objs import GetObjs

COLORS = {'green': (65,190,80), 'orange': (0,140,255), 'black': (160,100,230),
          'blue': (255,140,50), 'rsafe': (60,70,235), 'bsafe': (200,120,60)}


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
            'safe_split': {'supplies_on_left': self.declare_parameter('supplies_on_left', True).value},
        }
        self.settle = self.declare_parameter('camera_settle_ms', 400).value / 1000
        self.processor = None
        self.size = None
        self.robot = State()
        self.frames = OrderedDict()
        self.detections = OrderedDict()
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.preview = self.create_publisher(Image, 'vision/preview', qos)
        self.objects = self.create_publisher(Objects, 'vision/objects', qos)
        self.regions = self.create_publisher(SafetyRegions, 'vision/safety_regions', qos)
        # callback：开了推理就分别收原图和检测；只开预览时把相机图直接转发。
        if self.declare_parameter('detect', False).value:
            self.create_subscription(Image, 'vision/inference_image',
                                     lambda image: self.pair(image, self.frames, self.detections), qos)
            self.create_subscription(Objects, 'vision/detections',
                                     lambda det: self.pair(det, self.detections, self.frames), qos)
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

    def pair(self, message, pending, other):
        """按帧配对原图和检测，先到的一方放进缓存。

        Args:
            message: 本次 callback 收到的图像或检测消息。
            pending: 和本条消息同类型的缓存。
            other: 另一种消息的缓存，从中找同一帧。
        """
        header = message.header
        key = (header.stamp.sec, header.stamp.nanosec, header.frame_id)
        match = other.pop(key, None)
        if match is None:
            pending[key] = message
            # 另一条话题可能丢帧，只保留最近四条，免得永远等不到的消息越攒越多。
            if len(pending) > 4:
                pending.popitem(last=False)
            return
        if isinstance(message, Image):
            self.process(message, match)
        else:
            self.process(match, message)

    def process(self, image, detections):
        width, height = image.width, image.height
        if (image.encoding != 'bgr8' or not width or not height or image.step < width * 3 or
                len(image.data) != image.step * height):
            return
        if self.size != (width,height):
            self.processor = GetObjs(self.config, width=width, height=height)
            self.size = (width,height)
        processor = self.processor
        # 把 ROS 消息拆回旧算法的三个输入：框坐标、类别编号和置信度。
        boxes, classes, scores = [], [], []
        for obj in detections.items:
            if obj.name in processor.labels:
                boxes.append((obj.x1,obj.y1,obj.x2,obj.y2))
                classes.append(processor.labels.index(obj.name))
                scores.append(obj.score)
        frame = np.frombuffer(image.data, np.uint8).reshape(height,image.step)
        frame = frame[:, :width * 3].reshape(height,width,3).copy()
        # 用拍摄时间和舵机指令时间相减，判断拍下这帧时相机是否已经稳定。
        ready = (not self.robot.camera_known or
                 (Time.from_msg(image.header.stamp) - Time.from_msg(self.robot.camera_changed)).nanoseconds >= self.settle * 1e9)
        angle = self.robot.camera_deg if self.robot.camera_known else None
        objects, regions = processor.process(frame, boxes, classes, scores, self.robot.team, ready, angle)

        output = Objects(header=image.header)
        for obj in objects:
            x1,y1,x2,y2 = map(float,obj['box'])
            output.items.append(Object(name=obj['name'], id=obj['id'], score=obj['score'],
                                       x1=x1,y1=y1,x2=x2,y2=y2, observed=obj['observed'],
                                       in_safe_zone=obj['in_safe_zone']))
            if obj['observed']:
                color = COLORS.get(obj['name'], (65,190,80))
                cv2.rectangle(frame, (int(x1),int(y1)), (int(x2),int(y2)), color, 2)
                cv2.putText(frame, obj['name'], (int(x1),max(15,int(y1)-4)),
                            cv2.FONT_HERSHEY_SIMPLEX,.6,color,1)
        cutoff = processor.background_filter.last['cutoff_y']
        if cutoff is not None and 0 < cutoff < height:
            cv2.line(frame, (0,int(cutoff)), (width-1,int(cutoff)), (0,255,255), 1)
        zones = SafetyRegions(header=image.header)
        for region in regions:
            polygon = region['polygon']
            zones.regions.append(Region(name=region.get('name',''), polygon=[
                Point32(x=float(x),y=float(y)) for x,y in polygon]))
            cv2.polylines(frame, [np.asarray(polygon,dtype=np.int32)], True, (255,210,40), 2)

        # 动作节点读物块，安全区单独发布，HTTP 预览用画好框和线的图。
        self.objects.publish(output)
        self.regions.publish(zones)
        image.step = width * 3
        image.data = frame.tobytes()
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
