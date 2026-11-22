#!/usr/bin/env python3
"""运行原 Python YOLO，发布与原图时间戳一致的检测。"""
import numpy as np
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from robot_interfaces.msg import Object, Objects
from vision_pkg.inference.yolov8 import YOLOv8
from vision_pkg.inference.yolov8_onnx import YOLOv8Onnx


class InferenceNode(Node):
    """接收相机画面，调用原来的 YOLO，把检测框和对应原图分别发出去。"""
    def __init__(self):
        super().__init__('inference_node')
        path = self.declare_parameter('yolo.model_path', '').value
        self.declare_parameter('yolo.conf_threshold', .6)
        self.declare_parameter('yolo.iou_threshold', .45)
        # callback：桌面端改置信度时会先走 validate_settings，再决定是否接受。
        self.add_on_set_parameters_callback(self.validate_settings)
        if path.lower().endswith('.rknn'):
            self.detector = YOLOv8(path)
            self.labels = ['orange', 'blue', 'green', 'black', 'bsafe', 'rsafe']
        elif path.lower().endswith('.onnx'):
            self.detector = YOLOv8Onnx(path)
            self.labels = self.detector.labels
        else:
            raise ValueError('[YOLO] model 必须是 .rknn 或 .onnx 文件')
        # 只排队一帧，模型来不及推理时优先跟上最新画面。
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.objects = self.create_publisher(Objects, 'vision/detections', qos)
        self.frames = self.create_publisher(Image, 'vision/inference_image', qos)
        # callback：camera/image 到一帧就调用 detect()。
        self.create_subscription(Image, 'camera/image', self.detect, qos)

    @staticmethod
    def validate_settings(parameters):
        for parameter in parameters:
            if parameter.name in ('yolo.conf_threshold', 'yolo.iou_threshold'):
                if not isinstance(parameter.value, float) or not 0 <= parameter.value <= 1:
                    return SetParametersResult(successful=False, reason='阈值需要在 0～1 之间')
            if parameter.name == 'yolo.model_path':
                return SetParametersResult(successful=False, reason='换模型需要重启')
        return SetParametersResult(successful=True)

    def detect(self, message):
        if (message.encoding != 'bgr8' or not message.width or not message.height or
                message.step < message.width * 3 or len(message.data) != message.height * message.step):
            return
        frame = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
        frame = frame[:, :message.width * 3].reshape(message.height, message.width, 3)
        # 每帧都重新读参数，桌面端改完阈值，后续推理就会用新值。
        confidence = self.get_parameter('yolo.conf_threshold').value
        iou = self.get_parameter('yolo.iou_threshold').value
        boxes, classes, scores = self.detector.predict(frame, conf=confidence, iou=iou)
        # 原图和检测共用 camera_node 给的 header，感知节点据此配成一对。
        result = Objects(header=message.header)
        for box, index, score in zip(boxes, classes, scores):
            if not 0 <= index < len(self.labels):
                continue
            x1, y1, x2, y2 = map(float, box)
            result.items.append(Object(name=self.labels[index], score=float(score), x1=x1, y1=y1,
                                       x2=x2, y2=y2, observed=True))
        self.frames.publish(message)
        self.objects.publish(result)


def main():
    rclpy.init()
    node = None
    try:
        node = InferenceNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if isinstance(node.detector, YOLOv8):
                node.detector.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
