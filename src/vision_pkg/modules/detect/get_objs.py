"""
这个是对外暴露的得到比较纯净的物体列表，都是Objs类的实例
这里经过背景过滤
"""

import numpy as np

from ..my_filter.bg_filter import BackgroundFilter
from .objs import Objs, to_targets


class GetObjs:
    """统一完成模型推理、过滤和物体记忆，run返回物体字典列表。"""

    def __init__(self, config, detector=None, width=1280, height=720):
        self.config = config.get('yolo',{})
        self.detector = detector
        self._owns_detector = detector is None
        self.labels = list(getattr(detector,'labels',['orange','blue','green','black','bsafe','rsafe']))
        tracking = config.get('tracking',{})
        self.objects = Objs(width,height,tracking.get('max_dist_px',150),tracking.get('lost_frames',6))
        self.background_filter = BackgroundFilter(**config.get('background_filter',{}))
        self.detections = (np.empty((0,4)),np.empty(0,dtype=int),np.empty(0))

    def run(self, frame, team=None, ready=True, camera_deg=None):
        """每帧调用；相机未稳定时不沿用上一帧的像素记忆。"""
        if self.detector is None:
            self._load_model()
        if self._owns_detector:
            boxes,classes,scores = self.detector.predict(frame,conf=self.config.get('conf_threshold',.6),iou=self.config.get('iou_threshold',.45))
        else:
            boxes,classes,scores = self.detector.predict(frame)
        objs, _ = self.process(frame, boxes, classes, scores, team, ready, camera_deg)
        return objs

    def process(self, frame, boxes, classes, scores, team=None, ready=True, camera_deg=None):
        """复用感知处理，也可接收独立推理节点的检测结果。"""
        # ROS 版从 PerceptionNode 直接进 process()。boxes、classes、scores 已经由 YOLO 节点算好。
        if not ready:
            self.reset()
        self.detections = self.background_filter.filter(boxes,classes,scores,frame.shape,camera_deg if ready else None)
        # Objs 在帧之间保留物块 ID。ROS 感知节点另用同帧的 OpenCV 轮廓标出区内物块。
        targets = to_targets(*self.detections,self.labels)
        return self.objects.update([obj for obj in targets if obj['name'] in ('orange','blue','green','black')]), []

    def _load_model(self):
        """只在实机第一次推理时加载已有RKNN封装。"""
        model_path = self.config.get('model_path','')
        if not str(model_path).lower().endswith('.rknn'):
            raise ValueError('[GetObjs] 请在project_config.yaml中填写RKNN模型路径')
        from ..inference.yolov8 import YOLOv8
        self.detector = YOLOv8(model_path,input_size=tuple(self.config.get('input_size',[640,640])))

    def reset(self):
        """换相机位置或重新执行任务时清空物体记忆。"""
        self.objects.reset()

    def close(self):
        """释放本模块加载的实机模型；外部传入的模型由调用者管理。"""
        if self._owns_detector and self.detector is not None:
            self.detector.close()
