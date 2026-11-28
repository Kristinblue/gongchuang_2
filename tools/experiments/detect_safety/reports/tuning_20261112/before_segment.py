"""安全区颜色分割与边框线合并，只接收图像和预选ROI。"""

import math

import cv2
import numpy as np


DEFAULTS = {
    'h_min':125, 'h_max':155,
    's_min':100, 's_max':255,
    'v_min':25, 'v_max':255,
    'second_enabled':False,
    'close_size':3,
    'min_length_ratio':.08,
    'merge_angle_deg':10.,
    'merge_distance_ratio':.035,
}


def purple_mask(image, settings):
    """提取紫色；H使用OpenCV的0～179范围，允许跨越色相零点。"""
    hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    ranges = [settings]
    if settings.get('second_enabled',False):
        ranges.append(settings['second_range'])
    mask = np.zeros(image.shape[:2],np.uint8)
    for limits in ranges:
        low = (limits['h_min'],limits['s_min'],limits['v_min'])
        high = (limits['h_max'],limits['s_max'],limits['v_max'])
        if low[0]<=high[0]:
            selected = cv2.inRange(hsv,low,high)
        else:
            first = cv2.inRange(hsv,(0,low[1],low[2]),high)
            second = cv2.inRange(hsv,low,(179,high[1],high[2]))
            selected = first | second
        mask |= selected
    size = int(settings['close_size'])
    if size>1:
        mask = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((size,size),np.uint8))
    return mask


def representative(lines):
    """同一栏杆的线段按长度合方向，横向取中线，纵向保留观测范围。"""
    points = np.asarray(lines,dtype=np.float32).reshape(-1,2,2)
    vectors = points[:,1]-points[:,0]
    lengths = np.linalg.norm(vectors,axis=1)
    directions = vectors/lengths[:,None]
    reference = directions[np.argmax(lengths)]
    directions[directions@reference<0] *= -1
    direction = np.sum(directions*lengths[:,None],axis=0)
    direction /= np.linalg.norm(direction)
    normal = np.array([-direction[1],direction[0]])
    flat = points.reshape(-1,2)
    along, across = flat@direction, flat@normal
    offset = (across.min()+across.max())/2
    start = direction*along.min()+normal*offset
    end = direction*along.max()+normal*offset
    return np.concatenate((start,end))


def same_rail(line, group, mask, distance, angle_limit):
    """按方向、相对间距和紫色带支持合并，不跨越安全区内部空白。"""
    current = representative(group).reshape(2,2)
    candidate = np.asarray(line,dtype=np.float32).reshape(2,2)
    direction = current[1]-current[0]
    direction /= np.linalg.norm(direction)
    other = candidate[1]-candidate[0]
    other /= np.linalg.norm(other)
    if abs(float(direction@other))<math.cos(math.radians(angle_limit)):
        return False
    normal = np.array([-direction[1],direction[0]])
    first, second = np.sort(current@direction), np.sort(candidate@direction)
    overlap_start = max(first[0],second[0])
    overlap_end = min(first[1],second[1])
    gap = max(0.,overlap_start-overlap_end)
    if gap>distance*3:
        return False
    old_offset = float(current.mean(axis=0)@normal)
    new_offset = float(candidate.mean(axis=0)@normal)
    separation = abs(old_offset-new_offset)
    # 同一条边被遮挡后留下的共线碎段，不要求中间仍有紫色像素。
    if separation<=max(2.,distance*.2):
        return True
    if separation>distance*3 or overlap_end<=overlap_start:
        return False
    # 栏杆上下沿之间应当主要是紫色；前后栏杆之间则是红蓝底垫。
    along = (overlap_start+overlap_end)/2
    offsets = np.linspace(old_offset,new_offset,20)
    samples = along*direction+offsets[:,None]*normal
    xs = np.clip(np.rint(samples[:,0]).astype(int),0,mask.shape[1]-1)
    ys = np.clip(np.rint(samples[:,1]).astype(int),0,mask.shape[0]-1)
    return np.count_nonzero(mask[ys,xs])/len(xs)>=.6


def merge_lines(lines, mask, settings):
    """从最长线开始分组，每组输出一条代表线，不补造缺失边。"""
    distance = max(mask.shape)*settings['merge_distance_ratio']
    ordered = sorted(lines,key=lambda line:math.hypot(line[2]-line[0],line[3]-line[1]),reverse=True)
    groups = []
    for line in ordered:
        for group in groups:
            if same_rail(line,group,mask,distance,settings['merge_angle_deg']):
                group.append(line)
                break
        else:
            groups.append([line])
    return [representative(group).tolist() for group in groups]


def segment_safety(image, rois, settings=None):
    """分割YOLO预选安全区，在各个ROI内部独立拟合与合并线段。

    Args:
        image: 原始BGR图像，不修改输入。
        rois: 全图坐标的[x1,y1,x2,y2]列表；空列表不检测。
        settings: 覆盖DEFAULTS的阈值和比例参数。

    Returns:
        字典包含二值图mask、候选线图lines_image、黑底合并图merged_image，
        以及全图坐标的segments和merged_segments。候选数不固定为四条。
    """
    config = dict(DEFAULTS)
    if settings is not None:
        config.update(settings)
    height,width = image.shape[:2]
    mask = np.zeros((height,width),np.uint8)
    lines_image = image.copy()
    merged_image = np.zeros_like(image)
    segments, merged_segments = [], []
    colors = [(0,220,255),(255,180,70),(100,255,120),(220,120,255)]
    for index, roi in enumerate(rois):
        x1,y1,x2,y2 = np.rint(roi).astype(int)
        x1,y1,x2,y2 = max(0,x1),max(0,y1),min(width,x2),min(height,y2)
        if x2-x1<5 or y2-y1<5:
            continue
        local = purple_mask(image[y1:y2,x1:x2],config)
        mask[y1:y2,x1:x2] |= local
        edges = cv2.Canny(local,50,150)
        # ROI裁切处的边缘不能当成栏杆边界。
        edges[:2] = edges[-2:] = 0
        edges[:,:2] = edges[:,-2:] = 0
        scale = max(local.shape)
        length = max(8,round(scale*config['min_length_ratio']))
        found = cv2.HoughLinesP(edges,1,np.pi/180,max(8,length//2),minLineLength=length,maxLineGap=max(3,round(scale*.05)))
        if found is None:
            continue
        lines = found.reshape(-1,4).tolist()
        merged = merge_lines(lines,local,config)
        offset = np.array([x1,y1,x1,y1])
        color = colors[index%len(colors)]
        for line in lines:
            full = (np.asarray(line)+offset).astype(int).tolist()
            segments.append(full)
            cv2.line(lines_image,tuple(full[:2]),tuple(full[2:]),color,1,cv2.LINE_AA)
        for line in merged:
            full = np.rint(np.asarray(line)+offset).astype(int).tolist()
            merged_segments.append(full)
            cv2.line(merged_image,tuple(full[:2]),tuple(full[2:]),color,2,cv2.LINE_AA)
    return {'mask':mask,'lines_image':lines_image,'merged_image':merged_image,'segments':segments,'merged_segments':merged_segments}
