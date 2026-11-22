"""在相同的 full 样图上比较安全区四边形拟合。"""

from itertools import combinations
from os import environ
from pathlib import Path
from statistics import median

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from .fit_quad import HEIGHT, OUTPUT, WIDTH, draw, samples


def caption(title, title_font, detail='', detail_font=None, width=WIDTH):
    """在每个小图上方直接写方法名和本张结果。"""
    band = Image.new('RGB', (width, 64 if detail_font else 40), '#202837')
    painter = ImageDraw.Draw(band)
    painter.text((12, 5), title, font=title_font, fill='#ffffff')
    if detail_font:
        painter.text((12, 36), detail, font=detail_font, fill='#d7e1ef')
    return cv2.cvtColor(np.asarray(band), cv2.COLOR_RGB2BGR)


def longest_edges(points, separate):
    """取两条边，按原轮廓顺序连接四个端点。"""
    count = len(points)
    lengths = np.linalg.norm(points - np.roll(points, -1, axis=0), axis=1)
    if separate:
        first, second = max(
            ((i, j) for i, j in combinations(range(count), 2) if 1 < j - i < count - 1),
            key=lambda pair: lengths[pair[0]] + lengths[pair[1]],
        )
    else:
        first, second = np.argsort(-lengths, kind='stable')[:2]
    indices = sorted({first, (first + 1) % count, second, (second + 1) % count})
    return points[indices], (first, second)


def approx_dp4(points):
    """逐步加大容差，最多试到周长的 5%。"""
    perimeter = cv2.arcLength(points, True)
    for fraction in np.geomspace(.004, .05, 24):
        polygon = cv2.approxPolyDP(points, float(fraction * perimeter), True).reshape(-1, 2)
        if len(polygon) <= 4:
            return polygon if len(polygon) == 4 else None
    return None


def overlap(points, quad):
    """计算拟合框和检测多边形的面积交并比。"""
    source = points.astype(np.float32)
    target = quad.astype(np.float32)
    intersect, _ = cv2.intersectConvexConvex(source, target)
    return intersect / (cv2.contourArea(source) + cv2.contourArea(target) - intersect)


def main():
    fonts = Path(environ['WINDIR']) / 'Fonts'
    title_font = ImageFont.truetype(str(fonts / 'msyhbd.ttc'), 25)
    detail_font = ImageFont.truetype(str(fonts / 'msyh.ttc'), 19)
    methods = ('approxPolyN 外接拟合', '两条最长边直接连', '最长不相邻边直接连', 'approxPolyDP 取四点', '最小外接矩形')
    rows = {'real': [], 'sim': []}
    measurements = {name: [] for name in methods}
    yellow, cyan, pink = (50, 240, 255), (255, 220, 65), (235, 90, 245)
    for path, image, safety in samples():
        if not safety.status['full']:
            continue
        points = safety.points
        original = draw(image, points, cyan)
        name = path.stem.replace('sim_safe5fix_20261023_', 'sim_')
        tiles = [np.vstack([caption('原检测轮廓', title_font, '蓝线是检测多边形', detail_font), original])]
        candidates = [(None, ()) for _ in methods]
        if len(points) >= 4:
            candidates = [
                (cv2.approxPolyN(points, 4).reshape(4, 2), ()),
                longest_edges(points, separate=False),
                longest_edges(points, separate=True),
                (approx_dp4(points), ()),
                (cv2.boxPoints(cv2.minAreaRect(points)), ()),
            ]
        for method, (quad, edge_indices) in zip(methods, candidates):
            if quad is None or len(quad) != 4:
                result = original.copy()
                info = '两边共端点' if edge_indices else '本张未得到四个角'
            else:
                result = draw(image, quad, yellow, corners=True)
                outside = bool(np.any((quad < 0) | (quad >= image.shape[1::-1])))
                iou = overlap(points, quad)
                measurements[method].append((outside, iou))
                info = f'IoU {iou:.2f}' + ('  ·  有角出画' if outside else '')
            for index in edge_indices:
                edge = points[[index, (index + 1) % len(points)]]
                edge = np.rint(edge * [WIDTH, HEIGHT] / image.shape[1::-1]).astype(np.int32)
                cv2.line(result, tuple(edge[0]), tuple(edge[1]), pink, 2, cv2.LINE_AA)
            tiles.append(np.vstack([caption(method, title_font, info, detail_font), result]))
        group = 'sim' if path.name.startswith('sim_') else 'real'
        head = caption(name, title_font, width=3 * WIDTH)
        rows[group].append(np.vstack([head, np.hstack(tiles[:3]), np.hstack(tiles[3:])]))

    for group, images in rows.items():
        divider = np.full((8, 3 * WIDTH, 3), 235, np.uint8)
        sheet = np.vstack([np.vstack([panel, divider]) for panel in images])
        cv2.imencode('.jpg', sheet, [cv2.IMWRITE_JPEG_QUALITY, 94])[1].tofile(OUTPUT / f'compare_{group}.jpg')
    for method, values in measurements.items():
        if not values:
            print(f'{method}: no fits')
            continue
        outside = sum(is_outside for is_outside, _ in values)
        scores = [iou for _, iou in values]
        print(f'{method:<26} fits={len(values):2d}  outside={outside}  '
              f'median IoU={median(scores):.3f}  min IoU={min(scores):.3f}')


if __name__ == '__main__':
    main()
