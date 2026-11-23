"""对已有样图中 full=True 的安全区做四边形粗拟合。"""

from pathlib import Path

import cv2
import numpy as np

from ..v2.segment import SafetyDetector


OUTPUT = Path(__file__).resolve().parent
WIDTH, HEIGHT = 400, 225


def draw(image, points, color, corners=False):
    """缩小原图后画轮廓，出画的角用橙点标在图像边缘。"""
    view = cv2.resize(image, (WIDTH, HEIGHT), interpolation=cv2.INTER_AREA)
    limits = np.array([WIDTH, HEIGHT])
    scaled = np.rint(points * limits / image.shape[1::-1]).astype(np.int32)
    cv2.polylines(view, [scaled], True, color, 2, cv2.LINE_AA)
    if corners:
        inside = np.all((points >= 0) & (points < image.shape[1::-1]), axis=1)
        for point, visible in zip(scaled, inside):
            if not visible:
                point = np.clip(point, 4, limits - 5)
            cv2.circle(view, tuple(point), 4, color if visible else (0, 130, 255), -1, cv2.LINE_AA)
    return view


def samples():
    """读取此前收集的实拍和仿真样图。"""
    detector = SafetyDetector()
    for folder in ('first', 'extra'):
        for path in sorted((OUTPUT.parent / 'assets' / folder).glob('*.jpg')):
            if path.name.startswith('_'):
                continue
            image = cv2.imdecode(np.fromfile(path, np.uint8), cv2.IMREAD_COLOR)
            yield path, image, detector.detect(image)


def main():
    tiles = []
    total, outside_count = 0, 0
    for path, image, safety in samples():
        total += 1
        if not safety.status['full']:
            continue

        quad = cv2.approxPolyN(safety.points, 4).reshape(4, 2)
        outside = int(np.count_nonzero(np.any((quad < 0) | (quad >= image.shape[1::-1]), axis=1)))
        outside_count += outside > 0
        original = draw(image, safety.points, (255, 220, 65))
        fitted = draw(image, quad, (50, 240, 255), corners=True)
        header = np.full((30, WIDTH * 2, 3), 35, np.uint8)
        name = path.stem.replace('sim_safe5fix_20261023_', 'sim_')
        title = f'{name}   ok={safety.status["ok"]}'
        if outside:
            title += f'   |   outside corners = {outside}'
        cv2.putText(header, title, (10, 21), cv2.FONT_HERSHEY_SIMPLEX, .56, (245, 245, 245), 1, cv2.LINE_AA)
        tiles.append(np.vstack([header, np.hstack([original, fitted])]))
        print(f'{path.name}  ok={safety.status["ok"]}  outside={outside}')

    rows = []
    for index in range(0, len(tiles), 2):
        left, right = tiles[index], tiles[index + 1] if index + 1 < len(tiles) else np.zeros_like(tiles[index])
        divider = np.full((left.shape[0], 5, 3), 95, np.uint8)
        rows.append(np.hstack([left, divider, right]))
    cv2.imencode('.jpg', np.vstack(rows), [cv2.IMWRITE_JPEG_QUALITY, 93])[1].tofile(OUTPUT / 'full_quads.jpg')
    print(f'{total} images, {len(tiles)} full and fitted, {outside_count} with corners outside')


if __name__ == '__main__':
    main()
