"""在已有实拍图上画对准半区后的直行箭头。"""

import argparse
import json
from os import environ
from pathlib import Path

import cv2
import numpy as np
from PIL import ImageFont

from .compare_heading import DP_FRACTION, RECTANGLE, DetectorWithHull, fit_quad
from .compare_quad import caption
from .fit_quad import OUTPUT


EXTRA = ('frame_0693', 'frame_0705', 'frame_0901', 'frame_0912', 'frame_1056', 'frame_1193')
WIDTH, HEIGHT = 640, 360


def order_front(quad):
    """对面向场地正面的实拍视角，取画面较下的一条长边作为入口。"""
    lengths = np.linalg.norm(np.roll(quad, -1, axis=0) - quad, axis=1)
    pair = int(lengths[1] + lengths[3] > lengths[0] + lengths[2])
    edge = max((pair, pair + 2), key=lambda i: quad[i, 1] + quad[(i + 1) % 4, 1])
    ordered = np.roll(quad, -edge, axis=0)
    return ordered if ordered[0, 0] < ordered[1, 0] else ordered[[1, 0, 3, 2]]


def draw_heading(image, quad, target):
    """把地面上垂直前长边的半区中心线投影回图像。"""
    scale = np.array([WIDTH, HEIGHT]) / image.shape[1::-1]
    view = cv2.resize(image, (WIDTH, HEIGHT), interpolation=cv2.INTER_AREA)
    ordered = order_front(quad).astype(np.float32)
    transform = cv2.getPerspectiveTransform(RECTANGLE[:, :2].copy(), ordered)
    guides = np.float32([[[0, .03], [0, .33], [target, 0], [target, .19]]])
    guide_pixels = cv2.perspectiveTransform(guides, transform)[0]
    divider_start, divider_end, entry, tip = np.rint(guide_pixels * scale).astype(np.int32)
    cv2.polylines(view, [np.rint(ordered * scale).astype(np.int32)], True, (50, 220, 245), 2, cv2.LINE_AA)
    cv2.line(view, tuple(divider_start), tuple(divider_end), (245, 245, 245), 2, cv2.LINE_AA)
    direction = tip - entry
    length = np.linalg.norm(direction)
    if length > 0:
        # 按投影出的方向向场地侧延长，避免把画面外的地面点投到相机身后。
        extension = np.clip(length, 40, 100)
        origin = np.rint(entry - direction * extension / length).astype(int)
        tip_length = 20 / (length + extension)
        cv2.arrowedLine(view, tuple(origin), tuple(tip), (26, 45, 25), 9, cv2.LINE_AA, tipLength=tip_length)
        cv2.arrowedLine(view, tuple(origin), tuple(tip), (85, 245, 85), 5, cv2.LINE_AA, tipLength=tip_length)
    return view


def label_touches_edge(path, image):
    """仅在离线预览中用已有标注提示安全区可能被裁，拟合仍用紫色轮廓。"""
    label = path.with_suffix('.json')
    if not label.exists():
        return False
    for shape in json.loads(label.read_text(encoding='utf-8-sig'))['shapes']:
        if shape['label'] in ('bsafe', 'rsafe'):
            points = np.asarray(shape['points'])
            if np.any((points <= 1) | (points >= np.array(image.shape[1::-1]) - 1)):
                return True
    return False


def save_sheet(paths, name, target, detector, font, small):
    tiles = []
    for path in paths:
        image = cv2.imdecode(np.fromfile(path, np.uint8), cv2.IMREAD_COLOR)
        safety = detector.detect(image)
        if not safety.status['full']:
            continue
        quad, _ = fit_quad(detector.hull, DP_FRACTION)
        if quad is None:
            continue
        outside = np.any((quad < 0) | (quad >= image.shape[1::-1]))
        detail = '四角在画内，颜色已识别'
        if outside:
            detail = '角点出画，仅预览'
        elif label_touches_edge(path, image):
            detail = '已有标注贴画边，仅预览'
        elif not safety.status['ok']:
            detail = '颜色未定，仅预览'
        title = f'{path.stem}  ·  {"伤员区" if target > 0 else "物资区"}'
        view = draw_heading(image, quad, target)
        tiles.append(np.vstack([caption(title, font, detail, small, width=WIDTH), view]))

    if not tiles:
        return
    count = len(tiles)
    if len(tiles) % 2:
        tiles.append(np.zeros_like(tiles[0]))
    rows = [np.hstack(tiles[index:index + 2]) for index in range(0, len(tiles), 2)]
    cv2.imencode('.jpg', np.vstack(rows), [cv2.IMWRITE_JPEG_QUALITY, 95])[1].tofile(OUTPUT / name)
    print(f'{name}: {count}/{len(paths)} real photos with an approach preview')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--images', type=Path, help='现有实拍图目录，读取其中 train 和 val 的部分样图')
    parser.add_argument('--target', choices=('casualty', 'supply'), default='casualty')
    args = parser.parse_args()
    target = .155 if args.target == 'casualty' else -.155
    fonts = Path(environ['WINDIR']) / 'Fonts'
    font = ImageFont.truetype(str(fonts / 'msyhbd.ttc'), 24)
    small = ImageFont.truetype(str(fonts / 'msyh.ttc'), 18)
    detector = DetectorWithHull()
    assets = OUTPUT.parent / 'assets'
    suffix = '' if args.target == 'casualty' else '_supply'
    local = sorted(path for folder in ('first', 'extra') for path in (assets / folder).glob('frame_*.jpg'))
    save_sheet(local, f'heading_real{suffix}.jpg', target, detector, font, small)
    if args.images:
        extra = [path for stem in EXTRA for path in args.images.glob(f'*/{stem}.jpg')]
        save_sheet(extra, f'heading_real_extra{suffix}.jpg', target, detector, font, small)


if __name__ == '__main__':
    main()
