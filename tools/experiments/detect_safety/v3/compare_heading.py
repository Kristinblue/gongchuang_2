"""用 Godot 渲染批次的朝向真值比较两种四边形拟合。"""

import argparse
import json
from os import environ
from pathlib import Path
from statistics import median

import cv2
import numpy as np
from PIL import ImageFont

from ..v2.segment import SafetyDetector
from .compare_quad import caption
from .fit_quad import HEIGHT, OUTPUT, WIDTH, draw


RECTANGLE = np.float32([[-.33, 0, 0], [.33, 0, 0], [.33, .36, 0], [-.33, .36, 0]])
DP_FRACTION = .002


class DetectorWithHull(SafetyDetector):
    def _find_roi(self, mask):
        roi, self.hull = super()._find_roi(mask)
        return roi, self.hull


def fit_quad(hull, fraction=0.):
    """可按周长的给定比例先简化，已有四角时直接返回。"""
    points = hull
    if fraction:
        points = cv2.approxPolyDP(hull, fraction * cv2.arcLength(hull, True), True)
        if len(points) == 4:
            return points.reshape(4, 2), False
        if len(points) < 4:
            points = hull
    if len(points) < 4:
        return None, False
    return cv2.approxPolyN(points, 4).reshape(4, 2), True


def solve_pose(quad, intrinsics, pitch_deg):
    """用已知俯角排序四角，再用平面 PnP 求安全区位姿。"""
    pitch = np.deg2rad(pitch_deg)
    right = np.array([1., 0., 0.])
    forward = np.array([0., -np.sin(pitch), np.cos(pitch)])
    down = np.array([0., np.cos(pitch), np.sin(pitch)])
    rays = np.column_stack((quad, np.ones(4))) @ np.linalg.inv(intrinsics).T
    depths = rays @ down
    if np.any(depths <= 0):
        return None
    ground = (rays / depths[:, None]) @ np.column_stack((right, forward))
    lengths = np.linalg.norm(ground - np.roll(ground, -1, axis=0), axis=1)
    pair = int(lengths[1] + lengths[3] > lengths[0] + lengths[2])
    center = np.mean(ground, axis=0)
    # 只选面向车辆的长边。车辆原点和安全区中心应在该边两侧。
    for index in (pair, pair + 2):
        start, end = ground[[index, (index + 1) % 4]]
        edge = end - start
        normal = np.array([-edge[1], edge[0]])
        center_side = (center - start) @ normal
        if center_side * (-start @ normal) < 0:
            ordered = np.roll(quad, -index, axis=0)
            if center_side < 0:
                ordered = ordered[[1, 0, 3, 2]]
            break
    else:
        return None

    found, rotations, translations, _ = cv2.solvePnPGeneric(
        RECTANGLE, ordered.astype(np.float32), intrinsics, None, flags=cv2.SOLVEPNP_IPPE)
    candidates = []
    for rvec, tvec in zip(rotations, translations):
        rotation, _ = cv2.Rodrigues(rvec)
        if np.all((RECTANGLE @ rotation[2] + tvec[2, 0]) > 0):
            candidates.append((rotation, tvec[:, 0]))
    if not found or not candidates:
        return None
    # 平面 PnP 的两个候选里，选围栏法向最接近地面法向的一个。
    return max(candidates, key=lambda value: value[0][:, 2] @ -down)


def heading(quad, intrinsics, pitch_deg):
    """从安全区位姿取正对前长边所需的转角。"""
    pose = solve_pose(quad, intrinsics, pitch_deg)
    if pose is None:
        return None
    rotation, _ = pose
    pitch = np.deg2rad(pitch_deg)
    forward = np.array([0., -np.sin(pitch), np.cos(pitch)])
    inward = rotation[:, 1]
    return float(np.rad2deg(np.arctan2(inward[0], inward @ forward)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('batch', type=Path, help='含 render_manifest.json 和 images 的 Godot 渲染目录')
    parser.add_argument('--fov', type=float, default=70., help='渲染批次 Camera3D 的水平视场角，单位度')
    args = parser.parse_args()
    records = json.loads((args.batch / 'render_manifest.json').read_text(encoding='utf-8'))['records']
    detector = DetectorWithHull()
    results = []
    for record in records:
        path = args.batch / 'images' / record['image']
        image = cv2.imdecode(np.fromfile(path, np.uint8), cv2.IMREAD_COLOR)
        safety = detector.detect(image)
        if not safety.status['full']:
            continue
        height, width = image.shape[:2]
        focal = width / (2 * np.tan(np.deg2rad(args.fov / 2)))
        intrinsics = np.array([[focal, 0., width / 2], [0., focal, height / 2], [0., 0., 1.]])
        # 渲染脚本交替放红、蓝安全区。两区相对场地中心镜像。
        truth = record['side_deg'] * (1 if int(path.stem.rsplit('_', 1)[1]) % 2 else -1)
        direct, _ = fit_quad(detector.hull)
        staged, used_n = fit_quad(detector.hull, DP_FRACTION)
        quads = [direct, staged]
        estimates = [heading(q, intrinsics, record['pitch_deg']) if q is not None else None for q in quads]
        errors = [abs((value - truth + 180) % 360 - 180) if value is not None else None for value in estimates]
        results.append((path, safety.status['ok'], truth, quads, estimates, errors, used_n))

    print(f'Manifest frames: {len(records)}, full: {len(results)}, ok: {sum(row[1] for row in results)}')
    print(f'Small DP used N for {sum(row[6] for row in results)}/{len(results)} frames')
    for column, label in enumerate(('N direct', f'DP {DP_FRACTION:.1%} then N as needed')):
        errors = [row[5][column] for row in results if row[5][column] is not None]
        if errors:
            print(f'{label}: {len(errors)}/{len(results)} poses, median {median(errors):.2f} deg, '
                  f'mean {np.mean(errors):.2f} deg, max {max(errors):.2f} deg, over 10 deg {sum(e > 10 for e in errors)}')
    paired = [row for row in results if all(value is not None for value in row[5])]
    print(f'DP + N lower error: {sum(row[5][1] < row[5][0] for row in paired)}/{len(paired)} paired frames')

    if paired:
        show_comparison(paired)


def show_comparison(paired):
    """显示左右斜视、近正视和两种方法误差最大的样图。"""
    fonts = Path(environ['WINDIR']) / 'Fonts'
    font = ImageFont.truetype(str(fonts / 'msyhbd.ttc'), 23)
    small = ImageFont.truetype(str(fonts / 'msyh.ttc'), 18)
    examples = [
        min(paired, key=lambda row: abs(row[2])),
        min(paired, key=lambda row: row[2]),
        max(paired, key=lambda row: row[2]),
        max(paired, key=lambda row: row[5][0]),
        max(paired, key=lambda row: row[5][1]),
    ]
    tiles = []
    shown = set()
    for path, ok, truth, quads, estimates, errors, _ in examples:
        if path in shown:
            continue
        shown.add(path)
        image = cv2.imdecode(np.fromfile(path, np.uint8), cv2.IMREAD_COLOR)
        views = [cv2.resize(image, (WIDTH, HEIGHT))]
        views.extend(draw(image, quad, (50, 235, 250), corners=True) for quad in quads)
        detail = [f'真实转角 {truth:+.1f}°  ·  ok={ok}',
                  f'估计 {estimates[0]:+.1f}°  ·  偏差 {errors[0]:.1f}°',
                  f'估计 {estimates[1]:+.1f}°  ·  偏差 {errors[1]:.1f}°']
        titles = (path.stem.split('20261023_')[-1], '原凸包直接 N', f'DP {DP_FRACTION:.1%} 后按需用 N')
        labels = [caption(title, font, text, small) for title, text in zip(titles, detail)]
        tiles.extend([np.hstack(labels), np.hstack(views)])
    cv2.imencode('.jpg', np.vstack(tiles))[1].tofile(OUTPUT / 'heading_comparison.jpg')


if __name__ == '__main__':
    main()
