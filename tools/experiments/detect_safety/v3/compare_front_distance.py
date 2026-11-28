"""用已有 Godot 批次评估前长边到车轴的距离，不生成控制指令。"""

import argparse
from pathlib import Path

import cv2
import numpy as np

from .approach_trial import axes, camera_mount, read_json, static_check
from .compare_heading import DP_FRACTION, DetectorWithHull, fit_quad, solve_pose


def front_edge_gap(quad, intrinsics, basis, camera, ground_z, edge_height):
    """把四角投回平面，取朝向小车的长边，计算车轴到边的垂距。"""
    rays = np.column_stack((quad, np.ones(4))) @ np.linalg.inv(intrinsics).T @ basis.T
    if np.any(rays[:, 2] >= -1e-6) or camera[2] <= ground_z + edge_height:
        return None
    scale = (ground_z + edge_height - camera[2]) / rays[:, 2]
    corners = (camera + rays * scale[:, None])[:, :2]
    edges = np.roll(corners, -1, axis=0) - corners
    lengths = np.linalg.norm(edges, axis=1)
    pairs = ((0, 2), (1, 3))
    long_edges = max(pairs, key=lambda pair: sum(lengths[i] for i in pair))
    center = corners.mean(axis=0)
    for i in long_edges:
        if lengths[i] < 1e-6:
            continue
        normal = np.array([-edges[i, 1], edges[i, 0]]) / lengths[i]
        if (center - corners[i]) @ normal < 0:
            normal = -normal
        gap = float(corners[i] @ normal)
        if gap > 0:
            return gap
    return None


def compare(sim, batch, min_area, edge_height):
    detector = DetectorWithHull()
    params = read_json(sim / 'reports/robot_parameters.json')
    correction = read_json(sim / 'config/camera_reference.json')
    ground_z = -params['spawn_height_m'] - params['pivots_m']['left_wheel'][2]
    errors = []
    for record in read_json(batch / 'render_manifest.json')['records']:
        path = batch / 'images' / record['image']
        image = cv2.imdecode(np.fromfile(path, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            continue
        safety = detector.detect(image)
        if not safety.status['full']:
            continue
        quad, _ = fit_quad(detector.hull, DP_FRACTION)
        if static_check(image, safety, quad, min_area)[0] != 'ready':
            continue
        height, width = image.shape[:2]
        focal = width / (2 * np.tan(np.deg2rad(35)))
        intrinsics = np.array([[focal, 0, width / 2], [0, focal, height / 2], [0, 0, 1]])
        pitch = record['pitch_deg']
        camera, basis = camera_mount(params, correction, pitch)
        gap = front_edge_gap(quad, intrinsics, basis, camera, ground_z, edge_height)
        pnp = solve_pose(quad, intrinsics, pitch)
        if gap is None or pnp is None:
            continue
        rotation, translation = pnp
        inward = (basis @ rotation[:, 1])[:2]
        pnp_gap = (camera + basis @ translation)[:2] @ (inward / np.linalg.norm(inward))
        sign = 1 if int(path.stem.rsplit('_', 1)[1]) % 2 else -1
        truth_in = axes(sign * record['side_deg'])[1]
        truth_gap = (camera[:2] + [0, record['distance_m']]) @ truth_in - .18
        errors.append((abs(gap - truth_gap), abs(pnp_gap - truth_gap)))
    return np.asarray(errors)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('sim', type=Path)
    parser.add_argument('batch', type=Path)
    parser.add_argument('--min-area', type=float, default=.05)
    parser.add_argument('--edge-height', type=float, required=True, help='识别出的长边离地高度，米')
    args = parser.parse_args()
    errors = compare(args.sim, args.batch, args.min_area, args.edge_height)
    if not len(errors):
        raise SystemExit('没有通过四角和边支撑检查的可比样本')
    print(f'n={len(errors)} edge_height={args.edge_height:.3f} m')
    for label, column in zip(('长边投影', 'PnP'), errors.T):
        mean, median, p90 = 100 * np.array([column.mean(), np.median(column), np.quantile(column, .9)])
        print(f'{label}绝对误差 cm: mean={mean:.2f}, median={median:.2f}, p90={p90:.2f}')


if __name__ == '__main__':
    main()
