"""离线检查实拍四边形，并用 Godot 的位姿真值验证差速车对准动作。"""

import argparse
import json
from os import environ
from pathlib import Path

import cv2
import numpy as np
from PIL import ImageFont

from .compare_heading import DP_FRACTION, RECTANGLE, DetectorWithHull, fit_quad, solve_pose
from .compare_quad import caption
from .fit_quad import OUTPUT, draw


MIN_AREA = .08
MIN_MARGIN = .015
MIN_EDGE_COVERAGE = .60
TARGET_X = .155


def static_check(image, safety, quad, min_area=MIN_AREA):
    """只用当前一帧的图像检查四边形能否拿来解算。"""
    if quad is None or not cv2.isContourConvex(quad):
        return 'no convex quad', 0., 0., 0.
    height, width = image.shape[:2]
    area = cv2.contourArea(quad) / (width * height)
    margin = float(np.minimum(quad, [width - 1, height - 1] - quad).min())
    y, x = np.nonzero(safety.mask)
    pixels = np.column_stack((x, y)) + safety.roi[:2]
    coverage = []
    for start, end in zip(quad, np.roll(quad, -1, axis=0)):
        vector = end - start
        length = np.linalg.norm(vector)
        unit = vector / length
        along = (pixels - start) @ unit / length
        distance = (pixels - start) @ [-unit[1], unit[0]]
        near = (along >= .06) & (along < .94) & (np.abs(distance) <= .011 * height)
        bins = np.floor((along[near] - .06) / .088).astype(int)
        coverage.append(len(np.unique(bins)) / 10)
    weakest = min(coverage)
    if not safety.status['full']:
        reason = 'not full'
    elif margin < MIN_MARGIN * height:
        reason = 'near frame edge'
    elif area < min_area:
        reason = 'too small'
    elif weakest < MIN_EDGE_COVERAGE:
        reason = 'weak edge'
    elif not safety.status['ok']:
        reason = 'color unknown'
    else:
        reason = 'ready'
    return reason, area, margin, weakest


def read_json(path):
    return json.loads(path.read_text(encoding='utf-8'))


def camera_mount(params, correction, pitch_deg):
    """复用仿真报告中的相机安装几何，将 OpenCV 相机系变换到轴心车体系。"""
    pitch = np.deg2rad(pitch_deg)
    camera = params['camera_optical']
    pivots = params['pivots_m']
    spin, _ = cv2.Rodrigues(np.array([0., pitch - camera['cad_initial_pitch_rad'], 0.]))
    tilt, _ = cv2.Rodrigues(np.array([0., pitch, 0.]))
    axes = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])
    camera_basis = tilt @ axes
    camera_position = pivots['camera_tilt'] + spin @ camera['local_origin_in_tilt_link_m']
    camera_position += camera_basis @ np.asarray(correction['camera_local_translation_m'])
    camera_basis = camera_basis @ np.asarray(correction['camera_local_rotation_rows'])
    axle = (np.array(pivots['left_wheel']) + pivots['right_wheel']) / 2
    to_body = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    return to_body @ (camera_position - axle), to_body @ camera_basis @ np.diag([1., -1., -1.])


def axes(heading_deg):
    yaw = np.deg2rad(heading_deg)
    return np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])


def wrap(angle):
    return (angle + 180) % 360 - 180


def plan(stop, inward):
    """输出右转为正的两次转角和中间前进的厘米数。"""
    yaw = np.rad2deg(np.arctan2(inward[0], inward[1]))
    first = round(np.rad2deg(np.arctan2(stop[0], stop[1])))
    distance = round(100 * np.linalg.norm(stop))
    last = round(wrap(yaw - first))
    return (first, distance, last), axes(first)[1] * (distance / 100), wrap(first + last)


def visible_after(corners, position, yaw, camera_pos, camera_basis, intrinsics, width, height):
    """在终点投影真实地面角，验证是否具备重新看全的视野。"""
    local = corners.copy()
    local[:, :2] = (corners[:, :2] - position) @ axes(yaw).T
    rays = (local - camera_pos) @ camera_basis
    if np.any(rays[:, 2] <= 0):
        return False
    pixels = rays @ intrinsics.T / rays[:, 2:]
    return bool(np.all((pixels[:, :2] >= 3) & (pixels[:, :2] <= [width - 4, height - 4])))


def run(batch, sim, standoff, min_area):
    detector = DetectorWithHull()
    records = read_json(batch / 'render_manifest.json')['records']
    params = read_json(sim / 'reports' / 'robot_parameters.json')
    correction = read_json(sim / 'config' / 'camera_reference.json')
    stats = {}
    estimates = []
    for record in records:
        image = cv2.imdecode(np.fromfile(batch / 'images' / record['image'], np.uint8), cv2.IMREAD_COLOR)
        safety = detector.detect(image)
        if not safety.status['full']:
            continue
        quad, _ = fit_quad(detector.hull, DP_FRACTION)
        reason, _, _, _ = static_check(image, safety, quad, min_area)
        stats[reason] = stats.get(reason, 0) + 1
        if reason != 'ready':
            continue
        height, width = image.shape[:2]
        focal = width / (2 * np.tan(np.deg2rad(35)))
        intrinsics = np.array([[focal, 0., width / 2], [0., focal, height / 2], [0., 0., 1.]])
        pitch = record['pitch_deg']
        pose = solve_pose(quad, intrinsics, pitch)
        if pose is None:
            continue
        rotation, translation = pose
        camera_pos, camera_basis = camera_mount(params, correction, pitch)
        estimated_front = (camera_pos + camera_basis @ translation)[:2]
        inward = (camera_basis @ rotation[:, 1])[:2]
        inward /= np.linalg.norm(inward)
        side = np.array([inward[1], -inward[0]])
        aim = estimated_front + TARGET_X * side - standoff * inward
        commands, actual_stop, final_yaw = plan(aim, inward)
        sign = 1 if int(Path(record['image']).stem.rsplit('_', 1)[1]) % 2 else -1
        truth_deg = sign * record['side_deg']
        truth_side, truth_in = axes(truth_deg)
        truth_front = camera_pos[:2] + [0, record['distance_m']] - .18 * truth_in
        offset = actual_stop - truth_front
        lateral_error = 100 * (offset @ truth_side - TARGET_X)
        gap = -100 * (offset @ truth_in)
        truth_corners = np.column_stack(
            (truth_front + RECTANGLE[:, :2] @ axes(truth_deg),
             np.full(4, -params['spawn_height_m'] - params['pivots_m']['left_wheel'][2])))
        full_after = visible_after(truth_corners, actual_stop, final_yaw, camera_pos, camera_basis, intrinsics, width, height)
        check_mount = camera_mount(params, correction, 20.)
        full_at_20 = visible_after(truth_corners, actual_stop, final_yaw, *check_mount, intrinsics, width, height)
        estimates.append((Path(record['image']).stem, commands, lateral_error, gap,
                          wrap(final_yaw - truth_deg), full_after, full_at_20, actual_stop, final_yaw, truth_corners))
    return stats, estimates


def show_real(dataset):
    """显示可疑实拍和一张通过检查的实拍。"""
    detector = DetectorWithHull()
    fonts = Path(environ['WINDIR']) / 'Fonts'
    bold = ImageFont.truetype(str(fonts / 'msyhbd.ttc'), 23)
    small = ImageFont.truetype(str(fonts / 'msyh.ttc'), 18)
    labels = {'ready': '允许解算', 'not full': '未看全', 'near frame edge': '角点离画边太近',
              'too small': '面积太小', 'weak edge': '边上缺少紫色像素', 'color unknown': '颜色未定'}
    assets = OUTPUT.parent / 'assets'
    rows = []
    for stem in ('frame_0320', 'frame_0901', 'frame_0902', 'frame_0912', 'frame_1056', 'frame_1193'):
        matches = list(assets.glob(f'*/{stem}.jpg')) + list(dataset.glob(f'*/{stem}.jpg'))
        if not matches:
            continue
        image = cv2.imdecode(np.fromfile(matches[0], np.uint8), cv2.IMREAD_COLOR)
        safety = detector.detect(image)
        quad, _ = fit_quad(detector.hull, DP_FRACTION)
        reason, area, margin, coverage = static_check(image, safety, quad)
        detail = f'面积 {area:.1%} · 余量 {margin:.0f} px · 最弱边 {coverage:.0%}'
        _, _, original = safety.draw_result(image)
        mask = detector._purple_mask(image)[1]
        fitted = draw(image, quad, (50, 230, 250), corners=True)
        views = [cv2.resize(original, (400, 225)), fitted,
                 cv2.cvtColor(cv2.resize(mask, (400, 225), interpolation=cv2.INTER_NEAREST), cv2.COLOR_GRAY2BGR)]
        notes = [caption(stem, bold, 'v2 检测轮廓', small), caption(labels[reason], bold, detail, small),
                 caption('紫色掩膜', bold, '白色是阈值内的像素', small)]
        rows.extend([np.hstack(notes), np.hstack(views)])
        print(f'Real {stem}: {reason}, area={area:.1%}, margin={margin:.0f}px, weakest edge={coverage:.0%}')
    if rows:
        cv2.imencode('.jpg', np.vstack(rows))[1].tofile(OUTPUT / 'approach_static_real.jpg')


def show_paths(estimates, standoff):
    """俯视图标出理想执行后的停车点和最终车头方向。"""
    fonts = Path(environ['WINDIR']) / 'Fonts'
    bold = ImageFont.truetype(str(fonts / 'msyhbd.ttc'), 23)
    small = ImageFont.truetype(str(fonts / 'msyh.ttc'), 18)
    width, height = 560, 425
    panels = []
    for row in estimates:
        name, commands, offset, gap, error, visible, _, stop, yaw, corners = row
        if not any(name.endswith(sample) for sample in ('neutral_05', 'bright_side_11')):
            continue
        rectangle = corners[:, :2]
        inward = (rectangle[3] - rectangle[0]) / .36
        side = (rectangle[1] - rectangle[0]) / .66
        entry = (rectangle[0] + rectangle[1]) / 2 + TARGET_X * side
        lane = np.array([entry - standoff * inward, entry + .28 * inward])
        final = stop + .32 * axes(yaw)[1]
        points = np.vstack([rectangle, lane, stop, final, [0., 0.], [0., .14]])
        lo, hi = points.min(axis=0), points.max(axis=0)
        scale = min((width - 72) / (hi[0] - lo[0]), (height - 72) / (hi[1] - lo[1]))
        center = (lo + hi) / 2

        def pixel(point):
            x, y = (np.asarray(point) - center) * scale
            return round(width / 2 + x), round(height / 2 - y)

        view = np.full((height, width, 3), 244, np.uint8)
        poly = np.int32([pixel(point) for point in rectangle])
        cv2.fillConvexPoly(view, poly, (214, 200, 227), cv2.LINE_AA)
        cv2.polylines(view, [poly], True, (120, 70, 120), 2, cv2.LINE_AA)
        divider = (rectangle[0] + rectangle[1]) / 2
        cv2.line(view, pixel(divider), pixel(divider + .36 * inward), (140, 100, 140), 2, cv2.LINE_AA)
        cv2.line(view, pixel(lane[0]), pixel(lane[1]), (0, 166, 227), 3, cv2.LINE_AA)
        cv2.arrowedLine(view, pixel([0., 0.]), pixel(stop), (205, 118, 38), 3, cv2.LINE_AA, tipLength=.09)
        cv2.arrowedLine(view, pixel(stop), pixel(final), (45, 146, 40), 5, cv2.LINE_AA, tipLength=.18)
        cv2.arrowedLine(view, pixel([0., 0.]), pixel([0., .14]), (75, 75, 75), 2, cv2.LINE_AA, tipLength=.20)
        cv2.circle(view, pixel(stop), 5, (45, 146, 40), -1, cv2.LINE_AA)
        first, distance, last = commands
        detail = f'转 {first:+d}° → 走 {distance} cm → 转 {last:+d}°'
        summary = f'终点横偏 {offset:+.1f} cm · 朝向偏 {error:+.1f}° · 视野{"可看全" if visible else "有截断"}'
        panels.append(np.vstack([caption(name.split('20261023_')[-1], bold, detail, small, width=width),
                                 view, caption(summary, small, width=width)]))
    if panels:
        cv2.imencode('.jpg', np.hstack(panels))[1].tofile(OUTPUT / 'approach_paths.jpg')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('sim', type=Path, help='原 gongchuang_sim 目录')
    parser.add_argument('--standoff', type=float, default=.65, help='目标轴心到前围栏距离，单位米')
    parser.add_argument('--min-area', type=float, default=MIN_AREA, help='拟合四边形的最小图像面积占比')
    parser.add_argument('--real-images', type=Path, help='实拍图的数据目录，含 train 和 val')
    parser.add_argument('--path-image', action='store_true', help='输出典型动作序列的俯视图')
    args = parser.parse_args()
    if args.real_images:
        show_real(args.real_images)
    batch = args.sim / 'reports' / 'safe_light_20261023_fixed'
    stats, estimates = run(batch, args.sim, args.standoff, args.min_area)
    print('FULL image gate:', stats)
    if not estimates:
        return
    if args.path_image:
        show_paths(estimates, args.standoff)
    for name, commands, lateral, gap, yaw, visible, visible20, *_ in estimates:
        print(f'{name}: {commands}, final lateral {lateral:+.1f} cm, '
              f'gap {gap:.1f} cm, yaw {yaw:+.1f} deg, visible {visible}/{visible20}')
    errors = np.abs(np.array([[row[2], row[4]] for row in estimates]))
    print(f'Final merit, N={len(estimates)}: mean abs lateral {errors[:, 0].mean():.2f} cm, '
          f'yaw {errors[:, 1].mean():.2f} deg, max yaw {errors[:, 1].max():.2f} deg; '
          f'full rectangle still visible {sum(row[5] for row in estimates)}/{len(estimates)} at original pitch, '
          f'{sum(row[6] for row in estimates)}/{len(estimates)} at 20 deg')


if __name__ == '__main__':
    main()
