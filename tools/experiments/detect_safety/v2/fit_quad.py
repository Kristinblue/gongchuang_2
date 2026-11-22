"""离线预览安全区四边形和伤员区的对准线。"""

from pathlib import Path

import cv2
import numpy as np

from segment import SafetyDetector


ROOT = Path(__file__).resolve().parents[1]
FIELD_CORNERS_MM = np.float32([[-330, 0], [330, 0], [330, 360], [-330, 360]])
EXAMPLES = {'frame_0320', 'frame_0227', 'frame_1190',
            'sim_safe5fix_20261023_bright_side_05', 'sim_safe5fix_20261023_bright_side_11'}


def fit_quad(points, mask, roi):
    """用凸包粗找四边，再用实际可见的紫边像素修正交点。"""
    perimeter = cv2.arcLength(points, True)
    for fraction in np.geomspace(.004, .04, 18):
        coarse = cv2.approxPolyDP(points, float(fraction * perimeter), True)[:, 0]
        if len(coarse) <= 4:
            break
    if len(coarse) != 4:
        return None

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    pixels = np.concatenate(contours)[:, 0] + np.array(roi[:2])
    width, height = cv2.minAreaRect(points)[1]
    tolerance = np.clip(.04 * min(width, height), 2, 6)
    lines = []
    for start, end in zip(coarse, np.roll(coarse, -1, axis=0)):
        edge = end - start
        length = np.linalg.norm(edge)
        direction = edge / length
        normal = np.array([-direction[1], direction[0]])
        along = (pixels - start) @ direction
        across = (pixels - start) @ normal
        near = (along >= .08 * length) & (along <= .92 * length) & (np.abs(across) <= tolerance)
        if not np.any(near) or np.ptp(along[near]) < .35 * length:
            return None
        vx, vy, x, y = cv2.fitLine(pixels[near].astype(np.float32), cv2.DIST_HUBER, 0, .01, .01).ravel()
        line_normal = np.array([-vy, vx])
        lines.append((line_normal, line_normal @ [x, y]))

    corners = []
    for corner, (n1, d1), (n2, d2) in zip(coarse, lines[-1:] + lines[:-1], lines):
        matrix = np.array([n1, n2])
        if abs(np.linalg.det(matrix)) < .12:
            return None
        refined = np.linalg.solve(matrix, [d1, d2])
        if np.linalg.norm(refined - corner) > 4 * tolerance:
            return None
        corners.append(refined)
    return np.array(corners, np.float32)


def front_first(quad):
    """中央视角下按前左、前右、后右、后左排列。"""
    lengths = np.linalg.norm(np.roll(quad, -1, axis=0) - quad, axis=1)
    first = int(lengths[1] + lengths[3] > lengths[0] + lengths[2])
    front = max((first, first + 2), key=lambda i: quad[i, 1] + quad[(i + 1) % 4, 1])
    ordered = np.roll(quad, -front, axis=0)
    return ordered if ordered[0, 0] < ordered[1, 0] else ordered[[1, 0, 3, 2]]


def overlay(image, quad):
    """画外框、分区线和伤员区的进场方向。"""
    shown = image.copy()
    quad = front_first(quad)
    to_image = cv2.getPerspectiveTransform(FIELD_CORNERS_MM, quad)
    guides = np.float32([[0, 30], [0, 330], [155, -85], [155, 180]])
    divider_start, divider_end, arrow_start, arrow_end = np.rint(
        cv2.perspectiveTransform(guides[None], to_image)[0]
    ).astype(int)
    corners = np.rint(quad).astype(np.int32)
    cv2.polylines(shown, [corners], True, (60, 220, 255), 2, cv2.LINE_AA)
    cv2.line(shown, tuple(divider_start), tuple(divider_end), (255, 255, 255), 2, cv2.LINE_AA)
    cv2.arrowedLine(shown, tuple(arrow_start), tuple(arrow_end), (75, 245, 80), 4, cv2.LINE_AA, tipLength=.27)
    for corner in corners:
        cv2.circle(shown, tuple(corner), 4, (60, 220, 255), -1, cv2.LINE_AA)
    return shown


def main():
    detector = SafetyDetector()
    examples = []
    totals = dict(images=0, detected=0, full=0, ok=0, coarse_in_frame=0, fit=0)
    for folder in ('first', 'extra'):
        for path in sorted((ROOT / 'assets' / folder).glob('*.jpg')):
            if path.name.startswith('_'):
                continue
            image = cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR)
            safety = detector.detect(image)
            totals['images'] += 1
            for status in safety.status:
                totals[status] += safety.status[status]
            coarse, quad = None, None
            if safety.status['ok']:
                coarse = cv2.approxPolyN(safety.points, 4).reshape(4, 2).astype(np.float32)
                in_frame = bool(np.all((coarse >= 0) & (coarse < image.shape[1::-1])))
                totals['coarse_in_frame'] += in_frame
                quad = fit_quad(safety.points, safety.mask, safety.roi)
                totals['fit'] += quad is not None
                print(f'{path.stem}  coarse_in_frame={in_frame}  refined={quad is not None}')
            if path.stem in EXAMPLES:
                examples.append((path, image, safety, coarse, quad))

    rows = []
    for path, image, safety, coarse, quad in examples:
        _, _, polygon = safety.draw_result(image)
        rough = overlay(image, coarse) if coarse is not None else image
        result = overlay(image, quad) if quad is not None else image
        titles = ['Detected polygon', 'Approximate quad' if coarse is not None else 'Wait for full view',
                  'Refined quad' if quad is not None else 'No supported fit']
        tiles = []
        for frame, title in zip((polygon, rough, result), titles):
            tile = cv2.resize(frame, (400, 225))
            cv2.rectangle(tile, (0, 197), (400, 225), (35, 35, 35), -1)
            cv2.putText(tile, title, (10, 216), cv2.FONT_HERSHEY_SIMPLEX, .52, (245, 245, 245), 1, cv2.LINE_AA)
            tiles.append(tile)
        header = np.full((32, 1200, 3), 35, np.uint8)
        title = f'{path.stem}   full={safety.status["full"]}   ok={safety.status["ok"]}'
        cv2.putText(header, title, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, .58, (245, 245, 245), 1, cv2.LINE_AA)
        rows.extend([header, np.hstack(tiles)])
    out = ROOT / 'reports' / 'v2' / 'delivery_quad.jpg'
    out.parent.mkdir(parents=True, exist_ok=True)
    cv2.imencode('.jpg', np.vstack(rows))[1].tofile(out)
    print(totals)


if __name__ == '__main__':
    main()
