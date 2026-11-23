"""固定实拍图的检测凸包，只比较 DP 容差对四边形的影响。"""

from os import environ
from pathlib import Path
from statistics import median

import cv2
import numpy as np
from PIL import ImageFont

from .compare_heading import DetectorWithHull, fit_quad
from .compare_quad import caption, overlap
from .fit_quad import OUTPUT, WIDTH, draw


RATIOS = (0., .0005, .001, .002, .004)


def corner_shift(reference, quad):
    """对齐起点和绕行方向，返回四角中最大的移动距离。"""
    matches = [np.linalg.norm(reference - np.roll(points, start, axis=0), axis=1)
               for points in (quad, quad[::-1]) for start in range(4)]
    return float(min(matches, key=np.sum).max())


def main():
    detector = DetectorWithHull()
    fonts = Path(environ['WINDIR']) / 'Fonts'
    bold = ImageFont.truetype(str(fonts / 'msyhbd.ttc'), 23)
    small = ImageFont.truetype(str(fonts / 'msyh.ttc'), 17)
    rows = []
    seen = 0
    scores = {ratio: [] for ratio in RATIOS}
    for folder in ('first', 'extra'):
        for path in sorted((OUTPUT.parent / 'assets' / folder).glob('frame_*.jpg')):
            image = cv2.imdecode(np.fromfile(path, np.uint8), cv2.IMREAD_COLOR)
            safety = detector.detect(image)
            seen += 1
            if not safety.status['full']:
                continue
            hull = detector.hull
            raw, _ = fit_quad(hull)
            name = f'{path.stem}    full=True    ok={safety.status["ok"]}'
            rows.append(caption(name, bold, width=WIDTH * (len(RATIOS) + 1)))
            original = draw(image, safety.points, (255, 225, 60))
            tiles = [np.vstack([caption('v2 检测轮廓', bold, '供判色使用', small), original])]
            perimeter = cv2.arcLength(hull, True)
            for ratio in RATIOS:
                quad, used_n = fit_quad(hull, ratio)
                shift = corner_shift(raw, quad)
                excess = np.maximum(-quad, quad - (np.array(image.shape[1::-1]) - 1))
                overrun = int(max(0, excess.max()))
                scores[ratio].append((overrun, shift, overlap(hull.reshape(-1, 2), quad)))
                stage = '后接 N' if used_n else '直接四角'
                title = '原凸包直接 N' if ratio == 0 else f'DP {ratio*100:g}% {stage}'
                detail = (f'容差 {ratio*perimeter:.1f} px  ·  角移动 {shift:.1f} px'
                          if ratio else '作为比较角点移动的参考')
                if overrun:
                    detail += f'  ·  出画 {overrun} px'
                tiles.append(np.vstack([caption(title, bold, detail, small),
                                        draw(image, quad, (60, 235, 250), corners=True)]))
                print(f'{path.stem} {ratio*100:g}%: max corner shift {shift:.1f}px, overrun {overrun}px')
            rows.append(np.hstack(tiles))

    print(f'{seen} real photos, {len(scores[0.])} full')
    for ratio, rows_for_ratio in scores.items():
        print(f'{ratio*100:g}%: outside {sum(r[0]>0 for r in rows_for_ratio)}, '
              f'median max shift {median(r[1] for r in rows_for_ratio):.2f}px, '
              f'worst shift {max(r[1] for r in rows_for_ratio):.2f}px, '
              f'median hull IoU {median(r[2] for r in rows_for_ratio):.3f}')
    cv2.imencode('.jpg', np.vstack(rows), [cv2.IMWRITE_JPEG_QUALITY, 94])[1].tofile(OUTPUT / 'real_epsilon.jpg')


if __name__ == '__main__':
    main()
