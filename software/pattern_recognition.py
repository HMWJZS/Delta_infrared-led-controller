"""Current test-board pattern recognition; no pose or partial-occlusion claims."""
import csv
import itertools
import math
from pathlib import Path
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from ir_target_pose.estimator import (detect_bright_candidates, TargetModel, estimate_pose,
                                      rotation_to_euler_deg, rotation_distance_deg)


def load_coordinates():
    with (ROOT / 'patterns/LED_10x10_center_coordinates.csv').open(
            encoding='gb18030', newline='') as stream:
        rows = list(csv.reader(stream))[1:]
    points = np.full((100, 2), np.nan)
    for _, row, col, x, y in rows:
        if not (1 <= int(row) <= 10 and 1 <= int(col) <= 10):
            raise ValueError('测试板坐标行列必须为 1～10')
        index = (int(row) - 1) * 10 + int(col) - 1
        if not 0 <= index < 100 or np.isfinite(points[index]).any():
            raise ValueError('测试板坐标索引重复或越界')
        points[index] = float(x), float(y)
    if not np.isfinite(points).all():
        raise ValueError('测试板坐标必须包含完整 100 点')
    return points


def match_pattern(template, points, tolerance=3.0, inverse_camera=None, pixel_scale=1., unordered=False,
                  areas=None, all_orders=False):
    """Enumerate cyclic hull correspondences, then check every selected LED."""
    n = len(template)
    hull = cv2.convexHull(template.astype(np.float32), returnPoints=False).ravel()
    if len(hull) < 4:
        return '图案需至少四个凸包顶点', None
    if len(points) < n:
        return '未匹配：光点不足 / 灯灭 / 遮挡', None
    # ponytail: bounded exhaustive subsets; oversized clutter groups are rejected.
    if len(points) > n + 3 or math.comb(len(points), n) > 256:
        return '亮点过多，请提高阈值或减少背景亮点', None
    matches = {}
    for combo in itertools.combinations(range(len(points)), n):
        if areas is not None:
            selected = np.asarray(areas)[list(combo)]
            # ponytail: equal blocks at equal drive should have comparable cores;
            # reject >2x variation; a radiometric model is needed for unequal drive.
            if selected.max() > 2 * selected.min():
                continue
        observed = points[list(combo)]
        ih = cv2.convexHull(observed.astype(np.float32), returnPoints=False).ravel()
        if len(ih) != len(hull):
            continue
        # Nearly collinear background texture can fit a numerically collapsed homography.
        span = np.linalg.norm(np.ptp(observed, axis=0))
        if cv2.contourArea(observed[ih].astype(np.float32)) < .02 * span ** 2:
            continue
        for order in (ih, ih[::-1]):
            for offset in range(len(hull)):
                dst = observed[np.roll(order, offset)]
                H, _ = cv2.findHomography(template[hull], dst, 0)
                if H is None or not np.isfinite(H).all():
                    continue
                predicted = cv2.perspectiveTransform(template[None].astype(float), H)[0]
                distances = np.linalg.norm(predicted[:, None] - observed[None], axis=2)
                assignment = distances.argmin(axis=1)
                if len(set(assignment)) != n or distances[np.arange(n), assignment].max() > tolerance * 2:
                    continue
                ordered = observed[assignment]
                H, _ = cv2.findHomography(template, ordered, 0)
                if H is None or not np.isfinite(H).all():
                    continue
                if inverse_camera is not None:
                    # A calibrated rigid plane has equally scaled, orthogonal axes.
                    # Loose bounds allow pixel noise on tiny targets, but reject
                    # arbitrary projective fits to keyboard/monitor reflections.
                    axes = (inverse_camera @ H)[:, :2]
                    lengths = np.linalg.norm(axes, axis=0)
                    if (lengths.min() < 1e-12 or lengths.max() > 2 * lengths.min()
                            or abs(axes[:, 0] @ axes[:, 1]) > .5 * np.prod(lengths)):
                        continue
                # A single visible plane must not cross the projective horizon.
                denominator = np.column_stack((template, np.ones(n))) @ H[2]
                if not (np.all(denominator > 1e-8) or np.all(denominator < -1e-8)):
                    continue
                fitted = cv2.perspectiveTransform(template[None].astype(float), H)[0]
                errors = np.linalg.norm(fitted - ordered, axis=1)
                span = np.linalg.norm(np.ptp(ordered, axis=0))
                limit = min(tolerance, span * .015)
                separation = np.linalg.norm(ordered[:, None] - ordered[None], axis=2)
                np.fill_diagonal(separation, np.inf)
                # Resolve each pair rather than rejecting an otherwise accurate
                # small target solely because its bounding span is under 24 px.
                if separation.min() < 2 * pixel_scale or errors.max() > limit:
                    continue
                ids = tuple(combo[i] for i in assignment)
                matches[ids] = float(np.sqrt(np.mean(errors ** 2)))
    if all_orders:
        return '候选对应（不保证唯一）', [np.array(order) for order in sorted(matches, key=matches.get)]
    if unordered:
        # Same observed set under several symmetric labelings is one detected
        # layout. Different point sets must still compete and remain rejectable.
        sets = {}
        for order, error in matches.items():
            key = tuple(sorted(order))
            sets[key] = min(sets.get(key, float('inf')), error)
        matches = sets
    ranked = sorted(matches, key=matches.get)
    if not ranked:
        return '未匹配：几何关系不符', None
    if len(ranked) > 1 and matches[ranked[1]] - matches[ranked[0]] < .5 * pixel_scale:
        return '匹配歧义：图案对称或有多个相似目标', None
    return f'匹配当前图案 {n}/{n} · 误差 {matches[ranked[0]]:.2f} px', np.array(ranked[0])


def led_clusters(frame):
    """Configured equal rectangular LED blocks, not guessed subdivisions of pixels."""
    grid = np.asarray(frame).reshape(10, 10)
    count, labels = cv2.connectedComponents((grid > 0).astype(np.uint8), connectivity=4)
    groups = [np.flatnonzero(labels.ravel() == label) for label in range(1, count)]
    if len(groups) < 5 or len({len(group) for group in groups}) != 1 or len(groups[0]) < 2:
        return []
    if np.ptp(grid[grid > 0]) or len({(np.ptp(g // 10), np.ptp(g % 10)) for g in groups}) != 1:
        return []
    for group in groups:
        rows, cols = group // 10, group % 10
        if len(group) != (np.ptp(rows)+1) * (np.ptp(cols)+1) or np.ptp(grid.ravel()[group]):
            return []
    return groups


def pose_text(result):
    pose = result.get('pose')
    if pose is None:
        return result.get('pose_status', '位姿：等待识别')
    x, y, z = np.asarray(pose['tvec_m']) * 1000
    roll, pitch, yaw = pose['rpy_deg']
    return (f"{result['pose_status']}\n"
            f"相机坐标 XYZ：{x:+.1f} / {y:+.1f} / {z:.1f} mm   "
            f"RPY：{roll:+.1f}° / {pitch:+.1f}° / {yaw:+.1f}°   "
            f"重投影 RMS：{pose['rms_px']:.2f} px")


class PatternRecognizer:
    def __init__(self, K, D, image_size):
        self.K, self.D = K.copy(), D.copy()
        self.image_size = image_size
        self.coordinates = load_coordinates()
        self.origin_mm = self.coordinates.mean(0)

    def add_pose(self, result, template):
        """IPPE candidates in camera coordinates; score using native fisheye pixels."""
        raw = result['matched']
        orders = [np.arange(len(template))]
        if raw is None:
            raw = result['cluster_centers']
            corrected = cv2.fisheye.undistortPoints(raw[:, None], self.K, self.D, P=self.K)[:, 0]
            _, orders = match_pattern(template, corrected, 6., np.linalg.inv(self.K), 2., all_orders=True)
        xy = template - self.origin_mm
        objects = np.column_stack((xy, np.zeros(len(xy)))) / 1000
        model = TargetModel('test_board_v1', 'CSV LED centers', tuple(np.ptp(self.coordinates, axis=0)),
                            tuple(self.origin_mm), xy, objects, ())
        normalized = cv2.fisheye.undistortPoints(raw[:, None], self.K, self.D)[:, 0]
        candidates = []
        for order in orders:
            # The existing planar solver expects pinhole coordinates. Undistort
            # only the points to normalized rays; never pass fisheye D to PnP.
            for _, _, rvec, tvec, _ in estimate_pose(model, normalized[order], np.eye(3), np.zeros(4)):
                projected, _ = cv2.fisheye.projectPoints(objects[:, None], rvec, tvec, self.K, self.D)
                error = np.linalg.norm(projected[:, 0] - raw[order], axis=1)
                rms = float(np.sqrt(np.mean(error ** 2)))
                if not np.isfinite(error).all() or rms > 2. or error.max() > 3.:
                    continue
                if any(rotation_distance_deg(rvec, np.array(p['rvec'])) < .1
                       and np.linalg.norm(tvec.ravel() - p['tvec_m']) < 1e-5 for p in candidates):
                    continue
                candidates.append(dict(rvec=rvec.ravel().tolist(), tvec_m=tvec.ravel().tolist(),
                                       rpy_deg=list(rotation_to_euler_deg(rvec)), rms_px=rms,
                                       max_error_px=float(error.max())))
        candidates.sort(key=lambda p: p['rms_px'])
        result['pose_candidates'] = candidates
        if not candidates:
            result['pose_status'] = '已识别，位姿未通过正深度/重投影检查'
            return
        best = candidates[0]
        # ponytail: per-frame candidates, no temporal disambiguation or IMU fusion.
        ambiguous = result['orientation_ambiguous'] or (len(candidates) > 1
                       and candidates[1]['rms_px'] - best['rms_px'] < .25)
        result.update(pose=best, pose_ambiguous=ambiguous,
                      pose_status='位姿估计 · 最小重投影误差'
                                  + (' · 灯组光团近似' if result['kind'] == 'clusters' else ''))
        axes = np.array([[0., 0., 0.], [.015, 0., 0.], [0., .015, 0.], [0., 0., .015]])
        R, _ = cv2.Rodrigues(np.array(best['rvec']))
        if np.all((axes @ R.T + best['tvec_m'])[:, 2] > 0):
            projected, _ = cv2.fisheye.projectPoints(axes[:, None], np.array(best['rvec']),
                                                    np.array(best['tvec_m']), self.K, self.D)
            result['pose_axes_raw'] = projected[:, 0]

    def detect(self, image, frame, threshold):
        if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
            raise ValueError('识别输入必须为 uint8 BGR 原图')
        if len(frame) != 100 or not all(isinstance(v, (int, np.integer)) and 0 <= v <= 255 for v in frame):
            raise ValueError('图案必须包含 100 个 0～255 整数')
        if not 1 <= threshold <= 254:
            raise ValueError('识别阈值必须为 1～254')
        ids = np.flatnonzero(np.asarray(frame) > 0)
        clusters = led_clusters(frame)
        template = np.array([self.coordinates[group].mean(0) for group in clusters]) if clusters else self.coordinates[ids]
        border = np.array([i for i in range(100) if i // 10 in (0, 9) or i % 10 in (0, 9)])
        is_border = np.array_equal(ids, border)
        result = dict(status='', candidates=np.empty((0, 2)), matched=None, ids=ids,
                      detected=False, kind='border' if is_border else ('clusters' if clusters else 'points'),
                      outline=None, cluster_centers=None, orientation_ambiguous=is_border or bool(clusters),
                      pose=None, pose_candidates=[], pose_axes_raw=None, pose_ambiguous=False,
                      pose_status='位姿：未检出标靶')
        if self.image_size != (3840, 2160) or (image.shape[1], image.shape[0]) != self.image_size:
            result['status'] = '识别需要与内参一致的 3840×2160 图像'
            return result
        if len(ids) < 5:
            result['status'] = '当前图案需至少 5 个亮点（推荐非对称 7 点）'
            return result
        # DECXIN's recorded IR spots have both red and blue above green. Preserve
        # native pixels and separate these spots from white paper at the board edge.
        # ponytail: this color cue is camera-specific; retain grayscale for neutral IR images.
        br = cv2.min(image[:, :, 0], image[:, :, 2])
        infrared = cv2.bitwise_and(br, cv2.compare(cv2.subtract(br, image[:, :, 1]), 25, cv2.CMP_GT))
        K = self.K.copy()
        # No resize or crop: extraction and output use the entire native 4K frame.
        # The corrected plane also uses the original K; its error unit is now 4K pixels.
        inverse_camera = np.linalg.inv(K)
        def to_plane(points):
            return cv2.fisheye.undistortPoints(points[:, None], self.K, self.D, P=K)[:, 0]
        # Saturated blocks have white cores; color gating can cut one block into
        # several rims. Use the unmodified grayscale intensity for block centers.
        for color_pass in ((False,) if clusters else (True, False)):
            gray = infrared if color_pass else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            if is_border:
                status, outline, used = detect_border(gray, threshold, to_plane)
                result.update(status=status + f' · 阈值 {used} · 4K 全图',
                              outline=outline, detected=outline is not None)
                if outline is not None or '歧义' in status:
                    break
                continue
            status, candidates, order, used = detect_region(
                gray, template, threshold, to_plane, inverse_camera,
                pixel_scale=2., search_dim=color_pass, unordered=bool(clusters))
            result['status'] = status + f' · 阈值 {used} · 4K 全图' + (' · 原像素色差' if color_pass else '')
            result['candidates'] = candidates
            if order is not None:
                if clusters:
                    result['cluster_centers'] = result['candidates'][order]
                    result['status'] = (f'已检出 {len(clusters)}/{len(clusters)} 灯组（{len(ids)} 灯）'
                                        f' · 方向/灯号不唯一 · 阈值 {used} · 4K 全图')
                else:
                    result['matched'] = result['candidates'][order]
                result['detected'] = True
                break
            if '歧义' in status:
                break
        if result['detected']:
            if is_border:
                result['pose_status'] = '已识别连续边框 · 无可靠灯点对应，暂不输出位姿'
            else:
                self.add_pose(result, template)
        return result


def detect_border(gray, threshold, to_plane):
    """Detect a continuous luminous ring, never invent 36 individual LED centers."""
    for value in sorted(set([threshold, *range(threshold + 20, 241, 20)])):
        _, binary = cv2.threshold(cv2.GaussianBlur(gray, (0, 0), .6), value, 255, cv2.THRESH_BINARY)
        contours, hierarchy = cv2.findContours(binary, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        rings = []
        for index, contour in enumerate(contours):
            child = hierarchy[0, index, 2]
            if child < 0 or cv2.contourArea(contour) < 200:
                continue
            outer = to_plane(contour[:, 0].astype(float)).astype(np.float32)
            inner = to_plane(contours[child][:, 0].astype(float)).astype(np.float32)
            if not np.isfinite(outer).all() or not np.isfinite(inner).all():
                continue
            area = cv2.contourArea(outer)
            if area < 200 or not .3 < cv2.contourArea(inner) / area < .93:
                continue
            quads = [cv2.approxPolyDP(c, .03 * cv2.arcLength(c, True), True)[:, 0] for c in (outer, inner)]
            if any(len(q) != 4 or not cv2.isContourConvex(q) for q in quads):
                continue
            q = quads[0]
            if abs(cv2.contourArea(q) / area - 1) > .12:
                continue
            # The inner hole must occupy all four sides of the rectified outer ring.
            H = cv2.getPerspectiveTransform(q, np.float32([[0, 0], [1, 0], [1, 1], [0, 1]]))
            hole = cv2.perspectiveTransform(quads[1][None], H)[0]
            lo, hi = hole.min(0), hole.max(0)
            if not (np.all(lo > -.03) and np.all(lo < .25) and np.all(hi > .75) and np.all(hi < 1.03)):
                continue
            raw = contour[:, 0].astype(float)
            # Saturated color cores can create concentric ring contours on one board.
            center, size = raw.mean(0), np.linalg.norm(np.ptp(raw, axis=0))
            if not any(np.linalg.norm(center - other.mean(0)) < .15 * min(size, np.linalg.norm(np.ptp(other, axis=0))) for other in rings):
                rings.append(raw)
        if len(rings) > 1:
            return '边框歧义：检测到多个连续边框', None, value
        if rings:
            return '已检出 10×10 连续边框 · 对称方向不确定，未分离 36 灯', rings[0], value
    return '未检出连续边框：检查断边、遮挡或光点是否分离', None, value


def candidate_groups(candidates, expected):
    """Separate nearby spots of comparable area before geometric subset matching."""
    if len(candidates) <= expected + 3:
        return [list(range(len(candidates)))]
    points = np.array([c.center for c in candidates])
    area = np.array([c.core_area for c in candidates])
    smaller = np.minimum(area[:, None], area[None])
    distance = np.linalg.norm(points[:, None] - points[None], axis=2)
    comparable = np.maximum(area[:, None], area[None]) <= 4 * smaller
    groups = set()
    # Multiple radii cover compact saturated spots and sparse, sharply focused LEDs.
    for radius in (8, 16, 32):
        adjacent = (distance <= radius * np.sqrt(smaller)) & comparable
        seen = set()
        for seed in range(len(candidates)):
            if seed in seen:
                continue
            group, pending = {seed}, [seed]
            seen.add(seed)
            while pending:
                neighbors = set(np.flatnonzero(adjacent[pending.pop()])) - seen
                group.update(neighbors)
                seen.update(neighbors)
                pending.extend(neighbors)
            if expected <= len(group) <= expected + 3:
                groups.add(tuple(sorted(group)))
    return [list(group) for group in sorted(groups)]


def detect_region(gray, template, threshold, to_plane=lambda points: points, inverse_camera=None,
                  pixel_scale=1., search_dim=False, unordered=False):
    """Brightness sweep and local grouping; keep all matches to reject duplicate targets."""
    last = ('未匹配', np.empty((0, 2)), None, threshold)
    budget = 2048 if gray.shape == (2160, 3840) else 512
    values = sorted(set([threshold, *range(threshold + 20, 241, 20)]))
    # Only the color-filtered native image gets a dim search. Small faint spots
    # can separate only in a narrow threshold interval; 20-level steps skip it.
    if search_dim:
        values.extend(range(threshold - 5, 39, -5))
    for value in values:
        # No fixed 800-pixel ceiling: a genuine near-camera LED can be larger.
        # ponytail: bounded quadratic grouping; native 4K gets four times the component budget.
        candidates, _, _ = detect_bright_candidates(
            gray, len(template), value, .6, 3, 5, 20, 2, gray.size, budget + 1,
            allow_peak_fallback=False)
        points = np.asarray([c.center for c in candidates], dtype=float).reshape(-1, 2)
        if len(points) > budget:
            last = (f'全图亮点超过 {budget} 个，请调整阈值或曝光', points, None, value)
            continue
        if len(points) < len(template):
            last = (f'未匹配：检测到 {len(points)}/{len(template)} 点', points, None, value)
            continue
        successes, ambiguous = [], False
        status = '未匹配：全图未找到完整光点组'
        for group in candidate_groups(candidates, len(template)):
            corrected = to_plane(points[group])
            if not np.isfinite(corrected).all() or np.abs(corrected).max() > 1e6:
                continue  # One edge reflection must not reject other groups in the frame.
            status, order = match_pattern(template, corrected, tolerance=3. * pixel_scale,
                                          inverse_camera=inverse_camera, pixel_scale=pixel_scale,
                                          unordered=unordered,
                                          areas=[candidates[i].core_area for i in group] if unordered else None)
            ambiguous |= '歧义' in status
            if order is not None:
                successes.append((status, np.asarray(group)[order]))
        successes = list({tuple(order): (status, order) for status, order in successes}.values())
        if len(successes) > 1:
            return ('匹配歧义：多个相似目标，请使用不同图案', points, None, value)
        if successes and ambiguous:
            last = ('匹配歧义：另有可疑光点组', points, None, value)
            continue  # A higher threshold can remove an ambiguous reflection group.
        if successes:
            return (successes[0][0], points, successes[0][1], value)
        last = ('匹配歧义：图案对称或有多个相似目标' if ambiguous else status, points, None, value)
    return last
