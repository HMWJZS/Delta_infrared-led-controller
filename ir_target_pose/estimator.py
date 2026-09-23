from __future__ import annotations

import functools
import itertools
import json
import math
from dataclasses import dataclass, replace
from pathlib import Path

import cv2
import numpy as np


@dataclass(frozen=True)
class TargetModel:
    layout: str
    description: str
    board_size_mm: tuple[float, float]
    origin_mm: tuple[float, float]
    template_points_mm: np.ndarray
    object_points: np.ndarray
    hull_ids: tuple[int, ...]

    @property
    def point_count(self) -> int:
        return len(self.template_points_mm)


@dataclass(frozen=True)
class TargetSelection:
    target_id: str
    model: TargetModel


@dataclass
class PoseEstimate:
    score: float
    rms: float
    maximum: float
    geometry_error: float
    image_points: np.ndarray
    point_ids: np.ndarray
    rvec: np.ndarray
    tvec: np.ndarray
    projected: np.ndarray

    @property
    def matched_count(self) -> int:
        return len(self.point_ids)


@dataclass(frozen=True)
class CameraIntrinsics:
    """OpenCV pinhole intrinsics for one exact image resolution."""

    K: np.ndarray
    D: np.ndarray
    image_size: tuple[int, int]
    camera_model: str = "opencv_pinhole"

    def __post_init__(self) -> None:
        if self.camera_model != "opencv_pinhole":
            raise ValueError(f"camera_model 必须为 opencv_pinhole，实际为 {self.camera_model!r}")
        K = np.asarray(self.K, dtype=np.float64)
        if K.shape != (3, 3) or not np.all(np.isfinite(K)):
            raise ValueError("K 必须是有限的 3x3 数组")
        if K[0, 0] <= 0 or K[1, 1] <= 0:
            raise ValueError("K 中的 fx 和 fy 必须为正数")
        D = np.asarray(self.D, dtype=np.float64)
        if D.ndim == 1:
            D = D.reshape(-1, 1)
        if D.ndim != 2 or D.shape[1] != 1 or len(D) not in (4, 5, 8, 12, 14):
            raise ValueError("D 必须是一维或单列数组，长度为 4、5、8、12 或 14")
        if not np.all(np.isfinite(D)):
            raise ValueError("D 必须只包含有限值")
        if not isinstance(self.image_size, (tuple, list)) or len(self.image_size) != 2:
            raise ValueError("image_size 必须为 (width, height)")
        width, height = self.image_size
        if (
            isinstance(width, (bool, np.bool_))
            or isinstance(height, (bool, np.bool_))
            or not isinstance(width, (int, np.integer))
            or not isinstance(height, (int, np.integer))
            or width <= 0
            or height <= 0
        ):
            raise ValueError("image_size 的 width 和 height 必须为正整数")
        object.__setattr__(self, "K", np.ascontiguousarray(K).copy())
        object.__setattr__(self, "D", np.ascontiguousarray(D).copy())
        object.__setattr__(self, "image_size", (int(width), int(height)))

    @classmethod
    def from_json(cls, path: str | Path) -> "CameraIntrinsics":
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        try:
            image_size = (data["image_width"], data["image_height"])
            return cls(
                K=data["K"],
                D=data["D"],
                image_size=image_size,
                camera_model=data.get("camera_model", ""),
            )
        except KeyError as error:
            raise ValueError(f"内参 JSON 缺少字段: {error.args[0]}") from error


@dataclass(frozen=True)
class DetectionParameters:
    threshold: int
    blur_sigma: float
    min_peak_distance: int
    core_radius: int
    core_drop: int
    min_core_area: int
    max_core_area: int
    max_candidates: int
    allow_peak_fallback: bool
    max_geometry_error: float
    max_reprojection_error: float
    min_match_margin: float
    max_area_cv: float | None = None

    def __post_init__(self) -> None:
        if (
            isinstance(self.threshold, (bool, np.bool_))
            or not isinstance(self.threshold, (int, np.integer))
            or not 0 <= self.threshold <= 255
        ):
            raise ValueError("threshold 必须是 0 到 255 的整数")
        for name in ("min_peak_distance", "core_radius", "min_core_area", "max_core_area", "max_candidates"):
            value = getattr(self, name)
            if not isinstance(value, (int, np.integer)) or isinstance(value, (bool, np.bool_)) or value <= 0:
                raise ValueError(f"{name} 必须为正整数")
        if (
            isinstance(self.core_drop, (bool, np.bool_))
            or not isinstance(self.core_drop, (int, np.integer))
            or not 0 <= self.core_drop <= 255
        ):
            raise ValueError("core_drop 必须是 0 到 255 的整数")
        if not isinstance(self.allow_peak_fallback, (bool, np.bool_)):
            raise ValueError("allow_peak_fallback 必须为布尔值")
        if self.min_core_area > self.max_core_area:
            raise ValueError("min_core_area 不能大于 max_core_area")
        for name in ("blur_sigma", "max_geometry_error", "max_reprojection_error"):
            value = getattr(self, name)
            if (
                isinstance(value, (bool, np.bool_))
                or not isinstance(value, (int, float, np.integer, np.floating))
                or not math.isfinite(float(value))
                or value <= 0
            ):
                raise ValueError(f"{name} 必须为有限正数")
        if (
            isinstance(self.min_match_margin, (bool, np.bool_))
            or not isinstance(self.min_match_margin, (int, float, np.integer, np.floating))
            or not math.isfinite(float(self.min_match_margin))
            or self.min_match_margin < 0
        ):
            raise ValueError("min_match_margin 必须为有限非负数")
        if self.max_area_cv is not None and (
            isinstance(self.max_area_cv, (bool, np.bool_))
            or not isinstance(self.max_area_cv, (int, float, np.integer, np.floating))
            or not math.isfinite(float(self.max_area_cv))
            or self.max_area_cv <= 0
        ):
            raise ValueError("max_area_cv 必须为有限正数或 None")


@dataclass(frozen=True)
class PoseResult:
    """Result of one independent image estimate."""

    ok: bool
    status: str
    target_id: str
    wavelength_nm: int
    support_status: str
    candidate_count: int
    matched_count: int = 0
    T_camera_target: np.ndarray | None = None
    rvec: np.ndarray | None = None
    tvec_m: np.ndarray | None = None
    image_points: np.ndarray | None = None
    point_ids: np.ndarray | None = None
    reprojection_rms_px: float | None = None
    reprojection_max_px: float | None = None
    geometry_error_px: float | None = None
    T_device_target: np.ndarray | None = None


def _validate_rigid_transform(value: np.ndarray, name: str) -> np.ndarray:
    transform = np.asarray(value, dtype=np.float64)
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise ValueError(f"{name} 必须是有限的 4x4 数组")
    if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-9):
        raise ValueError(f"{name} 最后一行必须为 [0, 0, 0, 1]")
    rotation = transform[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6) or not np.isclose(
        np.linalg.det(rotation), 1.0, atol=1e-6
    ):
        raise ValueError(f"{name} 旋转部分必须是正交且行列式为 1")
    return np.ascontiguousarray(transform).copy()


def make_target_model(
    layout: str,
    description: str,
    board_size_mm: tuple[float, float],
    origin_mm: tuple[float, float],
    points_mm: list[tuple[float, float]],
) -> TargetModel:
    template_points = np.asarray(points_mm, dtype=np.float32)
    object_points = np.column_stack(
        (
            (template_points[:, 0] - origin_mm[0]) / 1000.0,
            -(template_points[:, 1] - origin_mm[1]) / 1000.0,
            np.zeros(len(template_points), dtype=np.float32),
        )
    ).astype(np.float64)
    hull = cv2.convexHull(template_points, returnPoints=False)
    if hull is None or len(hull) < 4:
        raise ValueError(f"{layout} 至少需要四个凸包点")
    return TargetModel(
        layout=layout,
        description=description,
        board_size_mm=board_size_mm,
        origin_mm=origin_mm,
        template_points_mm=template_points,
        object_points=object_points,
        hull_ids=tuple(int(value) for value in hull.reshape(-1)),
    )


TARGET_MODELS = {
    "LEGACY5": make_target_model(
        "LEGACY5",
        "旧版实测五灯，P2 为原点",
        (86.5, 25.5),
        (36.0, 17.5),
        [(0.0, 0.0), (0.0, 25.5), (36.0, 17.5), (76.5, 0.0), (86.5, 25.5)],
    ),
    "L5": make_target_model(
        "L5",
        "100 × 30 mm 长方形五灯",
        (100.0, 30.0),
        (50.0, 15.0),
        [(7.0, 5.0), (7.0, 25.0), (45.0, 17.0), (84.0, 6.0), (93.0, 24.0)],
    ),
    "L7": make_target_model(
        "L7",
        "100 × 30 mm 长方形七灯",
        (100.0, 30.0),
        (50.0, 15.0),
        [(7.0, 5.0), (7.0, 25.0), (29.0, 8.0), (39.0, 23.0), (54.0, 14.0), (76.0, 6.0), (93.0, 24.0)],
    ),
    "S5": make_target_model(
        "S5",
        "Ø30 mm 圆形五灯",
        (30.0, 30.0),
        (15.0, 15.0),
        [(7.0, 7.0), (6.0, 21.0), (14.0, 16.0), (21.0, 8.0), (23.0, 21.0)],
    ),
    "S7": make_target_model(
        "S7",
        "Ø30 mm 圆形七灯",
        (30.0, 30.0),
        (15.0, 15.0),
        [(7.0, 7.0), (6.0, 21.0), (12.0, 14.0), (16.0, 6.0), (16.0, 23.0), (23.0, 11.0), (23.0, 22.0)],
    ),
}

TARGET_IDS = {
    "IR-L5-R-850-5": "L5",
    "IR-L5-R-850-3": "L5",
    "IR-L7-R-850-5": "L7",
    "IR-S5-R-850-3": "S5",
    "IR-S7-R-850-3": "S7",
    "IR-L5-R-940-5": "L5",
    "IR-L5-R-940-3": "L5",
    "IR-S5-R-940-3": "S5",
}

TARGET_SUPPORT_STATUS = {
    "IR-L5-R-850-5": "supported",
    "IR-L5-R-850-3": "supported",
    "IR-L7-R-850-5": "supported",
    "IR-L5-R-940-5": "experimental",
    "IR-L5-R-940-3": "experimental",
    "IR-S5-R-850-3": "geometry_only",
    "IR-S7-R-850-3": "geometry_only",
    "IR-S5-R-940-3": "geometry_only",
}
SUPPORTED_TARGET_IDS = tuple(
    target_id for target_id, status in TARGET_SUPPORT_STATUS.items() if status != "geometry_only"
)

PARAMETERS_850 = DetectionParameters(
    threshold=235,
    blur_sigma=1.2,
    min_peak_distance=10,
    core_radius=8,
    core_drop=40,
    min_core_area=100,
    max_core_area=20000,
    max_candidates=8,
    allow_peak_fallback=True,
    max_geometry_error=15.0,
    max_reprojection_error=5.0,
    min_match_margin=0.10,
)
PARAMETERS_940_5 = DetectionParameters(
    threshold=140,
    blur_sigma=1.2,
    min_peak_distance=20,
    core_radius=16,
    core_drop=40,
    min_core_area=15,
    max_core_area=20000,
    max_candidates=10,
    allow_peak_fallback=False,
    max_geometry_error=45.0,
    max_reprojection_error=8.0,
    min_match_margin=0.10,
    max_area_cv=0.70,
)
PARAMETERS_940_3 = replace(PARAMETERS_940_5, threshold=200)

TARGET_DEFAULT_PARAMETERS = {
    "IR-L5-R-850-5": PARAMETERS_850,
    "IR-L5-R-850-3": PARAMETERS_850,
    "IR-L7-R-850-5": PARAMETERS_850,
    "IR-S5-R-850-3": PARAMETERS_850,
    "IR-S7-R-850-3": PARAMETERS_850,
    "IR-L5-R-940-5": PARAMETERS_940_5,
    "IR-L5-R-940-3": PARAMETERS_940_3,
    "IR-S5-R-940-3": PARAMETERS_940_3,
}


def resolve_target(value: str) -> TargetSelection:
    target_id = value.strip().upper()
    if target_id in TARGET_MODELS:
        return TargetSelection(target_id=target_id, model=TARGET_MODELS[target_id])
    layout = TARGET_IDS.get(target_id)
    if layout is None:
        supported = ", ".join([*TARGET_MODELS, *TARGET_IDS])
        raise ValueError(f"未知标靶 {value!r}；支持: {supported}")
    return TargetSelection(target_id=target_id, model=TARGET_MODELS[layout])


def target_slug(target_id: str) -> str:
    return "".join(character.lower() if character.isalnum() else "_" for character in target_id).strip("_")


@dataclass
class BrightCandidate:
    center: np.ndarray
    peak: int
    core_area: int


def load_pinhole_intrinsics(path: Path) -> tuple[np.ndarray, np.ndarray, tuple[int, int]]:
    intrinsics = CameraIntrinsics.from_json(path)
    return intrinsics.K.copy(), intrinsics.D.copy(), intrinsics.image_size


def make_peak_seeds(smooth: np.ndarray, threshold: int, min_distance: int) -> list[tuple[int, int, int]]:
    # A 3x3 maximum filter is enough to produce seeds. The requested physical
    # separation is enforced later by sub-pixel non-maximum suppression; using
    # a large dilation kernel here is unnecessarily expensive at 1280x720.
    kernel = np.ones((3, 3), dtype=np.uint8)
    local_max = cv2.dilate(smooth, kernel)
    peak_mask = ((smooth >= threshold) & (smooth == local_max)).astype(np.uint8)
    seeds = []
    contours, _ = cv2.findContours(peak_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        x0, y0, width, height = cv2.boundingRect(contour)
        moments = cv2.moments(contour)
        if moments["m00"] > 0:
            cx = moments["m10"] / moments["m00"]
            cy = moments["m01"] / moments["m00"]
        else:
            cx = x0 + (width - 1) / 2.0
            cy = y0 + (height - 1) / 2.0
        x = int(round(cx))
        y = int(round(cy))
        if 0 <= x < smooth.shape[1] and 0 <= y < smooth.shape[0]:
            seeds.append((int(smooth[y, x]), x, y))
    seeds.sort(reverse=True)
    return seeds


def refine_bright_center(
    gray: np.ndarray,
    seed: tuple[int, int, int],
    threshold: int,
    radius: int,
    core_drop: int,
) -> BrightCandidate | None:
    peak, x, y = seed
    y0, y1 = max(0, y - radius), min(gray.shape[0], y + radius + 1)
    x0, x1 = max(0, x - radius), min(gray.shape[1], x + radius + 1)
    patch = gray[y0:y1, x0:x1].astype(np.float32)
    core_threshold = max(int(threshold), int(peak) - int(core_drop))
    core = patch >= core_threshold
    core_area = int(np.count_nonzero(core))
    if core_area == 0:
        return None
    weights = np.where(core, np.maximum(patch - core_threshold + 1.0, 0.0), 0.0)
    total = float(weights.sum())
    if total <= 0:
        return None
    ys, xs = np.indices(patch.shape, dtype=np.float32)
    cx = float(x0 + (xs * weights).sum() / total)
    cy = float(y0 + (ys * weights).sum() / total)
    return BrightCandidate(np.array([cx, cy], dtype=np.float32), int(peak), core_area)


def split_component_centers(
    gray: np.ndarray,
    component_labels: np.ndarray,
    label: int,
    threshold: int,
    cluster_count: int,
) -> list[BrightCandidate]:
    """Split one merged high-threshold component into spatial LED centres.

    A saturated acrylic halo often joins two adjacent LEDs.  Generic local
    maxima are unsafe here because a flat saturated plateau can produce many
    maxima in one physical spot.  Deterministic 1-D PCA initialization followed
    by spatial Lloyd iterations produces exactly the requested number of lobes.
    """
    ys, xs = np.nonzero(component_labels == label)
    if len(xs) < cluster_count:
        return []
    points = np.column_stack((xs, ys)).astype(np.float64)
    centered = points - points.mean(axis=0)
    covariance = centered.T @ centered
    _, eigenvectors = np.linalg.eigh(covariance)
    axis = eigenvectors[:, -1]
    projection = centered @ axis
    order = np.argsort(projection)
    groups = np.array_split(order, cluster_count)
    centers = np.asarray([points[group].mean(axis=0) for group in groups], dtype=np.float64)
    assignments = np.zeros(len(points), dtype=np.int32)
    for _ in range(12):
        distances = np.linalg.norm(points[:, None, :] - centers[None, :, :], axis=2)
        new_assignments = np.argmin(distances, axis=1)
        if np.array_equal(assignments, new_assignments):
            break
        assignments = new_assignments
        for cluster_id in range(cluster_count):
            members = points[assignments == cluster_id]
            if len(members):
                centers[cluster_id] = members.mean(axis=0)

    result: list[BrightCandidate] = []
    for cluster_id in range(cluster_count):
        member_indices = np.flatnonzero(assignments == cluster_id)
        if len(member_indices) == 0:
            return []
        member_points = points[member_indices]
        member_x = member_points[:, 0].astype(np.int32)
        member_y = member_points[:, 1].astype(np.int32)
        intensities = gray[member_y, member_x].astype(np.float64)
        weights = np.maximum(intensities - threshold + 1.0, 1.0)
        center = np.average(member_points, axis=0, weights=weights).astype(np.float32)
        result.append(BrightCandidate(center, int(intensities.max()), len(member_indices)))
    return result


def detect_bright_candidates(
    gray: np.ndarray,
    expected_point_count: int,
    threshold: int,
    blur_sigma: float,
    min_peak_distance: int,
    core_radius: int,
    core_drop: int,
    min_core_area: int,
    max_core_area: int,
    max_candidates: int,
    allow_peak_fallback: bool = True,
) -> tuple[list[BrightCandidate], np.ndarray, np.ndarray]:
    smooth = cv2.GaussianBlur(gray, (0, 0), blur_sigma)
    _, binary = cv2.threshold(smooth, threshold, 255, cv2.THRESH_BINARY)
    component_count, component_labels, component_stats, _ = cv2.connectedComponentsWithStats(binary, connectivity=8)
    component_candidates: list[BrightCandidate] = []
    component_records: list[tuple[int, BrightCandidate]] = []
    for label in range(1, component_count):
        area = int(component_stats[label, cv2.CC_STAT_AREA])
        if not (min_core_area <= area <= max_core_area):
            continue
        x0 = int(component_stats[label, cv2.CC_STAT_LEFT])
        y0 = int(component_stats[label, cv2.CC_STAT_TOP])
        width = int(component_stats[label, cv2.CC_STAT_WIDTH])
        height = int(component_stats[label, cv2.CC_STAT_HEIGHT])
        patch = gray[y0 : y0 + height, x0 : x0 + width].astype(np.float32)
        label_patch = component_labels[y0 : y0 + height, x0 : x0 + width] == label
        weights = np.where(label_patch, np.maximum(patch - threshold + 1.0, 0.0), 0.0)
        total = float(weights.sum())
        if total <= 0:
            continue
        ys, xs = np.indices(patch.shape, dtype=np.float32)
        center = np.array(
            [x0 + float((xs * weights).sum() / total), y0 + float((ys * weights).sum() / total)],
            dtype=np.float32,
        )
        peak = int(patch[label_patch].max())
        candidate = BrightCandidate(center, peak, area)
        component_candidates.append(candidate)
        component_records.append((label, candidate))

    # Distinct high-threshold components are more stable than multiple local
    # maxima inside one acrylic halo. Use local peaks only when fewer than the
    # requested number of independent cores remain.
    if len(component_candidates) >= expected_point_count:
        component_candidates.sort(key=lambda item: (item.peak, item.core_area), reverse=True)
        return component_candidates[:max_candidates], smooth, binary

    if not allow_peak_fallback:
        component_candidates.sort(key=lambda item: (item.peak, item.core_area), reverse=True)
        return component_candidates[:max_candidates], smooth, binary

    # Infer a two-LED merged halo from its area relative to the other independent
    # cores, then split only that component.  This avoids harvesting several
    # false maxima from one saturated plateau.
    if 2 <= len(component_candidates) < expected_point_count:
        if expected_point_count == 5 and len(component_candidates) == 3:
            by_area = sorted(component_records, key=lambda item: item[1].core_area, reverse=True)
            if by_area[1][1].core_area >= 1.5 * by_area[2][1].core_area:
                split_pairs = [
                    split_component_centers(gray, component_labels, label, threshold, 2)
                    for label, _ in by_area[:2]
                ]
                if all(len(pair) == 2 for pair in split_pairs):
                    rebuilt = [*split_pairs[0], *split_pairs[1], by_area[2][1]]
                    rebuilt.sort(key=lambda item: (item.peak, item.core_area), reverse=True)
                    return rebuilt[:max_candidates], smooth, binary
        typical_area = float(np.median([item.core_area for item in component_candidates]))
        missing = expected_point_count - len(component_candidates)
        rebuilt: list[BrightCandidate] = []
        remaining_splits = missing
        for label, candidate in sorted(component_records, key=lambda item: item[1].core_area, reverse=True):
            ratio = candidate.core_area / max(typical_area, 1.0)
            extra = min(remaining_splits, max(0, int(round(ratio)) - 1)) if ratio >= 1.65 else 0
            if extra:
                split = split_component_centers(gray, component_labels, label, threshold, 1 + extra)
                if len(split) == 1 + extra:
                    rebuilt.extend(split)
                    remaining_splits -= extra
                    continue
            rebuilt.append(candidate)
        if len(rebuilt) >= expected_point_count:
            rebuilt.sort(key=lambda item: (item.peak, item.core_area), reverse=True)
            return rebuilt[:max_candidates], smooth, binary

    seeds = make_peak_seeds(smooth, threshold, min_peak_distance)
    candidates: list[BrightCandidate] = []
    for seed in seeds:
        candidate = refine_bright_center(gray, seed, threshold, core_radius, core_drop)
        if candidate is None or not (min_core_area <= candidate.core_area <= max_core_area):
            continue
        if any(np.linalg.norm(candidate.center - other.center) < min_peak_distance for other in candidates):
            continue
        candidates.append(candidate)
        if len(candidates) >= max_candidates:
            break
    return candidates, smooth, binary


def candidate_geometry_matches(
    candidates: list[BrightCandidate],
    model: TargetModel,
    max_geometry_error: float,
) -> list[tuple[float, np.ndarray]]:
    if len(candidates) < model.point_count:
        return []
    points = np.asarray([candidate.center for candidate in candidates], dtype=np.float32)
    template_hull_ids = list(model.hull_ids)
    template_inner_ids = sorted(set(range(model.point_count)) - set(template_hull_ids))
    matches: list[tuple[float, np.ndarray]] = []
    for combo in itertools.combinations(range(len(candidates)), model.point_count):
        combo_points = points[list(combo)]
        hull_local = cv2.convexHull(combo_points, returnPoints=False)
        if hull_local is None or len(hull_local) != len(template_hull_ids):
            continue
        image_hull_ids = [combo[int(value)] for value in hull_local.reshape(-1)]
        image_inner_ids = sorted(set(combo) - set(image_hull_ids))
        for reverse in (False, True):
            oriented_hull_ids = list(reversed(image_hull_ids)) if reverse else image_hull_ids
            for offset in range(len(oriented_hull_ids)):
                ordered_hull_ids = oriented_hull_ids[offset:] + oriented_hull_ids[:offset]
                src = model.template_points_mm[template_hull_ids].astype(np.float32)
                dst = points[ordered_hull_ids].astype(np.float32)
                if len(template_hull_ids) == 4:
                    H = cv2.getPerspectiveTransform(src, dst)
                else:
                    H, _ = cv2.findHomography(src, dst, method=0)
                if H is None or not np.all(np.isfinite(H)):
                    continue
                predicted = cv2.perspectiveTransform(
                    model.template_points_mm.reshape(1, -1, 2),
                    H,
                ).reshape(-1, 2)
                inner_orders = itertools.permutations(image_inner_ids)
                for ordered_inner_ids in inner_orders:
                    ordered = np.empty((model.point_count, 2), dtype=np.float64)
                    for template_id, image_id in zip(template_hull_ids, ordered_hull_ids):
                        ordered[template_id] = points[image_id]
                    for template_id, image_id in zip(template_inner_ids, ordered_inner_ids):
                        ordered[template_id] = points[image_id]
                    errors = np.linalg.norm(predicted - ordered, axis=1)
                    maximum = float(errors.max())
                    if not math.isfinite(maximum) or maximum > max_geometry_error:
                        continue
                    geometry_rms = float(np.sqrt(np.mean(errors**2)))
                    matches.append((geometry_rms, ordered))
    matches.sort(key=lambda item: item[0])
    return matches[:24]


def reprojection_errors(
    object_points: np.ndarray,
    image_points: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    K: np.ndarray,
    D: np.ndarray,
) -> tuple[float, float, np.ndarray]:
    projected, _ = cv2.projectPoints(object_points, rvec, tvec, K, D)
    projected = projected.reshape(-1, 2)
    errors = np.linalg.norm(projected - image_points, axis=1)
    return float(np.sqrt(np.mean(errors**2))), float(errors.max()), projected


def estimate_pose(
    model: TargetModel,
    image_points: np.ndarray,
    K: np.ndarray,
    D: np.ndarray,
    point_ids: np.ndarray | None = None,
) -> list[tuple[float, float, np.ndarray, np.ndarray, np.ndarray]]:
    if point_ids is None:
        point_ids = np.arange(model.point_count, dtype=np.int32)
    else:
        point_ids = np.asarray(point_ids, dtype=np.int32)
    if len(point_ids) < 4:
        return []
    object_points = model.object_points[point_ids]
    matched_image_points = np.asarray(image_points, dtype=np.float64).reshape(-1, 2)
    try:
        result = cv2.solvePnPGeneric(
            object_points,
            matched_image_points.reshape(-1, 1, 2),
            K,
            D,
            flags=cv2.SOLVEPNP_IPPE,
        )
    except cv2.error:
        return []
    if not bool(result[0]):
        return []
    solutions = []
    for rvec, tvec in zip(result[1], result[2]):
        if not (np.all(np.isfinite(rvec)) and np.all(np.isfinite(tvec))):
            continue
        R, _ = cv2.Rodrigues(rvec)
        camera_points = (R @ model.object_points.T + tvec.reshape(3, 1)).T
        if np.any(camera_points[:, 2] <= 0):
            continue
        rms, maximum, _ = reprojection_errors(
            object_points,
            matched_image_points,
            rvec,
            tvec,
            K,
            D,
        )
        projected, _ = cv2.projectPoints(model.object_points, rvec, tvec, K, D)
        if math.isfinite(rms):
            solutions.append((rms, maximum, rvec, tvec, projected.reshape(-1, 2)))
    return solutions


def detect_order_and_pose(
    candidates: list[BrightCandidate],
    model: TargetModel,
    K: np.ndarray,
    D: np.ndarray,
    max_geometry_error: float,
    max_reprojection_error: float,
    min_match_margin: float = 0.10,
) -> PoseEstimate | None:
    return _detect_order_and_pose_with_status(
        candidates,
        model,
        K,
        D,
        max_geometry_error,
        max_reprojection_error,
        min_match_margin,
    )[0]


def _detect_order_and_pose_with_status(
    candidates: list[BrightCandidate],
    model: TargetModel,
    K: np.ndarray,
    D: np.ndarray,
    max_geometry_error: float,
    max_reprojection_error: float,
    min_match_margin: float,
) -> tuple[PoseEstimate | None, str]:
    matches = candidate_geometry_matches(candidates, model, max_geometry_error)
    if not matches:
        return None, "ambiguous_correspondence"
    ranked: list[PoseEstimate] = []
    pnp_succeeded = False
    point_ids = np.arange(model.point_count, dtype=np.int32)
    for geometry_error, ordered in matches:
        best_for_order = None
        solutions = estimate_pose(model, ordered, K, D)
        pnp_succeeded = pnp_succeeded or bool(solutions)
        for rms, maximum, rvec, tvec, projected in solutions:
            if rms > max_reprojection_error:
                continue
            score = rms + 0.15 * geometry_error
            item = PoseEstimate(
                score=score,
                rms=rms,
                maximum=maximum,
                geometry_error=geometry_error,
                image_points=ordered,
                point_ids=point_ids,
                rvec=rvec,
                tvec=tvec,
                projected=projected,
            )
            if best_for_order is None or item.score < best_for_order.score:
                best_for_order = item
        if best_for_order is not None:
            ranked.append(best_for_order)
    if not ranked:
        return None, "high_reprojection_error" if pnp_succeeded else "pnp_failed"
    ranked.sort(key=lambda item: item.score)
    best = ranked[0]
    if len(ranked) >= 2 and ranked[1].score - best.score < min_match_margin:
        return None, "ambiguous_correspondence"
    return best, "detected"


@functools.lru_cache(maxsize=32)
def assignment_permutations(candidate_count: int, assignment_size: int) -> np.ndarray:
    return np.asarray(
        list(itertools.permutations(range(candidate_count), assignment_size)),
        dtype=np.int16,
    )


def association_hypotheses(
    distances: np.ndarray,
    matched_count: int,
    tracking_gate: float,
) -> list[tuple[float, np.ndarray, np.ndarray]]:
    target_count, candidate_count = distances.shape
    if matched_count > target_count or matched_count > candidate_count:
        return []
    assignments = assignment_permutations(candidate_count, matched_count)
    hypotheses = []
    for target_ids_tuple in itertools.combinations(range(target_count), matched_count):
        target_ids = np.asarray(target_ids_tuple, dtype=np.int32)
        assignment_distances = distances[target_ids[None, :], assignments]
        valid = np.max(assignment_distances, axis=1) <= tracking_gate
        if not np.any(valid):
            continue
        costs = np.where(valid, np.sum(assignment_distances**2, axis=1), np.inf)
        best_index = int(np.argmin(costs))
        hypotheses.append(
            (
                float(costs[best_index]),
                target_ids,
                assignments[best_index].astype(np.int32),
            )
        )
    hypotheses.sort(key=lambda item: item[0])
    return hypotheses


def rotation_distance_deg(first_rvec: np.ndarray, second_rvec: np.ndarray) -> float:
    first_R, _ = cv2.Rodrigues(first_rvec)
    second_R, _ = cv2.Rodrigues(second_rvec)
    relative = first_R.T @ second_R
    cosine = float(np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def filter_pose_se3(
    previous_rvec: np.ndarray,
    previous_tvec: np.ndarray,
    measured_rvec: np.ndarray,
    measured_tvec: np.ndarray,
    rotation_alpha: float,
    translation_alpha: float,
) -> tuple[np.ndarray, np.ndarray]:
    rotation_alpha = float(np.clip(rotation_alpha, 0.0, 1.0))
    translation_alpha = float(np.clip(translation_alpha, 0.0, 1.0))
    previous_R, _ = cv2.Rodrigues(previous_rvec)
    measured_R, _ = cv2.Rodrigues(measured_rvec)
    relative_R = previous_R.T @ measured_R
    relative_rvec, _ = cv2.Rodrigues(relative_R)
    filtered_R = previous_R @ cv2.Rodrigues(relative_rvec * rotation_alpha)[0]
    filtered_rvec, _ = cv2.Rodrigues(filtered_R)
    filtered_tvec = previous_tvec + translation_alpha * (measured_tvec - previous_tvec)
    return filtered_rvec, filtered_tvec


def replace_pose_transform(
    pose: PoseEstimate,
    model: TargetModel,
    rvec: np.ndarray,
    tvec: np.ndarray,
    K: np.ndarray,
    D: np.ndarray,
) -> PoseEstimate:
    visible_points = pose.image_points[pose.point_ids]
    rms, maximum, _ = reprojection_errors(
        model.object_points[pose.point_ids],
        visible_points,
        rvec,
        tvec,
        K,
        D,
    )
    projected, _ = cv2.projectPoints(model.object_points, rvec, tvec, K, D)
    return PoseEstimate(
        score=pose.score,
        rms=rms,
        maximum=maximum,
        geometry_error=pose.geometry_error,
        image_points=pose.image_points,
        point_ids=pose.point_ids,
        rvec=rvec,
        tvec=tvec,
        projected=projected.reshape(-1, 2),
    )


def track_from_previous_pose(
    candidates: list[BrightCandidate],
    model: TargetModel,
    previous_rvec: np.ndarray,
    previous_tvec: np.ndarray,
    K: np.ndarray,
    D: np.ndarray,
    tracking_gate: float,
    max_reprojection_error: float,
    max_translation_jump: float = 0.120,
    max_rotation_jump: float = 45.0,
) -> PoseEstimate | None:
    minimum_points = model.point_count - 1 if model.point_count == 7 else model.point_count
    if len(candidates) < minimum_points:
        return None
    predicted, _ = cv2.projectPoints(model.object_points, previous_rvec, previous_tvec, K, D)
    predicted = predicted.reshape(-1, 2)
    candidate_points = np.asarray([candidate.center for candidate in candidates], dtype=np.float64)
    distances = np.linalg.norm(predicted[:, None, :] - candidate_points[None, :, :], axis=2)
    matched_counts = [model.point_count]
    if model.point_count == 7:
        matched_counts.append(6)

    best: PoseEstimate | None = None
    for matched_count in matched_counts:
        hypotheses = association_hypotheses(distances, matched_count, tracking_gate)
        for _, point_ids, candidate_ids in hypotheses[:12]:
            matched_points = candidate_points[candidate_ids].astype(np.float64)
            ordered = np.full((model.point_count, 2), np.nan, dtype=np.float64)
            ordered[point_ids] = matched_points
            for rms, maximum, rvec, tvec, projected in estimate_pose(
                model,
                matched_points,
                K,
                D,
                point_ids,
            ):
                if rms > max_reprojection_error:
                    continue
                translation_jump = float(np.linalg.norm(tvec.reshape(3) - previous_tvec.reshape(3)))
                rotation_jump = rotation_distance_deg(previous_rvec, rvec)
                if translation_jump > max_translation_jump or rotation_jump > max_rotation_jump:
                    continue
                missing_penalty = 0.5 * (model.point_count - matched_count)
                score = rms + 8.0 * translation_jump + 0.015 * rotation_jump + missing_penalty
                item = PoseEstimate(
                    score=score,
                    rms=rms,
                    maximum=maximum,
                    geometry_error=0.0,
                    image_points=ordered,
                    point_ids=point_ids,
                    rvec=rvec,
                    tvec=tvec,
                    projected=projected,
                )
                if best is None or item.score < best.score:
                    best = item
    return best


def rotation_to_euler_deg(rvec: np.ndarray) -> tuple[float, float, float]:
    R, _ = cv2.Rodrigues(rvec)
    sy = math.hypot(float(R[0, 0]), float(R[1, 0]))
    singular = sy < 1e-6
    if not singular:
        roll = math.atan2(float(R[2, 1]), float(R[2, 2]))
        pitch = math.atan2(float(-R[2, 0]), sy)
        yaw = math.atan2(float(R[1, 0]), float(R[0, 0]))
    else:
        roll = math.atan2(float(-R[1, 2]), float(R[1, 1]))
        pitch = math.atan2(float(-R[2, 0]), sy)
        yaw = 0.0
    return tuple(math.degrees(value) for value in (roll, pitch, yaw))


class IRTargetPoseEstimator:
    """Stateless single-image pose estimator for one known IR target."""

    def __init__(
        self,
        intrinsics: CameraIntrinsics,
        wavelength_nm: int,
        target_id: str,
        *,
        threshold: int | None = None,
        min_core_area: int | None = None,
        max_core_area: int | None = None,
        max_candidates: int | None = None,
        max_geometry_error: float | None = None,
        max_reprojection_error: float | None = None,
        T_device_camera: np.ndarray | None = None,
    ) -> None:
        if not isinstance(intrinsics, CameraIntrinsics):
            raise ValueError("intrinsics 必须是 CameraIntrinsics")
        if (
            isinstance(wavelength_nm, (bool, np.bool_))
            or not isinstance(wavelength_nm, (int, np.integer))
            or int(wavelength_nm) not in (850, 940)
        ):
            raise ValueError("wavelength_nm 只允许 850 或 940")
        if not isinstance(target_id, str):
            raise ValueError("target_id 必须是完整型号字符串")
        normalized_target_id = target_id.strip().upper()
        if normalized_target_id not in TARGET_IDS:
            supported = ", ".join(TARGET_IDS)
            raise ValueError(f"target_id 必须是完整型号；支持: {supported}")
        target_wavelength = int(normalized_target_id.split("-")[-2])
        if target_wavelength != int(wavelength_nm):
            raise ValueError(
                f"波段与型号不一致: wavelength_nm={wavelength_nm}, target_id={normalized_target_id}"
            )

        overrides = {
            name: value
            for name, value in {
                "threshold": threshold,
                "min_core_area": min_core_area,
                "max_core_area": max_core_area,
                "max_candidates": max_candidates,
                "max_geometry_error": max_geometry_error,
                "max_reprojection_error": max_reprojection_error,
            }.items()
            if value is not None
        }
        parameters = replace(TARGET_DEFAULT_PARAMETERS[normalized_target_id], **overrides)
        selection = resolve_target(normalized_target_id)
        if parameters.max_candidates < selection.model.point_count:
            raise ValueError(
                f"max_candidates 不能小于标靶点数 {selection.model.point_count}"
            )
        self.intrinsics = intrinsics
        self.wavelength_nm = int(wavelength_nm)
        self.target_id = normalized_target_id
        self.selection = selection
        self.support_status = TARGET_SUPPORT_STATUS[normalized_target_id]
        self.parameters = parameters
        self.T_device_camera = (
            None
            if T_device_camera is None
            else _validate_rigid_transform(T_device_camera, "T_device_camera")
        )

    def estimate(self, image: np.ndarray) -> PoseResult:
        gray = self._validate_image(image)
        model = self.selection.model
        parameters = self.parameters
        candidates, _, _ = detect_bright_candidates(
            gray,
            expected_point_count=model.point_count,
            threshold=parameters.threshold,
            blur_sigma=parameters.blur_sigma,
            min_peak_distance=parameters.min_peak_distance,
            core_radius=parameters.core_radius,
            core_drop=parameters.core_drop,
            min_core_area=parameters.min_core_area,
            max_core_area=parameters.max_core_area,
            max_candidates=parameters.max_candidates,
            allow_peak_fallback=parameters.allow_peak_fallback,
        )
        if len(candidates) < model.point_count:
            return self._failure("insufficient_candidates", len(candidates))

        pose, status = _detect_order_and_pose_with_status(
            candidates,
            model,
            self.intrinsics.K,
            self.intrinsics.D,
            parameters.max_geometry_error,
            parameters.max_reprojection_error,
            parameters.min_match_margin,
        )
        if pose is None:
            return self._failure(status, len(candidates))

        if parameters.max_area_cv is not None:
            matched = [
                min(candidates, key=lambda item: np.linalg.norm(item.center - point))
                for point in pose.image_points[pose.point_ids]
            ]
            areas = np.asarray([candidate.core_area for candidate in matched], dtype=np.float64)
            if float(areas.std() / areas.mean()) > parameters.max_area_cv:
                return self._failure("ambiguous_correspondence", len(candidates))

        rotation, _ = cv2.Rodrigues(pose.rvec)
        transform = np.eye(4, dtype=np.float64)
        transform[:3, :3] = rotation
        transform[:3, 3] = pose.tvec.reshape(3)
        device_transform = (
            None if self.T_device_camera is None else self.T_device_camera @ transform
        )
        return PoseResult(
            ok=True,
            status="detected",
            target_id=self.target_id,
            wavelength_nm=self.wavelength_nm,
            support_status=self.support_status,
            candidate_count=len(candidates),
            matched_count=pose.matched_count,
            T_camera_target=transform,
            T_device_target=device_transform,
            rvec=pose.rvec.copy(),
            tvec_m=pose.tvec.copy(),
            image_points=pose.image_points[pose.point_ids].copy(),
            point_ids=pose.point_ids.copy(),
            reprojection_rms_px=pose.rms,
            reprojection_max_px=pose.maximum,
            geometry_error_px=pose.geometry_error,
        )

    def _validate_image(self, image: np.ndarray) -> np.ndarray:
        if not isinstance(image, np.ndarray) or image.size == 0:
            raise ValueError("image 必须是非空 NumPy 数组")
        if image.dtype != np.uint8:
            raise ValueError("image dtype 必须为 uint8")
        if image.ndim == 2:
            gray = image
        elif image.ndim == 3 and image.shape[2] == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            raise ValueError("image 必须是 HxW 灰度图或 HxWx3 BGR 图")
        actual_size = (gray.shape[1], gray.shape[0])
        if actual_size != self.intrinsics.image_size:
            expected = self.intrinsics.image_size
            raise ValueError(
                f"图像尺寸 {actual_size[0]}x{actual_size[1]} 与内参 {expected[0]}x{expected[1]} 不一致"
            )
        return gray

    def _failure(self, status: str, candidate_count: int) -> PoseResult:
        return PoseResult(
            ok=False,
            status=status,
            target_id=self.target_id,
            wavelength_nm=self.wavelength_nm,
            support_status=self.support_status,
            candidate_count=candidate_count,
        )
