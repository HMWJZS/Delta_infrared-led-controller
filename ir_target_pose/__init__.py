"""Single-image infrared LED target pose estimation."""

from .estimator import (
    CameraIntrinsics,
    DetectionParameters,
    IRTargetPoseEstimator,
    PoseResult,
    SUPPORTED_TARGET_IDS,
    TARGET_SUPPORT_STATUS,
)

__version__ = "0.2.0"

__all__ = [
    "CameraIntrinsics",
    "DetectionParameters",
    "IRTargetPoseEstimator",
    "PoseResult",
    "SUPPORTED_TARGET_IDS",
    "TARGET_SUPPORT_STATUS",
    "__version__",
]
