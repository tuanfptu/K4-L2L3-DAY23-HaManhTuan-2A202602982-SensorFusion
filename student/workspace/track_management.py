"""Track initialization, scoring, and deletion helpers.

Part H supplies lidar-driven existence decisions (docs/HUONG_DAN_KY_THUAT.md §2).
Use tracking parameters for the score window, thresholds, and covariance limit.
"""

from __future__ import annotations

from typing import Any

import numpy as np
from fusion_lab.workspace_support import get_tracking_params

# vi: from fusion_lab.workspace_support import get_tracking_params
# vi: import numpy as np


def init_track_state_from_meas(meas: Any) -> dict[str, Any]:
    """Initialize track state, covariance, lifecycle state, and score from a measurement.

    Args:
        meas: Lidar measurement with ``z``, ``R``, ``sensor``.

    Returns:
        Dict with keys ``x``, ``P``, ``state``, ``score`` (matrices as ``np.matrix``).
    """
    # vi: TODO Part H — đổi meas.z sang vehicle frame; x = [pos; 0 velocity];
    # vi: P block pos từ R xoay, vel từ sigma_p44/55/66; score = 1/window; state initialized.
    params = get_tracking_params()
    sensor = meas.sensor
    z = np.asarray(meas.z, dtype=float).reshape(3, 1)
    transform = np.asarray(sensor.sens_to_veh, dtype=float)
    rotation, translation = transform[:3, :3], transform[:3, 3:4]
    position = rotation @ z + translation
    covariance_position = rotation @ np.asarray(meas.R, dtype=float) @ rotation.T
    covariance = np.zeros((params.dim_state, params.dim_state), dtype=float)
    covariance[:3, :3] = covariance_position
    covariance[3, 3] = params.sigma_p44**2
    covariance[4, 4] = params.sigma_p55**2
    covariance[5, 5] = params.sigma_p66**2
    return {
        "x": np.asmatrix(np.vstack([position, np.zeros((3, 1))])),
        "P": np.asmatrix(covariance),
        "state": "initialized",
        "score": 1.0 / params.window,
    }


def update_track_score(track: dict[str, Any], associated: bool) -> dict[str, Any]:
    """Update existence once per lidar frame; camera passes never call this helper.

    A hit adds 1/window, capped at one; an in-FOV miss subtracts 1/window.
    Confirm above confirmed_threshold, and preserve confirmed state after misses.

    Args:
        track: Dict-like track with ``score``, ``state``.
        associated: True for a lidar hit; False for a lidar miss within the lidar FOV.

    Returns:
        Updated track dict.
    """
    # vi: TODO Part H — chỉ lidar: hit +1/window (tối đa 1), miss trong FOV -1/window.
    # vi: score > confirmed_threshold → confirmed; đã confirmed không hạ trạng thái.
    # vi: Camera không gọi hàm này; track chưa confirmed với hit → tentative.
    params = get_tracking_params()
    increment = 1.0 / params.window
    if associated:
        track["score"] = min(1.0, float(track["score"]) + increment)
        if track["score"] > params.confirmed_threshold:
            track["state"] = "confirmed"
        elif track["state"] != "confirmed":
            track["state"] = "tentative"
    else:
        track["score"] = float(track["score"]) - increment
        if track["state"] != "confirmed":
            track["state"] = "tentative"
    return track


def should_delete_track(track: dict[str, Any]) -> bool:
    """Return whether a lidar lifecycle pass should remove this track.

    Delete if either horizontal variance exceeds max_P, or if a confirmed
    track has score < delete_threshold, or an unconfirmed track has score <= 0.
    Camera passes never trigger deletion.

    Args:
        track: Dict with ``score``, ``state``, ``P``.

    Returns:
        True if track should be removed.
    """
    # vi: TODO Part H — Pxx hoặc Pyy > max_P: xóa bất kể score.
    # vi: confirmed: xóa khi score < delete_threshold; chưa confirmed: score <= 0.
    # vi: Các điều kiện là OR; camera không đánh giá/xóa track.
    params = get_tracking_params()
    covariance = np.asarray(track["P"], dtype=float)
    if covariance[0, 0] > params.max_P or covariance[1, 1] > params.max_P:
        return True
    if track["state"] == "confirmed":
        return float(track["score"]) < params.delete_threshold
    return float(track["score"]) <= 0.0
