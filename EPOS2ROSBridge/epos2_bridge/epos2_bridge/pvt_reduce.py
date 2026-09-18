from __future__ import annotations

from dataclasses import dataclass
from typing import List, Sequence
import math


@dataclass
class ReducedPVTSegment:
    target_rad: float
    target_vel_rad_s: float
    duration_sec: float
    source_index: int
    source_point: object


def _pt_time_sec(pt) -> float:
    return float(pt.time_from_start.sec) + float(pt.time_from_start.nanosec) * 1e-9


def _pt_pos_rad(pt) -> float:
    return float(pt.positions[0])


def _estimate_velocities(points: Sequence[object]) -> List[float]:
    times = [_pt_time_sec(p) for p in points]
    pos = [_pt_pos_rad(p) for p in points]

    # Use provided MoveIt velocities if present and populated.
    out: List[float] = []
    use_given = True
    for p in points:
        if not hasattr(p, "velocities") or len(p.velocities) < 1:
            use_given = False
            break
    if use_given:
        for p in points:
            out.append(float(p.velocities[0]))
        return out

    # Otherwise finite-difference estimate.
    n = len(points)
    if n == 1:
        return [0.0]

    vel = [0.0] * n
    for i in range(n):
        if i == 0:
            dt = max(times[1] - times[0], 1e-6)
            vel[i] = (pos[1] - pos[0]) / dt
        elif i == n - 1:
            dt = max(times[-1] - times[-2], 1e-6)
            vel[i] = (pos[-1] - pos[-2]) / dt
        else:
            dt = max(times[i + 1] - times[i - 1], 1e-6)
            vel[i] = (pos[i + 1] - pos[i - 1]) / dt
    return vel


def reduce_joint_trajectory(
    points: Sequence[object],
    nominal_dt_sec: float = 0.04,
    max_dt_sec: float = 0.08,
    max_abs_dq_rad: float = 0.02,
    max_abs_dv_rad_s: float = 0.30,
    max_slope_change_rad_s: float = 1.5,
) -> List[ReducedPVTSegment]:
    """
    Convert dense MoveIt JointTrajectory points into a smaller list of endpoint-like
    segments that are much closer to EPOS2's intended PVT interpretation.

    Keeps more points where motion changes quickly, fewer on smooth stretches.
    """
    if not points:
        return []

    n = len(points)
    times = [_pt_time_sec(p) for p in points]
    pos = [_pt_pos_rad(p) for p in points]
    vel = _estimate_velocities(points)

    keep_idx = [0]
    last_keep = 0

    for i in range(1, n - 1):
        dt_from_keep = times[i] - times[last_keep]
        dq_from_keep = abs(pos[i] - pos[last_keep])
        dv_from_keep = abs(vel[i] - vel[last_keep])

        # Local slope change is a cheap proxy for "curvature / dynamic change"
        prev_dt = max(times[i] - times[i - 1], 1e-6)
        next_dt = max(times[i + 1] - times[i], 1e-6)
        prev_slope = (pos[i] - pos[i - 1]) / prev_dt
        next_slope = (pos[i + 1] - pos[i]) / next_dt
        slope_change = abs(next_slope - prev_slope)

        keep = False
        if dt_from_keep >= max_dt_sec:
            keep = True
        elif dq_from_keep >= max_abs_dq_rad:
            keep = True
        elif dv_from_keep >= max_abs_dv_rad_s:
            keep = True
        elif slope_change >= max_slope_change_rad_s and dt_from_keep >= nominal_dt_sec:
            keep = True

        if keep:
            keep_idx.append(i)
            last_keep = i

    if keep_idx[-1] != n - 1:
        keep_idx.append(n - 1)

    segments: List[ReducedPVTSegment] = []
    prev_idx = keep_idx[0]

    # Skip a zero-duration first point if MoveIt gave one at t=0.
    start_k = 1 if len(keep_idx) > 1 else 0

    for k in range(start_k, len(keep_idx)):
        idx = keep_idx[k]
        dt = times[idx] - times[prev_idx]

        if k == 1 and times[prev_idx] <= 1e-9:
            dt = times[idx]

        dt = max(0.02, dt)

        # Keep future streaming compatibility with EPOS2's 255 ms point-time limit.
        # If a segment is too long, split it into equal-duration endpoint-like chunks.
        if dt <= 0.255:
            segments.append(
                ReducedPVTSegment(
                    target_rad=pos[idx],
                    target_vel_rad_s=vel[idx],
                    duration_sec=dt,
                    source_index=idx,
                    source_point=points[idx],
                )
            )
        else:
            n_split = max(2, math.ceil(dt / 0.255))
            for s in range(1, n_split + 1):
                frac = s / n_split
                interp_pos = pos[prev_idx] + frac * (pos[idx] - pos[prev_idx])
                interp_vel = vel[prev_idx] + frac * (vel[idx] - vel[prev_idx])
                segments.append(
                    ReducedPVTSegment(
                        target_rad=interp_pos,
                        target_vel_rad_s=interp_vel,
                        duration_sec=dt / n_split,
                        source_index=idx,
                        source_point=points[idx],
                    )
                )

        prev_idx = idx

    return segments
