from __future__ import annotations

import math
from collections.abc import Mapping
from typing import Any


INVALID_GBPLANNER_ABSOLUTE_TARGET = "invalid_gbplanner_absolute_target"


def map_target_to_local_ned_carrot(
    *,
    current_north_m: float,
    current_east_m: float,
    target_map_x_m: float,
    target_map_y_m: float,
    max_lead_m: float,
) -> tuple[float, float] | None:
    """Map an ENU waypoint to a bounded LOCAL_NED position target."""
    try:
        north, east, map_x, map_y, lead = (
            float(value)
            for value in (
                current_north_m,
                current_east_m,
                target_map_x_m,
                target_map_y_m,
                max_lead_m,
            )
        )
    except (TypeError, ValueError):
        return None
    values = (north, east, map_x, map_y, lead)
    if not all(math.isfinite(value) for value in values) or lead <= 0.0:
        return None

    target_north = map_y
    target_east = map_x
    delta_north = target_north - north
    delta_east = target_east - east
    distance = math.hypot(delta_north, delta_east)
    if distance <= lead:
        return target_north, target_east
    scale = lead / distance
    return north + delta_north * scale, east + delta_east * scale


def gbplanner_position_carrot(
    payload: Mapping[str, Any],
    *,
    current_north_m: float,
    current_east_m: float,
) -> tuple[tuple[float, float] | None, str | None]:
    """Resolve an active GBPlanner intent to a bounded LOCAL_NED target."""
    if payload.get("strategy") != "gbplanner" or not bool(payload.get("ok", False)):
        return None, None

    target_map_x = payload.get("target_map_x_m")
    target_map_y = payload.get("target_map_y_m")
    if target_map_x is None or target_map_y is None:
        return None, INVALID_GBPLANNER_ABSOLUTE_TARGET

    try:
        requested_lead_m = float(payload.get("max_position_lead_m", 0.10) or 0.10)
    except (TypeError, ValueError):
        requested_lead_m = 0.10
    if not math.isfinite(requested_lead_m):
        requested_lead_m = 0.10

    target = map_target_to_local_ned_carrot(
        current_north_m=current_north_m,
        current_east_m=current_east_m,
        target_map_x_m=target_map_x,
        target_map_y_m=target_map_y,
        max_lead_m=min(0.25, max(0.05, requested_lead_m)),
    )
    if target is None:
        return None, INVALID_GBPLANNER_ABSOLUTE_TARGET
    return target, None
