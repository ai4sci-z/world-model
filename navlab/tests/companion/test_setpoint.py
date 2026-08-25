from __future__ import annotations

from math import hypot, isclose

from navlab.common.companion.setpoint import (
    INVALID_GBPLANNER_ABSOLUTE_TARGET,
    gbplanner_position_carrot,
    map_target_to_local_ned_carrot,
)


def test_map_target_to_local_ned_carrot_swaps_enu_axes_and_limits_lead() -> None:
    target = map_target_to_local_ned_carrot(
        current_north_m=1.0,
        current_east_m=2.0,
        target_map_x_m=4.0,
        target_map_y_m=5.0,
        max_lead_m=0.10,
    )

    assert target is not None
    north, east = target
    assert isclose(hypot(north - 1.0, east - 2.0), 0.10)
    assert north > 1.0
    assert east > 2.0


def test_map_target_to_local_ned_carrot_uses_final_target_inside_lead() -> None:
    assert map_target_to_local_ned_carrot(
        current_north_m=1.0,
        current_east_m=2.0,
        target_map_x_m=2.04,
        target_map_y_m=1.03,
        max_lead_m=0.10,
    ) == (1.03, 2.04)


def test_map_target_to_local_ned_carrot_rejects_invalid_input() -> None:
    assert map_target_to_local_ned_carrot(
        current_north_m=0.0,
        current_east_m=0.0,
        target_map_x_m=float("nan"),
        target_map_y_m=1.0,
        max_lead_m=0.10,
    ) is None


def test_active_gbplanner_intent_requires_complete_absolute_target() -> None:
    target, error = gbplanner_position_carrot(
        {"strategy": "gbplanner", "ok": True, "target_map_x_m": 1.0},
        current_north_m=0.0,
        current_east_m=0.0,
    )

    assert target is None
    assert error == INVALID_GBPLANNER_ABSOLUTE_TARGET


def test_inactive_gbplanner_hold_does_not_require_absolute_target() -> None:
    assert gbplanner_position_carrot(
        {"strategy": "gbplanner", "ok": False},
        current_north_m=0.0,
        current_east_m=0.0,
    ) == (None, None)


def test_non_gbplanner_strategy_keeps_existing_setpoint_path() -> None:
    assert gbplanner_position_carrot(
        {"strategy": "frontier_lite", "ok": True},
        current_north_m=0.0,
        current_east_m=0.0,
    ) == (None, None)


def test_gbplanner_position_carrot_clamps_and_sanitizes_lead() -> None:
    payload = {
        "strategy": "gbplanner",
        "ok": True,
        "target_map_x_m": 1.0,
        "target_map_y_m": 0.0,
        "max_position_lead_m": float("nan"),
    }

    target, error = gbplanner_position_carrot(
        payload,
        current_north_m=0.0,
        current_east_m=0.0,
    )

    assert error is None
    assert target == (0.0, 0.10)

    payload["max_position_lead_m"] = 999.0
    target, error = gbplanner_position_carrot(
        payload,
        current_north_m=0.0,
        current_east_m=0.0,
    )
    assert error is None
    assert target == (0.0, 0.25)
