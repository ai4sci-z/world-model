from __future__ import annotations

import math
import time
from types import SimpleNamespace

from navlab.common.pose import quaternion_from_yaw
from navlab.real.companion.nodes import external_nav as external_nav_module
from navlab.real.companion.nodes.external_nav import (
    MavlinkExternalNavSender,
    _yaw_from_ros_quat_enu,
    rate_limit_xy,
    rate_limit_yaw,
    TimestampEpochGate,
    ros_enu_position_to_mavlink_local_frd,
    ros_enu_yaw_to_mavlink_local_frd,
)

# OPEN-2 (R003-A14): these node-level tests must not depend on pymavlink
# being installed. Without it the module falls back to mavlink=None and the
# send path crashes on constant lookup — an environment artifact, not the
# behavior under test. Inject the MAVLink spec constants
# (MAV_FRAME_LOCAL_FRD=20, MAV_FRAME_BODY_FRD=12, MAV_ESTIMATOR_TYPE_VIO=3)
# only in that case; with pymavlink present this is a no-op. Deliberately not
# a pytest fixture: the companion container has pymavlink but no pytest, and
# the tests must stay runnable there too.
if external_nav_module.mavlink is None:
    external_nav_module.mavlink = SimpleNamespace(
        MAV_FRAME_LOCAL_FRD=20,
        MAV_FRAME_BODY_FRD=12,
        MAV_ESTIMATOR_TYPE_VIO=3,
    )


def _pose_with_yaw(yaw_rad: float) -> SimpleNamespace:
    qx, qy, qz, qw = quaternion_from_yaw(yaw_rad)
    return SimpleNamespace(orientation=SimpleNamespace(x=qx, y=qy, z=qz, w=qw))


def _yaw_from_frd_quat(q: list[float]) -> float:
    w, x, y, z = q
    return math.atan2(2.0 * ((w * z) + (x * y)), 1.0 - (2.0 * ((y * y) + (z * z))))


def _roll_pitch_from_frd_quat(q: list[float]) -> tuple[float, float]:
    w, x, y, z = q
    roll = math.atan2(2.0 * ((w * x) + (y * z)), 1.0 - (2.0 * ((x * x) + (y * y))))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * ((w * y) - (z * x)))))
    return roll, pitch


def _sender_without_ros(max_yaw_rate_radps: float = 0.0) -> MavlinkExternalNavSender:
    sender = MavlinkExternalNavSender.__new__(MavlinkExternalNavSender)
    sender._use_fcu_roll_pitch = True
    sender._align_yaw_to_fcu = True
    sender._fcu_roll_rad = 0.0
    sender._fcu_pitch_rad = 0.0
    sender._fcu_yaw_rad = -0.5
    sender._fcu_rollspeed_radps = 0.0
    sender._fcu_pitchspeed_radps = 0.0
    sender._last_fcu_attitude_monotonic = time.monotonic()
    sender._yaw_alignment_offset_rad = None
    sender._max_yaw_rate_radps = max_yaw_rate_radps
    sender._last_sent_yaw_rad = None
    sender._last_sent_yaw_monotonic = 0.0
    return sender


def test_odometry_quaternion_aligns_initial_slam_yaw_to_fcu_attitude() -> None:
    sender = _sender_without_ros()

    first = sender._odometry_quaternion(_pose_with_yaw(0.1))
    second = sender._odometry_quaternion(_pose_with_yaw(0.2))

    assert math.isclose(_yaw_from_frd_quat(first), -0.5, abs_tol=1e-6)
    assert sender._yaw_alignment_offset_rad is not None
    assert math.isclose(_yaw_from_frd_quat(second), -0.6, abs_tol=1e-6)


def test_odometry_quaternion_can_send_raw_slam_yaw_without_alignment() -> None:
    sender = _sender_without_ros()
    sender._align_yaw_to_fcu = False

    q = sender._odometry_quaternion(_pose_with_yaw(0.1))

    assert math.isclose(_yaw_from_frd_quat(q), (math.pi * 0.5) - 0.1, abs_tol=1e-6)
    assert sender._yaw_alignment_offset_rad is None
    assert math.isclose(_yaw_from_ros_quat_enu(_pose_with_yaw(0.1).orientation), 0.1, abs_tol=1e-6)
    assert math.isclose(ros_enu_yaw_to_mavlink_local_frd(0.1), (math.pi * 0.5) - 0.1, abs_tol=1e-6)


def test_ros_enu_position_maps_to_mavlink_local_frd_axes() -> None:
    # Standard ENU->NED: north=y, east=x, down=-z. A negated east axis is a
    # reflection (left-handed feed) and destabilizes EK3 (truth-fed hover
    # dataflash replay, 2026-07-16).
    assert ros_enu_position_to_mavlink_local_frd(x_enu_m=2.0, y_enu_m=3.0, z_enu_m=0.5) == (
        3.0,
        2.0,
        -0.5,
    )


def test_ros_enu_position_keeps_frame_right_handed() -> None:
    assert ros_enu_position_to_mavlink_local_frd(x_enu_m=-0.5, y_enu_m=0.35, z_enu_m=0.5) == (
        0.35,
        -0.5,
        -0.5,
    )


def _drive(gate: TimestampEpochGate, stamps: list[int]) -> list[str]:
    return [gate.classify(t) for t in stamps]


def test_epoch_gate_fresh_duplicate_and_small_out_of_order() -> None:
    gate = TimestampEpochGate()
    assert _drive(gate, [1_000_000, 1_050_000, 1_050_000, 1_049_000, 1_100_000]) == [
        "send",
        "send",
        "drop_duplicate",  # same-epoch repeat
        "drop_duplicate",  # small out-of-order (< back threshold)
        "send",
    ]


def test_epoch_gate_single_old_delayed_packet_never_resets_or_sends() -> None:
    # R003-F12 counterexample: a straggler delayed by >1s must be dropped —
    # it is NOT a new clock epoch, and it must never be sent as a pose.
    gate = TimestampEpochGate()
    assert _drive(gate, [10_000_000, 10_050_000, 3_000_000, 10_100_000, 10_150_000]) == [
        "send",
        "send",
        "drop_stale",  # >1s-old packet: dropped, no reset
        "send",  # current epoch resumes untouched
        "send",
    ]


def test_epoch_gate_two_stragglers_still_do_not_reset() -> None:
    # Two isolated old packets interleaved with live samples: the live
    # samples keep clearing the candidate accumulator.
    gate = TimestampEpochGate()
    assert _drive(gate, [10_000_000, 3_000_000, 10_050_000, 3_100_000, 10_100_000]) == [
        "send",
        "drop_stale",
        "send",
        "drop_stale",
        "send",
    ]


def test_epoch_gate_confirms_real_clock_reset_after_consecutive_samples() -> None:
    # Sim relaunch / rosbag loop: the new epoch streams monotonically at the
    # feed rate; the third consecutive candidate confirms the reset.
    gate = TimestampEpochGate()
    assert _drive(gate, [60_000_000, 60_050_000, 1_000_000, 1_050_000, 1_100_000, 1_150_000]) == [
        "send",
        "send",
        "drop_stale",  # candidate 1
        "drop_stale",  # candidate 2
        "reset_send",  # confirmed on candidate 3
        "send",  # new epoch is now current
    ]


def test_epoch_gate_drops_old_epoch_straggler_after_reset() -> None:
    # After a confirmed reset a late packet from the PRE-reset epoch shows up
    # as an implausible forward jump: dropped, and the live stream continues.
    gate = TimestampEpochGate()
    verdicts = _drive(gate, [60_000_000, 1_000_000, 1_050_000, 1_100_000, 59_900_000, 1_150_000])
    assert verdicts == [
        "send",
        "drop_stale",
        "drop_stale",
        "reset_send",
        "drop_stale",  # old-epoch straggler: forward jump >= threshold
        "send",
    ]


def test_epoch_gate_non_monotonic_candidates_restart_confirmation() -> None:
    # An epoch-break candidate stream that is itself not monotonic cannot
    # confirm a reset: confirmation restarts from the offending sample.
    gate = TimestampEpochGate()
    assert _drive(gate, [60_000_000, 1_100_000, 1_050_000, 1_000_000]) == [
        "send",
        "drop_stale",
        "drop_stale",  # regressed vs candidate: restart, count=1
        "drop_stale",  # restart again
    ]


def test_epoch_gate_zero_and_negative_stamps_never_latch() -> None:
    gate = TimestampEpochGate()
    assert _drive(gate, [0, 0, -5]) == ["drop_invalid", "drop_invalid", "drop_invalid"]
    # A permanently-zero stream must not poison state: first real stamp sends.
    assert gate.classify(1_000_000) == "send"
    # And zero stamps after adoption stay invalid without disturbing dedup.
    assert gate.classify(0) == "drop_invalid"
    assert gate.classify(1_050_000) == "send"


class _RecordingMav:
    def __init__(self) -> None:
        self.odometry_calls: list[tuple] = []

    def odometry_send(self, *args) -> None:
        self.odometry_calls.append(args)


class _RecordingConnection:
    def __init__(self) -> None:
        self.mav = _RecordingMav()


def _odom_with(stamp_sec: int, stamp_nsec: int, x: float = 0.0) -> object:
    stamp = SimpleNamespace(sec=stamp_sec, nanosec=stamp_nsec)
    header = SimpleNamespace(stamp=stamp, frame_id="map")
    position = SimpleNamespace(x=x, y=0.0, z=0.5)
    orientation = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
    pose = SimpleNamespace(position=position, orientation=orientation)
    linear = SimpleNamespace(x=0.0, y=0.0, z=0.0)
    angular = SimpleNamespace(x=0.0, y=0.0, z=0.0)
    twist = SimpleNamespace(linear=linear, angular=angular)
    return SimpleNamespace(
        header=header,
        child_frame_id="base_link",
        pose=SimpleNamespace(pose=pose, covariance=[0.0] * 36),
        twist=SimpleNamespace(twist=twist, covariance=[0.0] * 36),
    )


def _sender_for_send_tick() -> MavlinkExternalNavSender:
    sender = _sender_without_ros()
    sender._use_fcu_roll_pitch = False
    sender._connection = _RecordingConnection()
    sender._last_odom = None
    sender._last_heartbeat_monotonic = time.monotonic()
    sender._last_odom_rx_monotonic = 0.0
    sender._sent_count = 0
    sender._quality = 100
    sender._reset_counter = 0
    sender._max_horizontal_speed_mps = 0.0
    sender._last_sent_x = None
    sender._last_sent_y = None
    sender._last_sent_time_usec = None
    sender._last_limited_odom_x = None
    sender._last_limited_odom_y = None
    sender._last_sent_xy_monotonic = 0.0
    sender._clock_reset_count = 0
    sender._invalid_stamp_count = 0
    sender._stale_stamp_count = 0
    sender._epoch_gate = TimestampEpochGate()
    sender._drain_mavlink = lambda now: None
    sender._request_fcu_attitude_if_needed = lambda now: None
    return sender


def test_epoch_gate_node_restart_mid_stream_adopts_current_epoch() -> None:
    # WP305 node-restart counterexample: a sender restart builds a fresh gate
    # while the source keeps streaming its (large, nonzero) epoch. The first
    # live sample must be adopted immediately — a restart must not stall the
    # feed behind epoch confirmation.
    gate = TimestampEpochGate()
    assert gate.classify(3_600_000_000) == "send"
    assert gate.classify(3_600_050_000) == "send"
    # And stragglers afterwards are still dropped, never a reset: within
    # EPOCH_BREAK_BACK_US they are dedup traffic, beyond it they are
    # unconfirmed epoch-break candidates.
    assert gate.classify(3_599_600_000) == "drop_duplicate"
    assert gate.classify(3_598_000_000) == "drop_stale"


def test_epoch_gate_source_dropout_forward_jump_needs_confirmation() -> None:
    # WP305 source-lifecycle counterexample: the source dies and comes back
    # much later on the same clock (long dropout => forward jump beyond the
    # epoch-break threshold). The jump is an epoch-break candidate, not a
    # send; the stream re-latches only after consecutive confirmation.
    gate = TimestampEpochGate()
    assert gate.classify(60_000_000) == "send"
    resumed = 60_000_000 + external_nav_module.EPOCH_BREAK_FORWARD_US + 10_000_000
    verdicts = []
    for i in range(external_nav_module.EPOCH_CONFIRM_SAMPLES):
        verdicts.append(gate.classify(resumed + (i * 50_000)))
    assert verdicts[-1] == "reset_send"
    assert all(v == "drop_stale" for v in verdicts[:-1])
    # After adoption the resumed stream is normal traffic again.
    assert gate.classify(resumed + (external_nav_module.EPOCH_CONFIRM_SAMPLES * 50_000)) == "send"


def test_send_tick_drops_old_packets_and_recovers_after_confirmed_reset() -> None:
    sender = _sender_for_send_tick()

    def tick(sec: int, nsec: int = 0, x: float = 0.0) -> None:
        sender._last_odom = _odom_with(sec, nsec, x=x)
        sender._send_tick()

    tick(60, 0)
    tick(60, 50_000_000)
    assert len(sender._connection.mav.odometry_calls) == 2

    # A delayed old packet (>1s back) must not be sent and must not reset.
    tick(3, 0, x=99.0)
    assert len(sender._connection.mav.odometry_calls) == 2
    assert sender._stale_stamp_count == 1
    assert sender._clock_reset_count == 0

    # Current epoch continues.
    tick(60, 100_000_000)
    assert len(sender._connection.mav.odometry_calls) == 3

    # Real clock reset: three consecutive new-epoch samples confirm; the
    # confirming sample is sent and the limiter/dedup state restarts.
    tick(1, 0)
    tick(1, 50_000_000)
    assert len(sender._connection.mav.odometry_calls) == 3
    tick(1, 100_000_000)
    assert len(sender._connection.mav.odometry_calls) == 4
    assert sender._clock_reset_count == 1
    assert sender._last_limited_odom_x is not None  # limiter re-primed post-reset

    # New epoch flows normally afterwards.
    tick(1, 150_000_000)
    assert len(sender._connection.mav.odometry_calls) == 5


def test_send_tick_zero_stamp_stream_counts_and_recovers() -> None:
    sender = _sender_for_send_tick()
    for _ in range(3):
        sender._last_odom = _odom_with(0, 0)
        sender._send_tick()
    assert len(sender._connection.mav.odometry_calls) == 0
    assert sender._invalid_stamp_count == 3

    sender._last_odom = _odom_with(5, 0)
    sender._send_tick()
    assert len(sender._connection.mav.odometry_calls) == 1


def test_odometry_quaternion_does_not_feed_fcu_roll_pitch_back_to_external_nav() -> None:
    sender = _sender_without_ros()
    sender._fcu_roll_rad = 0.4
    sender._fcu_pitch_rad = -0.3

    q = sender._odometry_quaternion(_pose_with_yaw(0.1))
    roll, pitch = _roll_pitch_from_frd_quat(q)

    assert math.isclose(roll, 0.0, abs_tol=1e-6)
    assert math.isclose(pitch, 0.0, abs_tol=1e-6)
    assert math.isclose(_yaw_from_frd_quat(q), -0.5, abs_tol=1e-6)


def test_rate_limit_xy_keeps_slam_horizontal_spikes_from_reaching_fcu() -> None:
    x_m, y_m = rate_limit_xy(
        target_x=1.0,
        target_y=0.0,
        last_x=0.0,
        last_y=0.0,
        dt_sec=0.1,
        max_speed_mps=0.2,
    )

    assert math.isclose(x_m, 0.02)
    assert math.isclose(y_m, 0.0)


def test_rate_limit_xy_can_be_disabled() -> None:
    assert rate_limit_xy(
        target_x=1.0,
        target_y=0.5,
        last_x=0.0,
        last_y=0.0,
        dt_sec=0.1,
        max_speed_mps=0.0,
    ) == (1.0, 0.5)


def test_rate_limit_yaw_keeps_slam_yaw_spikes_from_reaching_fcu() -> None:
    yaw = rate_limit_yaw(
        target_yaw_rad=1.5,
        last_yaw_rad=0.0,
        dt_sec=0.1,
        max_yaw_rate_radps=0.2,
    )

    assert math.isclose(yaw, 0.02)


def test_odometry_quaternion_rate_limits_aligned_slam_yaw() -> None:
    sender = _sender_without_ros(max_yaw_rate_radps=0.1)
    first = sender._odometry_quaternion(_pose_with_yaw(0.1), now_monotonic=10.0)
    second = sender._odometry_quaternion(_pose_with_yaw(1.6), now_monotonic=10.5)

    assert math.isclose(_yaw_from_frd_quat(first), -0.5, abs_tol=1e-6)
    assert math.isclose(_yaw_from_frd_quat(second), -0.55, abs_tol=1e-6)


def test_roll_pitch_speeds_are_zero_when_fcu_roll_pitch_is_used() -> None:
    sender = _sender_without_ros()
    sender._fcu_rollspeed_radps = 0.12
    sender._fcu_pitchspeed_radps = -0.34
    twist = SimpleNamespace(angular=SimpleNamespace(x=9.5, y=6.1))

    rollspeed, pitchspeed = sender._roll_pitch_speeds(twist, now_monotonic=time.monotonic())

    assert math.isclose(rollspeed, 0.0)
    assert math.isclose(pitchspeed, 0.0)


def test_roll_pitch_speeds_fall_back_to_odom_when_fcu_attitude_is_stale() -> None:
    sender = _sender_without_ros()
    sender._last_fcu_attitude_monotonic = 1.0
    twist = SimpleNamespace(angular=SimpleNamespace(x=9.5, y=6.1))

    rollspeed, pitchspeed = sender._roll_pitch_speeds(twist, now_monotonic=3.0)

    assert math.isclose(rollspeed, 9.5)
    assert math.isclose(pitchspeed, -6.1)
