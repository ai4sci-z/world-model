from __future__ import annotations

import argparse
import json
import math
import os
import time
from collections.abc import Sequence

os.environ.setdefault("MAVLINK20", "1")

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from pymavlink import mavutil
    from pymavlink.dialects.v20 import ardupilotmega as mavlink
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from std_msgs.msg import String
except ModuleNotFoundError:
    rclpy = None
    PoseStamped = object
    Odometry = object
    mavutil = None
    mavlink = None
    ExternalShutdownException = KeyboardInterrupt
    Node = object
    String = object

from navlab.real.companion.nodes.pose_mirror import NedPoseSample, build_pose_stamped_fields, ned_to_gazebo_pose

MAVLINK_TIME_SOURCE = "odometry_header_stamp_us"
MAVLINK_POSITION_FRAME = "MAV_FRAME_LOCAL_FRD"
MAVLINK_VELOCITY_FRAME = "MAV_FRAME_BODY_FRD"
MAVLINK_ESTIMATOR_TYPE = "MAV_ESTIMATOR_TYPE_VIO"
ROS_ODOM_SEMANTICS = "ROS ENU position with FLU body twist"
ROLL_PITCH_SOURCES = ("fcu", "level", "odom")
# OPEN-1 流请求风暴门控:两流均新鲜(墙钟)则不重发 SET_MESSAGE_INTERVAL;超此阈值视为陈旧可重发
STREAM_REREQUEST_STALE_SEC = 5.0


def _upper_triangular_covariance(covariance: Sequence[float]) -> list[float]:
    if len(covariance) != 36:
        return [0.0] * 21

    values: list[float] = []
    for row in range(6):
        for col in range(row, 6):
            value = covariance[(row * 6) + col]
            values.append(0.0 if math.isnan(value) else float(value))
    return values


def _ros_quat_to_frd(q: object) -> list[float]:
    # ROS odom is treated as ENU/FLU. MAVLink output is LOCAL_FRD/BODY_FRD.
    return [float(q.w), float(q.x), -float(q.y), -float(q.z)]


def _yaw_from_ros_quat_enu(q: object) -> float:
    x = float(q.x)
    y = float(q.y)
    z = float(q.z)
    w = float(q.w)
    return math.atan2(2.0 * ((w * z) + (x * y)), 1.0 - (2.0 * ((y * y) + (z * z))))


def ros_enu_position_to_mavlink_local_frd(
    *, x_enu_m: float, y_enu_m: float, z_enu_m: float
) -> tuple[float, float, float]:
    # ArduPilot's ODOMETRY handler requires MAV_FRAME_LOCAL_FRD. The odom map
    # contract is standard ENU (x=east, y=north): dataflash VISP-vs-SIM2 replay
    # of truth-fed hover runs fits VISP.PN=+truth_N with the yaw path already
    # matching the standard ENU formula. Negating x here mirrors the east axis
    # (det=-1, left-handed feed) — irreconcilable with the IMU for any source
    # frame — and the EKF innovation feedback diverges into a flip.
    return y_enu_m, x_enu_m, -z_enu_m


def ros_enu_yaw_to_mavlink_local_frd(yaw_enu_rad: float) -> float:
    return normalize_angle_rad((math.pi * 0.5) - yaw_enu_rad)


def _quat_from_roll_pitch_yaw_frd(*, roll_rad: float, pitch_rad: float, yaw_rad: float) -> list[float]:
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    return [
        (cr * cp * cy) + (sr * sp * sy),
        (sr * cp * cy) - (cr * sp * sy),
        (cr * sp * cy) + (sr * cp * sy),
        (cr * cp * sy) - (sr * sp * cy),
    ]


def _send_message_interval(connection, target_system: int, target_component: int, message_id: int, hz: float) -> None:
    connection.mav.command_long_send(
        target_system,
        target_component,
        mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        message_id,
        int(1_000_000.0 / hz),
        0,
        0,
        0,
        0,
        0,
    )


def rate_limit_xy(
    *,
    target_x: float,
    target_y: float,
    last_x: float | None,
    last_y: float | None,
    dt_sec: float,
    max_speed_mps: float,
) -> tuple[float, float]:
    if last_x is None or last_y is None or dt_sec <= 0.0 or max_speed_mps <= 0.0:
        return target_x, target_y
    dx = target_x - last_x
    dy = target_y - last_y
    distance = math.hypot(dx, dy)
    max_distance = max_speed_mps * dt_sec
    if distance <= max_distance or distance <= 0.0:
        return target_x, target_y
    scale = max_distance / distance
    return last_x + (dx * scale), last_y + (dy * scale)


EPOCH_BREAK_BACK_US = 1_000_000
EPOCH_BREAK_FORWARD_US = 10_000_000
EPOCH_CONFIRM_SAMPLES = 3


class TimestampEpochGate:
    """Timestamp-epoch contract for the external-nav feed.

    Contract (R003-F12): a measurement-clock epoch change (sim relaunch,
    rosbag loop) may only be adopted after EPOCH_CONFIRM_SAMPLES consecutive,
    mutually monotonic samples from the new epoch. A single regressed stamp
    — however far back — is a stale/delayed packet and is DROPPED, never
    sent and never treated as a reset. Symmetrically, a forward jump beyond
    EPOCH_BREAK_FORWARD_US (e.g. a straggler from the pre-reset epoch
    arriving after a reset) is an epoch-break candidate, not a send.
    Verdicts:
      "send"        -- in-epoch fresh sample; caller sends normally
      "reset_send"  -- epoch change confirmed on this sample; caller must
                       atomically clear dedup/slew state, then send
      "drop_duplicate" -- same stamp or small (<EPOCH_BREAK_BACK_US)
                       regression: no new measurement
      "drop_stale"  -- epoch-break candidate not yet confirmed; dropped
      "drop_invalid" -- non-positive stamp; never latches any state
    Boundary: loops shorter than EPOCH_BREAK_BACK_US are indistinguishable
    from out-of-order packets and stay deduped by design.
    """

    def __init__(
        self,
        *,
        back_threshold_us: int = EPOCH_BREAK_BACK_US,
        forward_threshold_us: int = EPOCH_BREAK_FORWARD_US,
        confirm_samples: int = EPOCH_CONFIRM_SAMPLES,
    ) -> None:
        self._back_threshold_us = back_threshold_us
        self._forward_threshold_us = forward_threshold_us
        self._confirm_samples = confirm_samples
        self._adopted_usec: int | None = None
        self._candidate_usec: int | None = None
        self._candidate_count = 0

    def _clear_candidates(self) -> None:
        self._candidate_usec = None
        self._candidate_count = 0

    def _track_candidate(self, time_usec: int) -> str:
        if self._candidate_usec is not None and 0 < time_usec - self._candidate_usec < self._forward_threshold_us:
            self._candidate_count += 1
        else:
            # First candidate, or a candidate stream that is itself not
            # monotonic/plausible: restart confirmation from this sample.
            self._candidate_count = 1
        self._candidate_usec = time_usec
        if self._candidate_count >= self._confirm_samples:
            self._adopted_usec = time_usec
            self._clear_candidates()
            return "reset_send"
        return "drop_stale"

    def classify(self, time_usec: int) -> str:
        if time_usec <= 0:
            return "drop_invalid"
        if self._adopted_usec is None:
            self._adopted_usec = time_usec
            self._clear_candidates()
            return "send"
        delta_usec = time_usec - self._adopted_usec
        if 0 < delta_usec < self._forward_threshold_us:
            self._adopted_usec = time_usec
            self._clear_candidates()
            return "send"
        if -self._back_threshold_us < delta_usec <= 0:
            # The current epoch is still alive: any pending epoch-break
            # candidates were stragglers, not a new clock.
            self._clear_candidates()
            return "drop_duplicate"
        return self._track_candidate(time_usec)


def normalize_angle_rad(angle_rad: float) -> float:
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def shortest_angle_delta_rad(target_rad: float, current_rad: float) -> float:
    return normalize_angle_rad(target_rad - current_rad)


def rate_limit_yaw(
    *,
    target_yaw_rad: float,
    last_yaw_rad: float | None,
    dt_sec: float,
    max_yaw_rate_radps: float,
) -> float:
    if last_yaw_rad is None or dt_sec <= 0.0 or max_yaw_rate_radps <= 0.0:
        return normalize_angle_rad(target_yaw_rad)
    max_delta = max_yaw_rate_radps * dt_sec
    delta = shortest_angle_delta_rad(target_yaw_rad, last_yaw_rad)
    if abs(delta) <= max_delta:
        return normalize_angle_rad(target_yaw_rad)
    return normalize_angle_rad(last_yaw_rad + math.copysign(max_delta, delta))


def _odometry_mapping_status(
    *,
    input_topic: str,
    quality: int,
    reset_counter: int,
    rate_hz: float,
    source_system: int,
    roll_pitch_source: str,
    align_yaw_to_fcu: bool,
    use_fcu_yaw: bool,
) -> dict[str, object]:
    return {
        "input": {
            "topic": input_topic,
            "message": "nav_msgs/msg/Odometry",
            "semantics": ROS_ODOM_SEMANTICS,
        },
        "output": {
            "message": "MAVLink v2 ODOMETRY",
            "position_frame": MAVLINK_POSITION_FRAME,
            "velocity_frame": MAVLINK_VELOCITY_FRAME,
            "estimator_type": MAVLINK_ESTIMATOR_TYPE,
            "component": "MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY",
            "source_system": source_system,
            "rate_hz": rate_hz,
            "quality": quality,
            "reset_counter": reset_counter,
            "time_usec_source": MAVLINK_TIME_SOURCE,
            "roll_pitch_source": roll_pitch_source,
            "yaw_source": (
                "FCU ATTITUDE yaw"
                if use_fcu_yaw
                else ("SLAM odom yaw, initial-aligned to FCU ATTITUDE" if align_yaw_to_fcu else "SLAM odom yaw")
            ),
        },
        "field_map": {
            "time_usec": MAVLINK_TIME_SOURCE,
            "x": "odom.pose.pose.position.y",
            "y": "odom.pose.pose.position.x",
            "z": "-odom.pose.pose.position.z",
            "q": (
                "FCU ATTITUDE roll/pitch + selected yaw"
                if roll_pitch_source == "fcu"
                else (
                    "level roll/pitch + selected yaw"
                    if roll_pitch_source == "level"
                    else "[w, x, -y, -z] from odom.pose.pose.orientation"
                )
            ),
            "vx": "odom.twist.twist.linear.x",
            "vy": "-odom.twist.twist.linear.y",
            "vz": "-odom.twist.twist.linear.z",
            "rollspeed": "0.0" if roll_pitch_source != "odom" else "odom.twist.twist.angular.x",
            "pitchspeed": "0.0" if roll_pitch_source != "odom" else "-odom.twist.twist.angular.y",
            "yawspeed": "-odom.twist.twist.angular.z",
            "pose_covariance": "upper triangular odom.pose.covariance",
            "velocity_covariance": "upper triangular odom.twist.covariance",
        },
    }


class MavlinkExternalNavSender(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("mavlink_external_nav_sender")
        self._endpoint = args.endpoint
        self._rate_hz = args.rate_hz
        self._quality = args.quality
        self._reset_counter = args.reset_counter
        self._source_system = args.source_system
        self._roll_pitch_source = getattr(args, "roll_pitch_source", None) or (
            "fcu" if getattr(args, "use_fcu_roll_pitch", False) else "odom"
        )
        self._use_fcu_roll_pitch = self._roll_pitch_source == "fcu"
        self._align_yaw_to_fcu = args.align_yaw_to_fcu
        self._use_fcu_yaw = args.use_fcu_yaw
        self._odom_topic = args.odom_topic
        self._max_odom_age_ms = args.max_odom_age_ms
        self._max_local_position_age_ms = args.max_local_position_age_ms
        self._max_horizontal_speed_mps = args.max_horizontal_speed_mps
        self._max_yaw_rate_radps = args.max_yaw_rate_radps
        self._local_position_pose_topic = args.local_position_pose_topic
        self._connection = mavutil.mavlink_connection(
            self._endpoint,
            source_system=self._source_system,
            source_component=mavlink.MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY,
            dialect="ardupilotmega",
        )
        self._last_odom: Odometry | None = None
        self._last_odom_rx_monotonic = 0.0
        self._sent_count = 0
        self._last_heartbeat_monotonic = 0.0
        self._target_system: int | None = None
        self._target_component: int | None = None
        self._next_stream_request_monotonic = 0.0
        self._fcu_roll_rad: float | None = None
        self._fcu_pitch_rad: float | None = None
        self._fcu_yaw_rad: float = 0.0
        self._fcu_rollspeed_radps: float | None = None
        self._fcu_pitchspeed_radps: float | None = None
        self._last_fcu_attitude_monotonic = 0.0
        self._yaw_alignment_offset_rad: float | None = None
        self._local_position_count = 0
        self._last_local_position_monotonic = 0.0
        # OPEN-1 fix: judge local_position freshness in the FCU's own sim-clock
        # domain (MAVLink time_boot_ms), not host wall-clock. Under RTF<1 / host
        # load the FCU's LOCAL_POSITION stream is throttled in wall time but its
        # time_boot_ms advances at sim rate, so a wall-clock age check spuriously
        # goes stale and flaps readiness. _fcu_boot_ms_latest tracks FCU sim-now
        # from any timestamped FCU message (ATTITUDE flows continuously).
        self._fcu_boot_ms_latest = 0
        self._last_local_position_boot_ms = 0
        self._last_sent_x: float | None = None
        self._last_sent_y: float | None = None
        self._last_sent_time_usec: int | None = None
        self._last_limited_odom_x: float | None = None
        self._last_limited_odom_y: float | None = None
        self._last_sent_xy_monotonic = 0.0
        self._last_sent_yaw_rad: float | None = None
        self._last_sent_yaw_monotonic = 0.0
        self._clock_reset_count = 0
        self._invalid_stamp_count = 0
        self._stale_stamp_count = 0
        self._epoch_gate = TimestampEpochGate()

        self.create_subscription(Odometry, args.odom_topic, self._handle_odom, 10)
        self._status_pub = self.create_publisher(String, args.status_topic, 10)
        self._local_position_pose_pub = (
            self.create_publisher(PoseStamped, args.local_position_pose_topic, 10)
            if args.local_position_pose_topic
            else None
        )
        self.create_timer(1.0 / self._rate_hz, self._send_tick)
        self.create_timer(0.5, self._publish_status)

        self.get_logger().info(
            "mavlink_external_nav_sender started "
            f"endpoint={self._endpoint} odom_topic={args.odom_topic} rate={self._rate_hz:.3f}Hz "
            f"quality={self._quality} reset_counter={self._reset_counter} "
            f"roll_pitch_source={self._roll_pitch_source} "
            f"align_yaw_to_fcu={self._align_yaw_to_fcu} "
            f"use_fcu_yaw={self._use_fcu_yaw} "
            f"local_position_pose_topic={args.local_position_pose_topic or '<disabled>'}"
        )

    def _handle_odom(self, msg: Odometry) -> None:
        self._last_odom = msg
        self._last_odom_rx_monotonic = time.monotonic()

    def _send_tick(self) -> None:
        now_monotonic = time.monotonic()
        self._drain_mavlink(now_monotonic)
        self._request_fcu_attitude_if_needed(now_monotonic)
        if now_monotonic - self._last_heartbeat_monotonic >= 1.0:
            self._connection.mav.heartbeat_send(
                mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                mavlink.MAV_STATE_ACTIVE,
            )
            self._last_heartbeat_monotonic = now_monotonic

        if self._last_odom is None:
            return

        if self._requires_fcu_attitude() and not self._fcu_attitude_fresh(now_monotonic):
            # The selected attitude contract is part of this measurement. Do
            # not silently switch to a different quaternion source in flight.
            return

        odom = self._last_odom
        pose = odom.pose.pose
        twist = odom.twist.twist
        # Timestamp the sample with the odometry header stamp, not the sender's
        # wall clock. The autopilot's EKF correlates time_usec against its own
        # clock, which under lockstep simulation is SIM time: a wall-clock stamp
        # drifts against it at the real-time-factor rate, so the EKF fuses every
        # position at a wandering effective delay and its velocity estimate
        # oscillates (dataflash-verified hover limit cycle ending in a flip).
        # The header stamp is the measurement's native time base in both worlds
        # (sim time in simulation, wall clock on real hardware).
        time_usec = int(odom.header.stamp.sec) * 1000000 + int(odom.header.stamp.nanosec) // 1000
        stamp_class = self._epoch_gate.classify(time_usec)
        if stamp_class == "drop_invalid":
            self._invalid_stamp_count += 1
            return
        if stamp_class == "drop_duplicate":
            # No new odometry sample: re-sending the stale pose with a fresh
            # timestamp would feed the EKF phantom zero-velocity measurements.
            return
        if stamp_class == "drop_stale":
            # Regressed or implausibly-forward stamp: a delayed straggler or
            # an unconfirmed epoch break. Never sent (R003-F12 contract).
            self._stale_stamp_count += 1
            return
        if stamp_class == "reset_send":
            # Epoch change confirmed by consecutive monotonic samples (sim
            # relaunch, rosbag loop): atomically drop the dedup/slew state
            # and start a fresh epoch, otherwise the feed starves until sim
            # time outruns the pre-reset stamp.
            self._clock_reset_count += 1
            self._reset_counter = (self._reset_counter + 1) % 256
            self._last_sent_time_usec = None
            self._last_limited_odom_x = None
            self._last_limited_odom_y = None
            self._last_sent_yaw_rad = None
        # The XY slew limit must also run on measurement time; wall-clock dt
        # rescales the limit by the real-time factor (0.25 m/s wall was ~0.8 m/s
        # sim on the slow WSL host but chokes the feed on faster native hosts,
        # lagging the reported position behind the vehicle mid-oscillation).
        xy_dt_sec = (time_usec - self._last_sent_time_usec) / 1e6 if self._last_sent_time_usec is not None else 0.0
        odom_x_m, odom_y_m = rate_limit_xy(
            target_x=float(pose.position.x),
            target_y=float(pose.position.y),
            last_x=self._last_limited_odom_x,
            last_y=self._last_limited_odom_y,
            dt_sec=xy_dt_sec,
            max_speed_mps=self._max_horizontal_speed_mps,
        )
        self._last_limited_odom_x = odom_x_m
        self._last_limited_odom_y = odom_y_m
        x_m, y_m, z_m = ros_enu_position_to_mavlink_local_frd(
            x_enu_m=odom_x_m,
            y_enu_m=odom_y_m,
            z_enu_m=float(pose.position.z),
        )
        self._last_sent_x = x_m
        self._last_sent_y = y_m
        self._last_sent_xy_monotonic = now_monotonic
        self._last_sent_time_usec = time_usec

        q = self._odometry_quaternion(pose, now_monotonic=now_monotonic, meas_dt_sec=xy_dt_sec)
        rollspeed_radps, pitchspeed_radps = self._roll_pitch_speeds(twist, now_monotonic=now_monotonic)
        yawspeed_radps = self._limit_yawspeed(float(twist.angular.z))

        self._connection.mav.odometry_send(
            int(time_usec),
            mavlink.MAV_FRAME_LOCAL_FRD,
            mavlink.MAV_FRAME_BODY_FRD,
            x_m,
            y_m,
            z_m,
            q,
            float(twist.linear.x),
            -float(twist.linear.y),
            -float(twist.linear.z),
            rollspeed_radps,
            pitchspeed_radps,
            -yawspeed_radps,
            _upper_triangular_covariance(odom.pose.covariance),
            _upper_triangular_covariance(odom.twist.covariance),
            int(self._reset_counter),
            mavlink.MAV_ESTIMATOR_TYPE_VIO,
            int(self._quality),
        )
        self._sent_count += 1

    def _drain_mavlink(self, now_monotonic: float) -> None:
        while True:
            msg = self._connection.recv_match(blocking=False)
            if msg is None:
                return
            msg_type = msg.get_type()
            if msg_type == "HEARTBEAT" and int(msg.autopilot) != mavlink.MAV_AUTOPILOT_INVALID:
                self._target_system = msg.get_srcSystem()
                self._target_component = msg.get_srcComponent()
            elif msg_type == "ATTITUDE":
                self._fcu_roll_rad = float(msg.roll)
                self._fcu_pitch_rad = float(msg.pitch)
                self._fcu_yaw_rad = float(msg.yaw)
                self._fcu_rollspeed_radps = float(msg.rollspeed)
                self._fcu_pitchspeed_radps = float(msg.pitchspeed)
                self._last_fcu_attitude_monotonic = now_monotonic
                self._fcu_boot_ms_latest = max(
                    self._fcu_boot_ms_latest, int(getattr(msg, "time_boot_ms", 0))
                )
            elif msg_type == "LOCAL_POSITION_NED":
                self._local_position_count += 1
                self._last_local_position_monotonic = now_monotonic
                self._last_local_position_boot_ms = int(getattr(msg, "time_boot_ms", 0))
                self._fcu_boot_ms_latest = max(
                    self._fcu_boot_ms_latest, self._last_local_position_boot_ms
                )
                self._publish_local_position_pose(msg)

    def _request_fcu_attitude_if_needed(self, now_monotonic: float) -> None:
        if not self._requires_fcu_attitude() and not self._local_position_pose_pub:
            return
        if self._target_system is None or self._target_component is None:
            return
        if now_monotonic < self._next_stream_request_monotonic:
            return
        # OPEN-1 单变量实验(2026-07-28):此前无条件每 2s 重发 SET_MESSAGE_INTERVAL=
        # 流请求风暴,是 FCU 周期遥测空洞(心跳 gap 实测 2.1~284.6s;洞盖 arm 段→armed
        # 不可见→S3 死循环;洞盖 preflight→LP 缺)的上游嫌疑。改为:两条流都新鲜时不再
        # 重发;bring-up(从未见流)与流变陈旧(>STREAM_REREQUEST_STALE_SEC)时照常请求,
        # 保留启动鲁棒性与空洞后自恢复。
        attitude_fresh = (
            self._last_fcu_attitude_monotonic > 0.0
            and now_monotonic - self._last_fcu_attitude_monotonic < STREAM_REREQUEST_STALE_SEC
        )
        local_position_fresh = (
            self._last_local_position_monotonic > 0.0
            and now_monotonic - self._last_local_position_monotonic < STREAM_REREQUEST_STALE_SEC
        )
        if attitude_fresh and local_position_fresh:
            self._next_stream_request_monotonic = now_monotonic + 2.0
            return
        for message_id, hz in (
            (mavlink.MAVLINK_MSG_ID_HEARTBEAT, 2.0),
            (mavlink.MAVLINK_MSG_ID_ATTITUDE, 20.0),
            (mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 20.0),
        ):
            _send_message_interval(self._connection, self._target_system, self._target_component, message_id, hz)
        self._next_stream_request_monotonic = now_monotonic + 2.0

    def _publish_local_position_pose(self, msg: object) -> None:
        if self._local_position_pose_pub is None:
            return
        pose = ned_to_gazebo_pose(
            NedPoseSample(
                x_north_m=float(msg.x),
                y_east_m=float(msg.y),
                z_down_m=float(msg.z),
                yaw_rad=self._fcu_yaw_rad,
            ),
            min_z_m=0.0,
        )
        fields = build_pose_stamped_fields(pose)
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.pose.position.x = fields["x"]
        message.pose.position.y = fields["y"]
        message.pose.position.z = fields["z"]
        message.pose.orientation.x = fields["qx"]
        message.pose.orientation.y = fields["qy"]
        message.pose.orientation.z = fields["qz"]
        message.pose.orientation.w = fields["qw"]
        self._local_position_pose_pub.publish(message)

    def _odometry_quaternion(
        self, pose: object, *, now_monotonic: float | None = None, meas_dt_sec: float | None = None
    ) -> list[float]:
        if now_monotonic is None:
            now_monotonic = time.monotonic()
        if self._roll_pitch_source == "odom":
            return _ros_quat_to_frd(pose.orientation)
        if self._requires_fcu_attitude() and not self._fcu_attitude_fresh(now_monotonic):
            raise RuntimeError("fresh FCU ATTITUDE is required by the selected odometry attitude contract")

        if self._use_fcu_yaw:
            yaw_ned_rad = self._fcu_yaw_rad
        else:
            yaw_ned_rad = ros_enu_yaw_to_mavlink_local_frd(_yaw_from_ros_quat_enu(pose.orientation))
        if self._align_yaw_to_fcu and not self._use_fcu_yaw:
            if self._yaw_alignment_offset_rad is None:
                self._yaw_alignment_offset_rad = self._fcu_yaw_rad - yaw_ned_rad
            yaw_ned_rad += self._yaw_alignment_offset_rad
        yaw_ned_rad = self._limit_yaw(yaw_ned_rad, now_monotonic=now_monotonic, meas_dt_sec=meas_dt_sec)

        roll_rad = self._fcu_roll_rad if self._roll_pitch_source == "fcu" else 0.0
        pitch_rad = self._fcu_pitch_rad if self._roll_pitch_source == "fcu" else 0.0
        return _quat_from_roll_pitch_yaw_frd(
            roll_rad=float(roll_rad),
            pitch_rad=float(pitch_rad),
            yaw_rad=yaw_ned_rad,
        )

    def _requires_fcu_attitude(self) -> bool:
        return self._roll_pitch_source == "fcu" or self._use_fcu_yaw or self._align_yaw_to_fcu

    def _fcu_attitude_fresh(self, now_monotonic: float) -> bool:
        return (
            self._last_fcu_attitude_monotonic > 0.0
            and self._fcu_roll_rad is not None
            and self._fcu_pitch_rad is not None
            and now_monotonic - self._last_fcu_attitude_monotonic <= 1.0
        )

    def _roll_pitch_speeds(self, twist: object, *, now_monotonic: float) -> tuple[float, float]:
        if self._roll_pitch_source != "odom":
            return 0.0, 0.0
        return float(twist.angular.x), -float(twist.angular.y)

    def _limit_yaw(self, target_yaw_rad: float, *, now_monotonic: float, meas_dt_sec: float | None = None) -> float:
        # Prefer measurement-time dt (odometry header stamps): a wall-clock dt
        # rescales the yaw slew limit by the simulation real-time factor.
        if meas_dt_sec is not None:
            yaw_dt_sec = meas_dt_sec
        else:
            yaw_dt_sec = now_monotonic - self._last_sent_yaw_monotonic if self._last_sent_yaw_monotonic > 0.0 else 0.0
        yaw_rad = rate_limit_yaw(
            target_yaw_rad=target_yaw_rad,
            last_yaw_rad=self._last_sent_yaw_rad,
            dt_sec=yaw_dt_sec,
            max_yaw_rate_radps=self._max_yaw_rate_radps,
        )
        self._last_sent_yaw_rad = yaw_rad
        self._last_sent_yaw_monotonic = now_monotonic
        return yaw_rad

    def _limit_yawspeed(self, target_yawspeed_radps: float) -> float:
        if self._max_yaw_rate_radps <= 0.0:
            return target_yawspeed_radps
        return max(-self._max_yaw_rate_radps, min(self._max_yaw_rate_radps, target_yawspeed_radps))

    def _publish_status(self) -> None:
        age_ms = -1.0
        if self._last_odom is not None:
            age_ms = (time.monotonic() - self._last_odom_rx_monotonic) * 1000.0
        attitude_age_ms = -1.0
        if self._last_fcu_attitude_monotonic > 0.0:
            attitude_age_ms = (time.monotonic() - self._last_fcu_attitude_monotonic) * 1000.0
        local_position_age_ms = -1.0
        if self._last_local_position_boot_ms > 0 and self._fcu_boot_ms_latest > 0:
            # FCU sim-clock age (RTF/host-load independent). time_boot_ms is ms.
            local_position_age_ms = float(
                self._fcu_boot_ms_latest - self._last_local_position_boot_ms
            )
        elif self._last_local_position_monotonic > 0.0:
            # Fallback (no FCU time_boot_ms seen yet): host wall-clock age.
            local_position_age_ms = (time.monotonic() - self._last_local_position_monotonic) * 1000.0

        odom_fresh = self._last_odom is not None and 0.0 <= age_ms <= self._max_odom_age_ms
        local_position_fresh = (
            self._local_position_count > 0 and 0.0 <= local_position_age_ms <= self._max_local_position_age_ms
        )
        attitude_required = self._requires_fcu_attitude()
        attitude_fresh = self._fcu_attitude_fresh(time.monotonic())
        if self._last_odom is None:
            state = "waiting_for_external_nav_odom"
        elif attitude_required and not attitude_fresh:
            state = "waiting_for_fcu_attitude"
        else:
            state = "sending"
        ready = (
            state == "sending"
            and self._sent_count > 0
            and odom_fresh
            and local_position_fresh
            and (not attitude_required or attitude_fresh)
        )

        status = {
            "state": state,
            "ready": ready,
            "endpoint": self._endpoint,
            "input_topic": self._odom_topic,
            "sent_count": self._sent_count,
            "rate_hz": self._rate_hz,
            "odom_age_ms": round(age_ms, 3),
            "max_odom_age_ms": self._max_odom_age_ms,
            "max_horizontal_speed_mps": self._max_horizontal_speed_mps,
            "max_yaw_rate_radps": self._max_yaw_rate_radps,
            "last_sent_x": self._last_sent_x,
            "last_sent_y": self._last_sent_y,
            "last_sent_yaw_rad": self._last_sent_yaw_rad,
            "odom_fresh": odom_fresh,
            "frame_id": self._last_odom.header.frame_id if self._last_odom else "",
            "child_frame_id": self._last_odom.child_frame_id if self._last_odom else "",
            "mav_frame_id": MAVLINK_POSITION_FRAME,
            "mav_child_frame_id": MAVLINK_VELOCITY_FRAME,
            "quality": self._quality,
            "reset_counter": self._reset_counter,
            "estimator_type": MAVLINK_ESTIMATOR_TYPE,
            "time_usec_source": MAVLINK_TIME_SOURCE,
            "clock_reset_count": self._clock_reset_count,
            "invalid_stamp_count": self._invalid_stamp_count,
            "stale_stamp_count": self._stale_stamp_count,
            "use_fcu_roll_pitch": self._use_fcu_roll_pitch,
            "roll_pitch_source": self._roll_pitch_source,
            "align_yaw_to_fcu": self._align_yaw_to_fcu,
            "use_fcu_yaw": self._use_fcu_yaw,
            "yaw_alignment_offset_rad": self._yaw_alignment_offset_rad,
            "fcu_attitude_age_ms": round(attitude_age_ms, 3),
            "fcu_attitude_required": attitude_required,
            "fcu_attitude_ready": attitude_fresh,
            "local_position_pose_topic": self._local_position_pose_topic,
            "local_position_count": self._local_position_count,
            "local_position_age_ms": round(local_position_age_ms, 3),
            "max_local_position_age_ms": self._max_local_position_age_ms,
            "fcu_local_position_ready": local_position_fresh,
            "mapping": _odometry_mapping_status(
                input_topic=self._odom_topic,
                quality=self._quality,
                reset_counter=self._reset_counter,
                rate_hz=self._rate_hz,
                source_system=self._source_system,
                roll_pitch_source=self._roll_pitch_source,
                align_yaw_to_fcu=self._align_yaw_to_fcu,
                use_fcu_yaw=self._use_fcu_yaw,
            ),
        }
        msg = String()
        msg.data = json.dumps(status, separators=(",", ":"))
        self._status_pub.publish(msg)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send ROS ExternalNav odom as MAVLink ODOMETRY.")
    parser.add_argument("--endpoint", default="udpout:mavlink-router:14550")
    parser.add_argument("--odom-topic", default="/external_nav/odom")
    parser.add_argument("--status-topic", default="/mavlink_external_nav/status")
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--quality", type=int, default=100)
    parser.add_argument("--reset-counter", type=int, default=0)
    parser.add_argument("--source-system", type=int, default=191)
    parser.add_argument("--roll-pitch-source", choices=ROLL_PITCH_SOURCES, default=None)
    parser.add_argument("--use-fcu-roll-pitch", action=argparse.BooleanOptionalAction, default=None)
    parser.add_argument("--align-yaw-to-fcu", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--use-fcu-yaw", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--local-position-pose-topic", default="")
    parser.add_argument("--max-odom-age-ms", type=float, default=1000.0)
    parser.add_argument("--max-local-position-age-ms", type=float, default=1000.0)
    parser.add_argument("--max-horizontal-speed-mps", type=float, default=0.0)
    parser.add_argument("--max-yaw-rate-radps", type=float, default=0.0)
    args = parser.parse_args(argv)
    if args.roll_pitch_source is None:
        args.roll_pitch_source = "fcu" if args.use_fcu_roll_pitch else "odom"
    elif args.use_fcu_roll_pitch is not None and args.use_fcu_roll_pitch != (args.roll_pitch_source == "fcu"):
        parser.error("--use-fcu-roll-pitch conflicts with --roll-pitch-source")
    args.use_fcu_roll_pitch = args.roll_pitch_source == "fcu"
    if args.reset_counter < 0 or args.reset_counter > 255:
        parser.error("--reset-counter must be in [0, 255]")
    return args


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    if rclpy is None or mavutil is None or mavlink is None:
        raise SystemExit(
            "mavlink_external_nav_sender requires ROS2 Python packages and pymavlink. "
            "Run it with the ROS Python environment."
        )
    rclpy.init(args=None)
    node = MavlinkExternalNavSender(args)
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
