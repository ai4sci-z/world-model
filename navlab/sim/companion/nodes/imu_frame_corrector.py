"""Republish sensor_msgs/Imu with a fixed mounting-convention correction.

Why this node exists: the official ArduPilot Gazebo iris models mount the IMU
sensor with ``<pose degrees="true">0 0 0 180 0 0</pose>`` inside ``imu_link``
(ArduPilotPlugin consumes the stream in FRD). The ros_gz bridge stamps the
resulting messages with ``frame_id=imu_link`` while the ROS-side TF tree keeps
``base_link -> imu_link`` at identity, so every FLU consumer (Cartographer in
particular) sees an upside-down IMU: static ``linear_acceleration.z`` reads
-9.8 instead of +9.8. This node applies the inverse of the roll-180 mount to
the *data* so the stream matches the frame it claims to be in. The frozen
official model stays untouched.

Roll-180 (about X) data correction: x stays, y and z negate — for both
angular velocity and linear acceleration. The orientation field is reset to
identity with ``orientation_covariance[0] = -1`` (per sensor_msgs/Imu, "no
orientation estimate"): downstream Cartographer uses only acc/gyro, and the
upstream orientation semantics under the flipped mount are not trustworthy.
"""

from __future__ import annotations

import argparse
import json
from collections.abc import Sequence
from dataclasses import dataclass

CORRECTION_ROLL180_FLU = "roll180_flu"


@dataclass(frozen=True)
class Vector3Data:
    x: float
    y: float
    z: float


def correct_vector_roll180(vector: Vector3Data) -> Vector3Data:
    """Rotate a body-frame vector by 180 degrees about X (roll)."""
    return Vector3Data(x=vector.x, y=-vector.y, z=-vector.z)


def corrected_imu_fields(
    *,
    angular_velocity: Vector3Data,
    linear_acceleration: Vector3Data,
    correction: str,
) -> tuple[Vector3Data, Vector3Data]:
    if correction != CORRECTION_ROLL180_FLU:
        raise ValueError(f"unsupported correction {correction!r}")
    return (
        correct_vector_roll180(angular_velocity),
        correct_vector_roll180(linear_acceleration),
    )


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Republish sensor_msgs/Imu with a mounting-convention data correction."
    )
    parser.add_argument("--input-topic", default="/imu")
    parser.add_argument("--output-topic", default="/navlab/slam/imu_source_flu")
    parser.add_argument("--status-topic", default="/navlab/slam/imu_source_flu/status")
    parser.add_argument("--correction", default=CORRECTION_ROLL180_FLU, choices=[CORRECTION_ROLL180_FLU])
    parser.add_argument("--qos-depth", type=int, default=50)
    return parser


def run(argv: Sequence[str] | None = None) -> int:
    args = _build_arg_parser().parse_args(argv)
    try:
        import rclpy
        from rclpy.executors import ExternalShutdownException
        from rclpy.node import Node
        from sensor_msgs.msg import Imu
        from std_msgs.msg import String
    except ModuleNotFoundError as exc:
        raise SystemExit("imu_frame_corrector requires ROS2 Python packages.") from exc

    class ImuFrameCorrector(Node):
        def __init__(self) -> None:
            super().__init__("navlab_imu_frame_corrector")
            self._count = 0
            self._last_input_frame_id = ""
            self._publisher = self.create_publisher(Imu, args.output_topic, args.qos_depth)
            self._status_publisher = self.create_publisher(String, args.status_topic, 10)
            self.create_subscription(Imu, args.input_topic, self._handle_imu, args.qos_depth)
            self.create_timer(0.5, self._publish_status)
            self.get_logger().info(f"correcting IMU {args.input_topic} -> {args.output_topic} ({args.correction})")

        def _handle_imu(self, msg: Imu) -> None:
            gyro, acc = corrected_imu_fields(
                angular_velocity=Vector3Data(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z),
                linear_acceleration=Vector3Data(
                    msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
                ),
                correction=args.correction,
            )
            out = Imu()
            out.header = msg.header
            out.angular_velocity.x = gyro.x
            out.angular_velocity.y = gyro.y
            out.angular_velocity.z = gyro.z
            out.angular_velocity_covariance = msg.angular_velocity_covariance
            out.linear_acceleration.x = acc.x
            out.linear_acceleration.y = acc.y
            out.linear_acceleration.z = acc.z
            out.linear_acceleration_covariance = msg.linear_acceleration_covariance
            out.orientation.w = 1.0
            out.orientation_covariance[0] = -1.0
            self._last_input_frame_id = msg.header.frame_id
            self._count += 1
            self._publisher.publish(out)

        def _publish_status(self) -> None:
            payload = {
                "state": "publishing" if self._count > 0 else "waiting_for_imu",
                "correction": args.correction,
                "input_topic": args.input_topic,
                "output_topic": args.output_topic,
                "message_count": self._count,
                "input_frame_id": self._last_input_frame_id,
            }
            message = String()
            message.data = json.dumps(payload, sort_keys=True)
            self._status_publisher.publish(message)

    rclpy.init()
    node = ImuFrameCorrector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(run())
