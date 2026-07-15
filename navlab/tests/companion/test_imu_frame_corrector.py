from __future__ import annotations

import pytest

from navlab.sim.companion.nodes.imu_frame_corrector import (
    CORRECTION_ROLL180_FLU,
    Vector3Data,
    correct_vector_roll180,
    corrected_imu_fields,
)


def test_static_gravity_flips_to_flu_convention() -> None:
    # Official iris SDF mounts the IMU sensor roll-180 inside imu_link, so the
    # bridged stream reads acc_z=-9.8 at rest; the corrected stream must read
    # +9.8 as a ROS FLU imu_link with identity TF implies.
    acc = correct_vector_roll180(Vector3Data(x=0.0, y=0.0, z=-9.8))
    assert acc == Vector3Data(x=0.0, y=0.0, z=9.8)


def test_roll180_negates_y_and_z_keeps_x() -> None:
    vec = correct_vector_roll180(Vector3Data(x=1.0, y=2.0, z=3.0))
    assert vec == Vector3Data(x=1.0, y=-2.0, z=-3.0)


def test_roll180_is_an_involution() -> None:
    original = Vector3Data(x=0.3, y=-1.7, z=4.2)
    assert correct_vector_roll180(correct_vector_roll180(original)) == original


def test_corrected_imu_fields_applies_to_both_vectors() -> None:
    gyro, acc = corrected_imu_fields(
        angular_velocity=Vector3Data(x=0.1, y=0.2, z=0.3),
        linear_acceleration=Vector3Data(x=0.0, y=0.5, z=-9.8),
        correction=CORRECTION_ROLL180_FLU,
    )
    assert gyro == Vector3Data(x=0.1, y=-0.2, z=-0.3)
    assert acc == Vector3Data(x=0.0, y=-0.5, z=9.8)


def test_unknown_correction_rejected() -> None:
    with pytest.raises(ValueError):
        corrected_imu_fields(
            angular_velocity=Vector3Data(x=0.0, y=0.0, z=0.0),
            linear_acceleration=Vector3Data(x=0.0, y=0.0, z=0.0),
            correction="yaw90",
        )
