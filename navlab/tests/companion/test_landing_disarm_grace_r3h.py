"""R3-H 回归测试:降落 disarm/motors 门在 force-disarm 宽限期内的时序竞态。

背景(实证 run 20260727T111415):slam_hover_probe 采样 /navlab/landing/status 抢在
"触地后、force_disarm 3s 宽限期未走完"时快照,抓到 disarmed=False/force_disarm_used=False,
经 build_landing_summary 误报 disarm_not_confirmed/motors_not_safe;而 mission 终态(force-disarm
完成后)disarmed=True/blockers=[]。飞行/降落本身正常,是采样时机太早。

修复语义(不掩盖真失败):只有当"上锁机制已给过机会"(force_disarm_used=True,或本就不靠
force_disarm)却仍未上锁,才判 disarm_not_confirmed。force-disarm 尚未触发时=上锁进行中,不算失败。
"""

from __future__ import annotations

from navlab.common.companion.mission.evidence.summary import build_landing_summary
from navlab.common.companion.mission.fsm import MissionPhaseRecorder
from navlab.common.companion.mission.stages.landing import LANDING_POLICY_AP_LAND_MODE_AFTER_HOVER


def _snapshot():
    recorder = MissionPhaseRecorder(started_at_monotonic=10.0)
    recorder.transition(now_monotonic=12.0, state="S6 hover_hold", reason="holding", guard="hover_hold")
    return recorder.snapshot(now_monotonic=13.5)


def _landing_summary(*, disarmed: bool, motors_safe: bool, force_disarm_after_touchdown: bool, force_disarm_used: bool):
    return build_landing_summary(
        fsm_snapshot=_snapshot(),
        policy=LANDING_POLICY_AP_LAND_MODE_AFTER_HOVER,
        state="landing_complete",
        started=True,
        frozen_hover_evidence={"ok": True},
        land_command_sent=True,
        land_command_sent_time_sec=5.0,
        land_command_accepted=True,
        mode_before_land="GUIDED",
        mode_after_land="LAND",
        land_mode_seen=True,
        land_mode_seen_elapsed_sec=0.4,
        landed_state_timeline=[],
        landing_duration_sec=9.7,
        touchdown_confirmed=True,
        touchdown_confirmed_time_sec=8.0,
        disarmed=disarmed,
        motors_safe=motors_safe,
        require_disarm=True,
        require_motors_safe=True,
        touchdown_confirm_sec=0.5,
        force_disarm_grace_sec=3.0,
        force_disarm_after_touchdown=force_disarm_after_touchdown,
        force_disarm_used=force_disarm_used,
        landing_setpoint_lookahead_sec=0.2,
        landing_slowdown_altitude_m=0.6,
        landing_near_ground_descent_rate_mps=0.01,
        last_range_m=0.0,
        last_rangefinder_relative_height_m=0.0,
        last_z_ned=0.0,
        last_vz_mps=1.2,
        landed_state="ON_GROUND",
        fcu_land_params={},
        descent_profile={"ok": True, "speed_ok": True, "bounce_ok": True},
        landing_blockers=[],
    )


def test_force_disarm_pending_within_grace_is_not_a_disarm_failure() -> None:
    """R3-H 主用例:force-disarm 尚未触发(pending)时,不得误报 disarm/motors 失败。"""
    summary = _landing_summary(
        disarmed=False, motors_safe=False, force_disarm_after_touchdown=True, force_disarm_used=False
    )
    assert "disarm_not_confirmed" not in summary["blockers"], summary["blockers"]
    assert "motors_not_safe" not in summary["blockers"], summary["blockers"]


def test_force_disarm_used_but_still_armed_still_flags() -> None:
    """守卫:force-disarm 已触发(给过机会)却仍未上锁 = 真失败,必须照报(不掩盖)。"""
    summary = _landing_summary(
        disarmed=False, motors_safe=False, force_disarm_after_touchdown=True, force_disarm_used=True
    )
    assert "disarm_not_confirmed" in summary["blockers"], summary["blockers"]
    assert "motors_not_safe" in summary["blockers"], summary["blockers"]


def test_no_force_disarm_and_not_disarmed_still_flags() -> None:
    """守卫:不靠 force-disarm(靠 land-mode 自动上锁)却没上锁 = 真失败,必须照报。"""
    summary = _landing_summary(
        disarmed=False, motors_safe=False, force_disarm_after_touchdown=False, force_disarm_used=False
    )
    assert "disarm_not_confirmed" in summary["blockers"], summary["blockers"]
    assert "motors_not_safe" in summary["blockers"], summary["blockers"]


def test_disarmed_success_has_no_blockers() -> None:
    """正常成功:已上锁 → 无 disarm/motors 阻塞(两版本都应通过)。"""
    summary = _landing_summary(
        disarmed=True, motors_safe=True, force_disarm_after_touchdown=True, force_disarm_used=True
    )
    assert "disarm_not_confirmed" not in summary["blockers"], summary["blockers"]
    assert "motors_not_safe" not in summary["blockers"], summary["blockers"]
