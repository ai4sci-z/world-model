"""OPEN-1 单变量实验(2026-07-28):SET_MESSAGE_INTERVAL 流请求风暴门控。

实证背景(BIN+tlog 交叉,样本 20260727T1114-1134):FCU 周期遥测流存在系统性空洞
(心跳 gap 2.1~284.6s;失败 run=洞盖住 arm 段→armed 不可见→S3 死循环;洞盖住 preflight
→LP 缺=经典 OPEN-1)。上游嫌疑:companion 每 2s 无条件重发 SET_MESSAGE_INTERVAL
(HEARTBEAT/ATTITUDE/LOCAL_POSITION_NED)=请求风暴,可能反复重置 FCU 流调度。

单变量改动:仅当流真变陈旧(>5s 墙钟无 ATTITUDE 或无 LP)才重发请求;流健康时不再骚扰。
守卫:bring-up(从未收到流)时必须照常持续请求——不丢启动鲁棒性。
"""

from __future__ import annotations

import time
from types import SimpleNamespace

from navlab.real.companion.nodes import external_nav as external_nav_module
from navlab.real.companion.nodes.external_nav import MavlinkExternalNavSender

if external_nav_module.mavlink is None:
    external_nav_module.mavlink = SimpleNamespace(
        MAV_FRAME_LOCAL_FRD=20,
        MAV_FRAME_BODY_FRD=12,
        MAV_ESTIMATOR_TYPE_VIO=3,
        MAVLINK_MSG_ID_HEARTBEAT=0,
        MAVLINK_MSG_ID_ATTITUDE=30,
        MAVLINK_MSG_ID_LOCAL_POSITION_NED=32,
    )


def _sender_for_stream_requests() -> tuple[MavlinkExternalNavSender, list]:
    sender = MavlinkExternalNavSender.__new__(MavlinkExternalNavSender)
    sender._use_fcu_roll_pitch = True
    sender._local_position_pose_pub = object()
    sender._target_system = 1
    sender._target_component = 1
    sender._connection = object()
    sender._next_stream_request_monotonic = 0.0
    sender._last_fcu_attitude_monotonic = 0.0
    sender._last_local_position_monotonic = 0.0
    sent: list = []
    return sender, sent


def _patch_send(monkeypatch, sent: list) -> None:
    monkeypatch.setattr(
        external_nav_module,
        "_send_message_interval",
        lambda conn, sys_, comp, mid, hz: sent.append(mid),
    )


def test_no_rerequest_while_streams_fresh(monkeypatch) -> None:
    """主用例(现行代码应红):ATTITUDE/LP 都新鲜时,不得再重发流请求(消风暴)。"""
    sender, sent = _sender_for_stream_requests()
    _patch_send(monkeypatch, sent)
    now = time.monotonic()
    sender._last_fcu_attitude_monotonic = now - 0.1
    sender._last_local_position_monotonic = now - 0.1
    sender._request_fcu_attitude_if_needed(now)
    assert sent == [], f"流新鲜仍重发请求(风暴未消): {sent}"


def test_requests_during_bringup_when_no_streams_yet(monkeypatch) -> None:
    """守卫:bring-up(从未收到任何流)必须照常请求——启动鲁棒性不丢。"""
    sender, sent = _sender_for_stream_requests()
    _patch_send(monkeypatch, sent)
    sender._request_fcu_attitude_if_needed(time.monotonic())
    assert len(sent) == 3, f"bring-up 未发流请求: {sent}"


def test_rerequests_when_stream_goes_stale(monkeypatch) -> None:
    """守卫:流真变陈旧(>阈值)时必须重发请求——空洞后能自恢复。"""
    sender, sent = _sender_for_stream_requests()
    _patch_send(monkeypatch, sent)
    now = time.monotonic()
    sender._last_fcu_attitude_monotonic = now - 30.0
    sender._last_local_position_monotonic = now - 30.0
    sender._request_fcu_attitude_if_needed(now)
    assert len(sent) == 3, f"流已陈旧却未重发请求: {sent}"


def test_rate_limit_still_applies(monkeypatch) -> None:
    """守卫:2s 节流窗口内不重复请求(与原行为一致)。"""
    sender, sent = _sender_for_stream_requests()
    _patch_send(monkeypatch, sent)
    now = time.monotonic()
    sender._request_fcu_attitude_if_needed(now)
    sender._request_fcu_attitude_if_needed(now + 0.5)
    assert len(sent) == 3, f"2s 窗口内重复请求: {sent}"
