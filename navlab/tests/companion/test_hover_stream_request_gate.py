"""OPEN-1 流请求风暴门控(mission 侧,86% 风暴源)的语义测试。

背景同 test_stream_request_storm_open1.py:run5 一 run 内 SET_MESSAGE_INTERVAL ×3114,
其中 hover_mission(sys246)9 消息×每2s=2664 次(86%);命令/ACK 风暴挤压 FCU TX →
周期消息空洞 2~285s → 洞盖 arm 段 → armed 不可见 → S3 死循环。
门控语义:节流窗口内不发;bring-up(从未见周期流)照常发;流新鲜不发;真陈旧再发(自恢复)。
"""

from __future__ import annotations

from navlab.sim.companion.nodes.hover_mission import (
    STREAM_REREQUEST_STALE_SEC,
    should_rerequest_streams,
)


def test_no_rerequest_while_periodic_stream_fresh() -> None:
    """主用例:周期流新鲜(刚收到心跳/LP)→ 不重发(消 86% 风暴)。"""
    assert should_rerequest_streams(
        now_monotonic=100.0, next_request_monotonic=99.0, last_periodic_stream_monotonic=99.9
    ) is False


def test_requests_during_bringup_when_stream_never_seen() -> None:
    """守卫:bring-up(从未见周期流)必须照常请求——启动鲁棒性不丢。"""
    assert should_rerequest_streams(
        now_monotonic=100.0, next_request_monotonic=99.0, last_periodic_stream_monotonic=0.0
    ) is True


def test_rerequests_when_stream_stale() -> None:
    """守卫:周期流真陈旧(>=stale)必须重发——空洞后自恢复。"""
    assert should_rerequest_streams(
        now_monotonic=100.0,
        next_request_monotonic=99.0,
        last_periodic_stream_monotonic=100.0 - STREAM_REREQUEST_STALE_SEC,
    ) is True


def test_rate_limit_window_blocks_even_when_stale() -> None:
    """守卫:2s 节流窗口内即使陈旧也不发(保持既有节流,防新风暴)。"""
    assert should_rerequest_streams(
        now_monotonic=100.0, next_request_monotonic=101.5, last_periodic_stream_monotonic=0.0
    ) is False
