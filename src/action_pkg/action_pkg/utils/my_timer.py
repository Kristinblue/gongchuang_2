"""非阻塞式计时器模块。"""

import time


def _ticks_ms() -> int:
    """获取单调递增的毫秒计数。

    Returns:
        从系统单调时钟起点经过的整数毫秒数。
    """
    return time.monotonic_ns() // 1_000_000
# 不用time.time,防止NTP干扰


class MyTimer:
    """我的非阻塞式计时器。"""

    def __init__(self, now_ms=_ticks_ms) -> None:
        """初始化一个尚未运行的计时器。"""
        self._start_time = 0
        self._is_running = False
        self._now_ms = now_ms

    def start(self) -> None:
        """开始计时；已经运行时不覆盖原来的起始时间。"""
        if not self._is_running:
            self._start_time = self._now_ms()
            self._is_running = True

    def reset(self) -> None:
        """清除计时起点并停止计时。"""
        self._is_running = False
        self._start_time = 0

    def restart(self) -> None:
        """无论当前状态如何，都从现在重新开始计时。"""
        self._start_time = self._now_ms()
        self._is_running = True

    def get_elapsed(self) -> int:
        """获取已经经过的时间。

        Returns:
            计时器运行后经过的整数毫秒数；未运行时返回 0。
        """
        if not self._is_running:
            return 0
        return self._now_ms() - self._start_time

    def is_timeout(self, target_ms: int | float) -> bool:
        """判断是否达到目标时间。

        Args:
            target_ms: 目标时长，单位为毫秒。

        Returns:
            正在运行且已经达到目标时长时返回 True，否则返回 False。
        """
        if not self._is_running:
            return False
        return self.get_elapsed() >= target_ms

    @property
    def is_running(self) -> bool:
        """返回计时器当前是否正在运行。"""
        return self._is_running
