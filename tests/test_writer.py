import threading
import time

from ros2_influx_bridge.config import DEFAULTS
from ros2_influx_bridge.writer import BatchWriter


class HTTPError(Exception):
    def __init__(self, status, headers=None):
        self.status, self.headers = status, headers or {}


class Sink:
    def __init__(self, behavior=None):
        self.behavior, self.calls, self.accepted = behavior, [], []
    def write(self, lines):
        self.calls.append(list(lines))
        if self.behavior:
            self.behavior(lines)
        self.accepted.extend(lines)
    def close(self):
        pass


def config(**kwargs):
    return {**DEFAULTS["writer"], "flush_ms": 60, "retry_initial_ms": 10, "retry_max_ms": 30,
            "shutdown_timeout_sec": .3, **kwargs}


def wait_until(predicate, timeout=2):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(.005)
    raise AssertionError("Condition did not become true")


def test_batch_accumulates_until_age_deadline():
    sink = Sink(); writer = BatchWriter(config(), sink)
    try:
        for i in range(10):
            writer.submit(f"m x={i}i {i+1}")
            time.sleep(.002)
        assert sink.calls == []
        wait_until(lambda: len(sink.accepted) == 10)
        assert len(sink.calls) == 1
    finally:
        writer.close()


def test_batch_count_and_bytes_flush():
    sink = Sink(); writer = BatchWriter(config(batch_size=2, batch_bytes=100, flush_ms=10000), sink)
    try:
        for i in range(4):
            writer.submit(f"m v={i}i {i}")
        wait_until(lambda: len(sink.accepted) == 4)
        assert all(len(c) == 2 for c in sink.calls)
    finally:
        writer.close()
    sink = Sink(); writer = BatchWriter(config(batch_bytes=12, flush_ms=10000), sink)
    try:
        writer.submit("m v=1i 1"); writer.submit("m v=2i 2")
        wait_until(lambda: len(sink.accepted) >= 1)
        writer.close()
        assert len(sink.accepted) == 2 and all(len(c) == 1 for c in sink.calls)
    finally:
        writer.close()


def test_transient_and_auth_retained_until_recovery():
    for status in [503, 401, 403, 429]:
        healthy = threading.Event()
        def behavior(lines):
            if not healthy.is_set():
                raise HTTPError(status)
        sink = Sink(behavior); writer = BatchWriter(config(flush_ms=1), sink)
        try:
            for i in range(3): writer.submit(f"m v={i}i {i}")
            wait_until(lambda: len(sink.calls) >= 2)
            assert writer.snapshot()["pending"] == 3
            healthy.set()
            wait_until(lambda: writer.snapshot().get("acknowledged") == 3)
            assert writer.snapshot().get("dropped_invalid", 0) == 0
        finally:
            writer.close()


def test_413_splits_batch():
    def behavior(lines):
        if len(lines) > 2: raise HTTPError(413)
    sink = Sink(behavior); writer = BatchWriter(config(flush_ms=1000), sink)
    for i in range(6): writer.submit(f"m v={i}i {i}")
    final = writer.close()
    assert final["acknowledged"] == 6 and len(sink.accepted) == 6
    assert final["pending_bytes"] == 0


def test_bad_record_does_not_discard_good_neighbors():
    def behavior(lines):
        if "bad" in lines: raise HTTPError(400)
    sink = Sink(behavior); writer = BatchWriter(config(flush_ms=1000), sink)
    for line in ["m a=1i 1", "bad", "m a=2i 2"]: writer.submit(line)
    final = writer.close()
    assert final["acknowledged"] == 2 and final["dropped_invalid"] == 1
    assert final["pending"] == 0


def test_bounded_memory_includes_inflight():
    def behavior(lines): raise HTTPError(503)
    writer = BatchWriter(config(queue_size=3, queue_bytes=70, flush_ms=1), Sink(behavior))
    try:
        writer.submit("m x=1i 1")
        wait_until(lambda: writer.snapshot().get("retry_requests", 0) > 0)
        for i in range(100): writer.submit(f"m x={i}i {i}")
        stats = writer.snapshot()
        assert stats["pending"] <= 3 and stats["pending_bytes"] <= 70
        assert stats["dropped_overflow"] > 0
    finally:
        started = time.monotonic()
        final = writer.close(.05)
        assert time.monotonic() - started < .3
        assert final["pending"] > 0


def test_retry_after_and_arrivals_do_not_cause_busy_retry():
    times = []
    def behavior(lines):
        times.append(time.monotonic())
        if len(times) == 1: raise HTTPError(429, {"Retry-After": "0.1"})
    sink = Sink(behavior); writer = BatchWriter(config(flush_ms=1, retry_max_ms=200), sink)
    try:
        writer.submit("m v=1i 1")
        wait_until(lambda: len(times) == 1)
        for i in range(10):
            writer.submit(f"m v=2i {i+2}"); time.sleep(.003)
        wait_until(lambda: writer.snapshot().get("acknowledged") == 11)
        assert times[1] - times[0] >= .09
    finally:
        writer.close()
