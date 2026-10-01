"""Bounded, volatile queue and acknowledged batched writes."""
from collections import Counter, deque
from dataclasses import dataclass
from email.utils import parsedate_to_datetime
import logging
import os
import random
import threading
import time


class InfluxSink:
    def __init__(self, config):
        self.config = config
        self.client = self.api = None
        self.token = None
        if not self._token():
            raise ValueError(f"Set the environment variable {config['token_env']} or configure influxdb.token")

    def _token(self):
        return self.config.get("token") or os.environ.get(self.config["token_env"], "")

    def write(self, lines):
        from influxdb_client import InfluxDBClient
        from influxdb_client.client.write_api import SYNCHRONOUS
        token = self._token()
        if token != self.token or self.client is None:
            self.close()
            self.client = InfluxDBClient(url=self.config["url"], token=token, org=self.config["org"],
                                         timeout=self.config["timeout_ms"], enable_gzip=self.config["gzip"], retries=0)
            self.api = self.client.write_api(write_options=SYNCHRONOUS)
            self.token = token
        self.api.write(bucket=self.config["bucket"], org=self.config["org"], record="\n".join(lines), write_precision="ns")

    def close(self):
        if self.api is not None:
            self.api.close()
        if self.client is not None:
            self.client.close()
        self.api = self.client = None


@dataclass
class Record:
    line: str
    size: int
    enqueued: float


class BatchWriter:
    def __init__(self, config, sink, *, logger=None):
        self.config, self.sink = config, sink
        self.log = logger or logging.getLogger(__name__)
        self.queue = deque()
        self.condition = threading.Condition()
        self.counts = Counter()
        self.pending = self.pending_bytes = 0
        self.closing = False
        self.deadline = float("inf")
        self.inflight = []
        self.thread = threading.Thread(target=self._run, name="influx-writer", daemon=True)
        self.thread.start()

    def submit(self, line):
        record = Record(line, len(line.encode("utf-8")) + 1, time.monotonic())
        with self.condition:
            if self.closing or record.size > self.config["queue_bytes"]:
                self.counts["dropped_overflow"] += 1
                return False
            while self.pending >= self.config["queue_size"] or self.pending_bytes + record.size > self.config["queue_bytes"]:
                if self.config["overflow"] == "drop_newest" or not self.queue:
                    self.counts["dropped_overflow"] += 1
                    return False
                old = self.queue.popleft()
                self.pending -= 1
                self.pending_bytes -= old.size
                self.counts["dropped_overflow"] += 1
            self.queue.append(record)
            self.pending += 1
            self.pending_bytes += record.size
            self.counts["enqueued"] += 1
            self.condition.notify_all()
            return True

    def snapshot(self):
        with self.condition:
            oldest = min([r.enqueued for r in self.inflight[:1]] + [r.enqueued for r in list(self.queue)[:1]], default=time.monotonic())
            return {**self.counts, "pending": self.pending, "pending_bytes": self.pending_bytes,
                    "oldest_pending_sec": max(0.0, time.monotonic() - oldest)}

    def _stopped(self):
        return self.closing and time.monotonic() >= self.deadline

    def _take(self):
        with self.condition:
            while not self._stopped():
                if not self.queue:
                    if self.closing:
                        return []
                    self.condition.wait()
                    continue
                age_left = self.queue[0].enqueued + self.config["flush_ms"] / 1000 - time.monotonic()
                if not self.closing and len(self.queue) < self.config["batch_size"] and self.pending_bytes < self.config["batch_bytes"] and age_left > 0:
                    self.condition.wait(timeout=age_left)
                    continue
                batch, size = [], 0
                while self.queue and len(batch) < self.config["batch_size"]:
                    record = self.queue[0]
                    if batch and size + record.size > self.config["batch_bytes"]:
                        break
                    batch.append(self.queue.popleft())
                    size += record.size
                self.inflight = batch
                return batch
            return []

    def _retire(self, batch, counter):
        with self.condition:
            self.pending -= len(batch)
            self.pending_bytes -= sum(r.size for r in batch)
            self.counts[counter] += len(batch)
            retired = {id(r) for r in batch}
            self.inflight = [r for r in self.inflight if id(r) not in retired]
            self.condition.notify_all()

    def _wait(self, delay):
        end = time.monotonic() + delay
        with self.condition:
            while not self._stopped() and time.monotonic() < end:
                self.condition.wait(timeout=max(0.0, min(end, self.deadline) - time.monotonic()))

    def _deliver(self, batch):
        delay = self.config["retry_initial_ms"] / 1000
        cap = self.config["retry_max_ms"] / 1000
        while batch and not self._stopped():
            try:
                self.sink.write([r.line for r in batch])
            except Exception as exc:
                status = getattr(exc, "status", 0) or 0
                try:
                    status = int(status)
                except (TypeError, ValueError):
                    status = 0
                if status in {400, 413, 422}:
                    if len(batch) > 1:
                        mid = len(batch) // 2
                        self._deliver(batch[:mid])
                        self._deliver(batch[mid:])
                    else:
                        self.log.error("Discarding one rejected record (HTTP %s)", status)
                        self._retire(batch, "dropped_invalid")
                    return
                if 400 <= status < 500 and status not in {401, 403, 404, 408, 409, 429}:
                    self.log.error("Discarding rejected batch (HTTP %s)", status)
                    self._retire(batch, "dropped_invalid")
                    return
                with self.condition:
                    self.counts["retry_requests"] += 1
                    self.counts["last_http_status"] = status
                if delay == self.config["retry_initial_ms"] / 1000:
                    self.log.warning("Write failed; retaining batch for retry (HTTP %s)", status or "transport")
                retry_after = (getattr(exc, "headers", None) or {}).get("Retry-After")
                sleep = delay
                if retry_after:
                    try:
                        sleep = float(retry_after)
                    except (ValueError, TypeError):
                        try:
                            sleep = parsedate_to_datetime(retry_after).timestamp() - time.time()
                        except (ValueError, TypeError, OverflowError):
                            pass
                self._wait(min(cap, max(0.01, sleep * random.uniform(1.0, 1.1))))
                delay = min(cap, delay * 2)
            else:
                self._retire(batch, "acknowledged")
                return

    def _run(self):
        try:
            while True:
                batch = self._take()
                if not batch:
                    return
                self._deliver(batch)
                if self._stopped():
                    return
        finally:
            self.sink.close()

    def close(self, timeout=None):
        timeout = self.config["shutdown_timeout_sec"] if timeout is None else timeout
        with self.condition:
            self.closing = True
            self.deadline = time.monotonic() + timeout
            self.condition.notify_all()
        self.thread.join(timeout=timeout + 0.05)
        return self.snapshot()
