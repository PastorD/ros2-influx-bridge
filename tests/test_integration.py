"""Real DDS + CLI subprocesses + native InfluxDB 2; enabled by INFLUXD_BIN."""
import copy
import gzip
import http.server
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time
from collections import Counter

import pytest
import yaml

pytest.importorskip("rclpy")
pytest.importorskip("bridge_test_msgs.msg")
pytestmark = pytest.mark.skipif(not os.environ.get("INFLUXD_BIN"), reason="Set INFLUXD_BIN for real database integration")


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def wait_for(test, timeout=15):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if test(): return
        time.sleep(.03)
    raise AssertionError("Timed out waiting for integration condition")


def test_live_custom_messages_dump_validate_throttle_and_outage(tmp_path):
    import requests
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from bridge_test_msgs.msg import MotorTelemetry
    from sensor_msgs.msg import NavSatFix
    from geometry_msgs.msg import Vector3
    from std_msgs.msg import Float64
    from influxdb_client import InfluxDBClient

    session = requests.Session(); session.trust_env = False
    port = free_port(); url = f"http://127.0.0.1:{port}"
    token = "local-test-token-only"
    db_log = (tmp_path / "influxd.log").open("w")
    db = subprocess.Popen([os.environ["INFLUXD_BIN"], "--http-bind-address", f"127.0.0.1:{port}",
                           "--bolt-path", str(tmp_path / "db/influxd.bolt"), "--engine-path", str(tmp_path / "db/engine"), "--reporting-disabled"], stdout=db_log, stderr=db_log)
    bridge = node = server = None
    bridge_log = None
    commands = []
    initialized = False

    def cli(*args):
        env = dict(os.environ); env.pop("INFLUXDB_TOKEN", None)
        p = subprocess.run([sys.executable, "-m", "ros2_influx_bridge", *map(str, args)], env=env,
                           text=True, capture_output=True, timeout=20)
        commands.append({"args": list(map(str, args)), "returncode": p.returncode, "stdout": p.stdout, "stderr": p.stderr})
        return p

    try:
        def healthy():
            try: return session.get(url + "/health", timeout=.5).ok
            except requests.RequestException: return False
        wait_for(healthy)
        response = session.post(url + "/api/v2/setup", json={"username": "evaluation", "password": "local-evaluation-password", "org": "robotics", "bucket": "telemetry", "token": token}, timeout=5)
        response.raise_for_status()

        class Proxy(http.server.BaseHTTPRequestHandler):
            mode = 503
            records = []
            def log_message(self, *args): pass
            def do_POST(self):
                body = self.rfile.read(int(self.headers["Content-Length"]))
                if self.headers.get("Content-Encoding") == "gzip": body = gzip.decompress(body)
                if type(self).mode == 204:
                    r = session.post(url + self.path, data=body, headers={"Authorization": "Token " + token}, timeout=5)
                    status, data = r.status_code, r.content
                else:
                    status, data = type(self).mode, b'{"message":"temporary test outage"}'
                type(self).records.append({"status": status, "body": body.decode()})
                self.send_response(status); self.end_headers(); self.wfile.write(data)

        server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), Proxy)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        proxy_url = f"http://127.0.0.1:{server.server_port}"
        rclpy.init(); initialized = True
        node = rclpy.create_node("bridge_integration_publisher", enable_rosout=False)
        best = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        reliable = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        pubs = {
            "all": node.create_publisher(MotorTelemetry, "/bridge_test/all", best),
            "selected": node.create_publisher(MotorTelemetry, "/bridge_test/selected", reliable),
            "gps": node.create_publisher(NavSatFix, "/bridge_test/gps", best),
            "gps_reliable": node.create_publisher(NavSatFix, "/bridge_test/gps", reliable),
            "latched": node.create_publisher(MotorTelemetry, "/bridge_test/latched", latched),
            "hidden": node.create_publisher(Float64, "/_bridge_test_hidden", reliable),
        }
        msg = MotorTelemetry()
        msg.latitude = 34.12345678912345; msg.longitude = -118.12345678912345
        msg.temperature = math.nan; msg.current = 1.25; msg.enabled = True
        msg.pose.position.x = 12345.678901
        msg.label = 'drive,α says "OK" \\'
        msg.readings = [1.1, 2.2]; msg.vectors = [Vector3(x=1.0, y=2.0, z=3.0)]
        msg.sequence = 2**64 - 1
        msg.flags = [True, False]; msg.names = ['alpha', 'βeta']; msg.bytes = [0, 255]
        msg.wide_label = '位置'; msg.fixed = [3., 4.]
        pubs["latched"].publish(msg)

        dump = tmp_path / "discovered.yaml"
        result = cli("dump", "--output", dump, "--discovery-seconds", "1.5")
        assert result.returncode == 0, result.stderr
        dumped = yaml.safe_load(dump.read_text())
        names = {t["topic"] for t in dumped["topics"]}
        assert {"/bridge_test/all", "/bridge_test/selected", "/bridge_test/gps", "/bridge_test/latched", "/_bridge_test_hidden"} <= names
        assert cli("dump", "--output", dump).returncode == 2
        result = cli("validate", "--config", dump, "--discovery-seconds", "1.5", "--json")
        assert result.returncode == 0, result.stdout + result.stderr
        assert not Proxy.records, "dump/validate must not write to InfluxDB"

        config = {"version": 1, "influxdb": {"url": proxy_url, "org": "robotics", "bucket": "telemetry", "token_env": "BRIDGE_TEST_TOKEN"},
                  "tags": {"robot": "test,one=left"},
                  "writer": {"flush_ms": 100, "retry_initial_ms": 50, "retry_max_ms": 200},
                  "topics": [
                      {"topic": "/bridge_test/all", "type": "bridge_test_msgs/msg/MotorTelemetry", "measurement": "all"},
                      {"topic": "/bridge_test/selected", "type": "bridge_test_msgs/msg/MotorTelemetry", "measurement": "selected", "fields": ["latitude", {"lon": "longitude"}, {"name": "x", "field": "pose.position.x", "type": "float64"}, {"field": "sequence", "type": "uint64"}, "label", "readings.1"]},
                      {"topic": "/bridge_test/gps", "measurement": "gps", "fields": ["latitude", "longitude"], "throttle_hz": 10.0},
                      {"topic": "/bridge_test/latched", "type": "bridge_test_msgs/msg/MotorTelemetry", "measurement": "latched", "fields": ["latitude"]},
                  ]}
        path = tmp_path / "bridge.yaml"; path.write_text(yaml.safe_dump(config))
        result = cli("validate", "--config", path, "--discovery-seconds", "1.5", "--json")
        assert result.returncode == 0, result.stdout + result.stderr
        validated = json.loads(result.stdout)
        assert validated["topics"][2]["qos"]["reliability"] == "best_effort"
        for mutation in ["field", "missing", "qos", "type"]:
            bad = copy.deepcopy(config)
            if mutation == "field": bad["topics"][0]["fields"] = ["pose.wrong"]
            if mutation == "missing": bad["topics"][0]["topic"] = "/bridge_test/missing"
            if mutation == "qos": bad["topics"][0]["qos"] = {"reliability": "reliable"}
            if mutation == "type": bad["topics"][0]["type"] = "std_msgs/msg/String"
            badpath = tmp_path / (mutation + ".yaml"); badpath.write_text(yaml.safe_dump(bad))
            result = cli("validate", "--config", badpath, "--discovery-seconds", "1.0", "--json")
            assert result.returncode == 2 and not json.loads(result.stdout)["valid"], mutation
        assert not Proxy.records

        config["topics"].append({"topic": "/bridge_test/late", "type": "std_msgs/msg/Float64", "measurement": "late"})
        path.write_text(yaml.safe_dump(config))
        env = dict(os.environ); env["BRIDGE_TEST_TOKEN"] = token
        stats_file = tmp_path / "stats.json"
        bridge_log = (tmp_path / "bridge.log").open("w")
        bridge = subprocess.Popen([sys.executable, "-m", "ros2_influx_bridge", "run", "--config", str(path), "--discovery-seconds", "1", "--stats-file", str(stats_file)], env=env, stdout=bridge_log, stderr=bridge_log)
        wait_for(lambda: all(pubs[k].get_subscription_count() >= 1 for k in ["all", "selected", "gps", "latched"]))
        assert bridge.poll() is None
        # Late publisher is created after run's startup discovery.
        pubs["late"] = node.create_publisher(Float64, "/bridge_test/late", reliable)
        wait_for(lambda: pubs["late"].get_subscription_count() >= 1)
        pubs["late"].publish(Float64(data=12.5))
        count = 40
        gps = NavSatFix(latitude=msg.latitude, longitude=msg.longitude)
        for _ in range(count):
            pubs["all"].publish(msg); pubs["selected"].publish(msg); pubs["gps"].publish(gps)
            rclpy.spin_once(node, timeout_sec=0)
            time.sleep(.01)
        wait_for(lambda: any(r["status"] == 503 for r in Proxy.records))
        Proxy.mode = 204
        client = InfluxDBClient(url=url, token=token, org="robotics")
        def rows():
            return [r.values for t in client.query_api().query('from(bucket:"telemetry") |> range(start:-1h)') for r in t.records]
        def stored():
            data = rows()
            return sum(r["_measurement"] == "all" and r["_field"] == "latitude" for r in data) == count
        wait_for(stored)
        bridge.send_signal(signal.SIGINT); bridge.wait(timeout=10)
        assert bridge.returncode == 0, (tmp_path / "bridge.log").read_text()
        data = rows(); client.close()
        by_key = {}
        for row in data:
            by_key.setdefault((row["_measurement"], row["_field"]), []).append(row["_value"])
            assert row["robot"] == "test,one=left"
        assert len(by_key["all", "latitude"]) == count
        assert len(by_key["selected", "latitude"]) == count
        assert set(by_key["all", "latitude"]) == {msg.latitude}
        assert set(by_key["selected", "x"]) == {12345.678901}
        assert set(by_key["selected", "lon"]) == {msg.longitude}
        assert set(by_key["all", "sequence"]) == {2**64 - 1}
        assert set(by_key["all", "label"]) == {msg.label}
        assert set(by_key["all", "wide_label"]) == {'位置'}
        assert set(by_key["all", "flags.1"]) == {False}
        assert set(by_key["all", "bytes.1"]) == {255}
        assert "temperature" not in {f for m, f in by_key if m == "all"}
        assert 1 <= len(by_key["gps", "latitude"]) <= 10
        assert by_key["latched", "latitude"] == [msg.latitude]
        assert by_key["late", "data"] == [12.5]
        stats = json.loads(stats_file.read_text())
        assert stats["writer"]["pending"] == 0
        assert stats["topics"]["/bridge_test/gps#2"]["throttled"] > 0
        summary = {"custom_messages_per_topic": count, "gps_points": len(by_key["gps", "latitude"]),
                   "latched_points": 1, "late_publisher_points": 1, "dump_topic_pairs": len(dumped["topics"]),
                   "http_statuses": dict(Counter(r["status"] for r in Proxy.records)), "stats": stats,
                   "gps_latitude_roundtrip": by_key["gps", "latitude"][0], "uint64_roundtrip": by_key["all", "sequence"][0],
                   "unicode_and_backslash_roundtrip": True}
        (tmp_path / "integration-summary.json").write_text(json.dumps(summary, indent=2))
        (tmp_path / "http-records.json").write_text(json.dumps(Proxy.records, indent=2))
        print(json.dumps(summary, indent=2))
    finally:
        (tmp_path / "cli-results.json").write_text(json.dumps(commands, indent=2))
        if bridge and bridge.poll() is None:
            bridge.send_signal(signal.SIGINT)
            try: bridge.wait(timeout=10)
            except subprocess.TimeoutExpired: bridge.kill(); bridge.wait()
        if bridge_log: bridge_log.close()
        if node: node.destroy_node()
        if initialized and rclpy.ok(): rclpy.shutdown()
        if server: server.shutdown(); server.server_close()
        if db.poll() is None: db.send_signal(signal.SIGINT); db.wait(timeout=15)
        db_log.close()
