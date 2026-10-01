# Executed tests — ros2_influx_bridge 0.1.0

Tested on 2026-09-28. Final result: **64 passed in 20.87 seconds, zero skipped**.
Both ROS packages built with colcon, and the installed command
`ros2 run ros2_influx_bridge bridge --help` returned successfully.

## Environment

| Component | Actual test environment |
|---|---|
| Python | 3.12.14 |
| ROS | ROS 2 Jazzy packages, Fast DDS (`rmw_fastrtps_cpp`) |
| Custom interfaces | Generated locally by colcon/rosidl; actual serialized DDS messages |
| InfluxDB | Native InfluxDB OSS 2.7.12 process on temporary loopback ports |
| Python InfluxDB client | `influxdb-client` 1.50.0 |
| YAML | PyYAML 6.0.3 |
| Test runner | pytest 9.1.1; unrelated plugin autoload disabled |
| HTTP test proxy | requests 2.34.2 |

Tests ran in a hosted Linux workspace using installed/extracted ROS runtime
packages, without Docker. The source is intended for Ubuntu 24.04 with ROS 2
Jazzy; a clean Ubuntu machine installation was not separately tested.

## Results by layer

| Layer | Passed | What was checked |
|---|---:|---|
| Configuration and extraction | 50 | Strict YAML, duplicate keys, field forms, aliases, nested/indexed paths, all-field flattening, limits, conversions, precision, escaping, nonfinite values, timestamps, throttling |
| Writer fault tests | 7 | Batch count/age/bytes, bounded queues including in-flight data, retry/backoff, authentication failures, 413 splitting, 400 isolation, shutdown deadline |
| Actual ROS schemas and QoS | 6 | Generated custom messages and arrays, standard messages, mixed reliability, durability, cross-topic field-type conflicts, dump measurement disambiguation |
| Live ROS + CLI + database | 1 | Real publisher/subscriber processes, graph discovery, run/dump/validate, database outage/recovery, InfluxDB query readback |
| **Total** | **64** | **No skipped tests in the final run** |

The writer fault tests deliberately substitute an HTTP sink so error policies
are deterministic. The integration test uses actual ROS DDS and an actual
InfluxDB server; an HTTP proxy injects temporary 503 responses before forwarding
writes to that server.

## Final live integration run

- Dump discovered six topic/type pairs, including the test topics, a hidden
  topic, and the visible system topic. The resulting configuration re-validated.
  Overwriting an existing dump required `--force`.
- Invalid fields, absent topics, wrong message types, and explicit incompatible
  QoS each failed validation. Dump and validation made zero database write
  requests.
- Two custom-message topics each delivered 40 records: one selected all fields;
  the other used aliases, nested paths, arrays, and explicit types.
- Of 40 GPS messages received, 5 were serialized and 35 were throttled with a
  10 Hz setting. Counts depend on scheduling; this test verifies the configured
  upper rate and drop behavior, not an exact output count for every run.
- One transient-local sample published before the bridge started was received.
  One topic whose publisher appeared after bridge startup was discovered and
  forwarded.
- Six HTTP requests received the injected 503 status. After recovery, two
  successful batched requests stored all **87 accepted points**. The final
  queue had zero pending records and zero pending bytes.
- InfluxDB readback preserved GPS latitude `34.12345678912345`, maximum uint64
  `18446744073709551615`, nested numeric values, Unicode, quotes, and trailing
  backslashes. A nonfinite field was omitted while its finite neighbors survived.
- Orderly SIGINT shutdown returned exit code zero and wrote final counters.

The same live test also passed in an earlier run with 86 accepted points and
4 GPS points. The scheduling-dependent difference is expected for a drop-based
throttle; the final evidence bundle corresponds to the 87-point run.

## Reproduce

From the extracted project root, with ROS Jazzy and Python dependencies installed:

```bash
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
python -m colcon build --base-paths src test_interfaces
source install/setup.bash
export INFLUXD_BIN=/absolute/path/to/influxd
export ROS_DOMAIN_ID=77
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q -s tests \
  --junitxml=test-results.xml
ros2 run ros2_influx_bridge bridge --help
```

Use an isolated ROS domain. The test starts and stops its own database and uses
disposable credentials. Without the ROS fixture or `INFLUXD_BIN`, the relevant
tests skip; that reduced run does not establish database integration coverage.
The README includes a pure-Python test command for machines without ROS.

The hosted test command used explicit build/install directories and a Python
executable because the ROS runtime was extracted under the workspace. Those
environment-specific paths are not required on a normal Jazzy installation.

## Included evidence and limits

The `evidence/` directory contains JUnit output, build and command logs, dependency
versions, final integration counters, CLI results, and recorded HTTP statuses.
The database files, extracted ROS runtime, and third-party executables are not
included.

The GitHub Actions workflow is supplied but has not run on hosted CI. It runs
unit and ROS tests; the database integration test skips there until an
`INFLUXD_BIN` is provided. The launch file is supplied but its launch invocation
was not separately executed; the installed ROS executable was verified.

These results establish functionality for small telemetry messages. They do
not establish sustained throughput, CPU/memory capacity on robot hardware,
other ROS distributions or RMW implementations, InfluxDB 3 compatibility,
long-duration recovery, or survival of process/power failure. Buffering is
bounded and RAM-only; overflow and shutdown can lose pending records. No
SQLite dependency or durable spool is included in this version.
