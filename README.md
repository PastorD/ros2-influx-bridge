# ROS 2 → InfluxDB bridge

A Python ROS 2 package that reads topic/field mappings from YAML. The bridge
loads installed message interfaces dynamically; it contains no per-message
application translators. Version 0.1.0 targets ROS 2 Jazzy and InfluxDB 2.

The implementation plan and acceptance criteria are in
[IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md). Execution results are in
[TEST_RESULTS.md](TEST_RESULTS.md).

## Install

On Ubuntu 24.04 with ROS 2 Jazzy installed, extract this directory and run:

```bash
source /opt/ros/jazzy/setup.bash
sudo apt-get install python3-venv python3-colcon-common-extensions
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install 'influxdb-client>=1.48,<2' 'PyYAML>=6,<7'
python -m colcon build --base-paths src --symlink-install
source install/setup.bash
```

Install/source each custom message package too. For example, if your robot's
interfaces are in another workspace, source that workspace's `install/setup.bash`
before starting the bridge. A missing interface is a validation error; the
bridge cannot reconstruct a custom message from a topic name alone.

The InfluxDB Python dependency is installed with pip above; it is not currently
declared as a rosdep key. This source package is not a published ROS binary
release. Replace the placeholder maintainer metadata before publishing one.

## Three commands

Dump a snapshot of the current graph:

```bash
ros2 run ros2_influx_bridge bridge dump \
  --output discovered.yaml --discovery-seconds 3
```

Dump includes every visible topic/type pair, including hidden topics and system
topics such as `/parameter_events`. It excludes endpoints created only by the
inspection node itself. It does not subscribe to data or write to InfluxDB.
`--output -` writes YAML to stdout. An existing file is protected unless
`--force` is supplied. Multiple types on one topic receive distinct measurement
names. The snapshot covers the current ROS domain/network and discovery window;
it is not an inventory of unreachable nodes or future publishers.

Validate YAML against installed schemas and current publishers:

```bash
ros2 run ros2_influx_bridge bridge validate \
  --config discovered.yaml --discovery-seconds 3

# Machine-readable output:
ros2 run ros2_influx_bridge bridge validate --config discovered.yaml --json

# Check syntax and installed message schemas without starting a ROS node:
ros2 run ros2_influx_bridge bridge validate --config discovered.yaml --offline
```

Live validation checks topic/type presence, at least one matching publisher,
field paths, type compatibility, conflicting field types within a measurement,
and QoS compatibility with every matching publisher. A currently absent topic
fails live validation. Offline mode needs explicit types and installed ROS
interfaces but does not inspect publishers. Validation never needs an InfluxDB
token, opens an HTTP connection, or sends telemetry. It does not verify database
availability, credentials, or existing database field schemas. A variable-array
index is schema-valid even when a particular sample's array is shorter.

Start forwarding:

```bash
export INFLUXDB_TOKEN='your-write-token'
ros2 run ros2_influx_bridge bridge run --config discovered.yaml
```

An installed alternate command is `ros2-influx-bridge run --config ...` (its
location follows the ROS package's executable directory). The launch interface is:

```bash
ros2 launch ros2_influx_bridge bridge.launch.py config:=/absolute/path/bridge.yaml
```

Use `--config`: this is application YAML, not a ROS `--params-file`. Use absolute
topic names in YAML. A topic type can be omitted during run if exactly one type
is discovered. Run waits for absent publishers, rechecks discovery every second,
and reports unresolved topics. Invalid installed schemas fail startup. Explicit
QoS incompatibilities are logged and the affected subscription waits for a
compatible configuration of publishers; other topics can continue.

Exit codes: 0 success/orderly stop, 2 configuration or validation error, 1
unexpected runtime failure. Standard argparse usage errors also return 2.

## Configuration

See [config/example.yaml](src/ros2_influx_bridge/config/example.yaml) for all
default limits and writer settings. Minimal example:

```yaml
version: 1
influxdb:
  url: http://localhost:8086
  org: robotics
  bucket: telemetry
  token_env: INFLUXDB_TOKEN
tags:
  robot_id: robot_1
topics:
  - topic: /gps/fix
    type: sensor_msgs/msg/NavSatFix
    measurement: gps
    throttle_hz: 10.0
    qos:
      reliability: auto
      durability: auto
      depth: 100
    fields:
      - latitude
      - lon: longitude
      - name: height
        field: altitude
        type: float64

  - topic: /custom/motor
    type: my_robot_msgs/msg/MotorState
    fields: []
```

The second message type is illustrative: install your real interface and change
the name. Missing, null, and empty `fields` all select every scalar leaf.
Measurement defaults to the topic path with slashes replaced by underscores.
Every point gets `topic` and `ros_type` tags; those names are reserved. Global
and per-topic static tags are supported; per-topic values override global ones.
Tags identify series but do not isolate InfluxDB field types within a measurement.

### Field selection

Each field-list entry accepts one of these forms:

```yaml
fields:
  - pose.position.x                 # Output name: pose.position.x
  - speed: twist.linear.x           # Output name: speed
  - field: latitude                 # Explicit type; default name: latitude
    type: float64
  - name: sample                    # Explicit name/path/type
    field: readings.0
    type: float64
```

These are syntax illustrations, not a declaration that one standard ROS message
has all those paths. Paths use dots for nested members and numeric array indices.
Selecting an entire nested message or array expands its scalar leaves. For
example `- p: pose` can produce `p.position.x`, and `- readings` produces
`readings.0`, `readings.1`, etc. Bare paths retain the entire path as their name.
An alias literally named `field` must use the explicit form (`name: field`).
Duplicate/overlapping output names are rejected.

Arrays flatten by index, including arrays of nested messages. Empty arrays emit
no fields. An explicitly selected index missing from a sample is omitted and
counted. Index-based names do not follow semantic identities when arrays reorder;
for example, generic JointState output follows array indices, not joint names.
The maximum array length, nesting depth, field count, and point bytes are bounded.
Exceeding a limit rejects the whole point and increments a rejection counter;
all-fields mode never silently truncates a large image/cloud into a partial record.

### Types and precision

| Type | Behavior |
|---|---|
| `auto` (default) | Infer bool/string/signed/unsigned/float from the ROS schema |
| `float64` or `float` | Preserve double precision; reject lossy integer-to-double conversion |
| `float32` | Explicitly round to float32 before writing; use only when intended |
| `int64` or `int` | Signed 64-bit; reject fractional values and overflow |
| `uint64` or `uint` | Unsigned 64-bit; reject negative/fractional values and overflow |
| `bool` | Accept ROS boolean fields |
| `string` | Convert a supported scalar to text |

InfluxDB 2 stores floating fields as 64-bit floats. `float32` is a deliberate
conversion before storage, not an InfluxDB float32 column. GPS double values
keep full precision with `auto`; an explicit type is not needed to repair the
six-significant-digit bug found in other bridges. A float32 source cannot gain
precision by being converted to float64. Integer and unsigned types use the
correct line-protocol suffixes.

NaN and infinity fields are omitted and counted while finite fields are kept.
A sample with no remaining fields produces no write. The client Point serializer
handles quoting/escaping; CR, LF, and NUL are rejected in names and strings.
An invalid value/conversion rejects the message and logs a rate-limited error.
Changing a field's type in an already populated measurement can conflict with
the database schema; validation cannot inspect that schema without a DB query.

### QoS

Supported settings: reliability (`auto`, `reliable`, `best_effort`), durability
(`auto`, `volatile`, `transient_local`), history (`keep_last`, `keep_all`), depth.
Depth defaults to 100. `keep_all` permits additional middleware memory use.

Automatic reliability selects reliable only when all matching publishers offer
reliable delivery. Otherwise it selects best effort. Automatic durability selects
transient-local only when all matching publishers offer it; mixed publishers use
volatile and produce a warning about historical samples. A latched map/status
publisher therefore works with automatic QoS when it is the only QoS profile.
ROS's compatibility check reports incompatible or uncertain publisher profiles.

Automatic QoS is reconsidered as publishers change. Recreating a subscription
can cause a brief gap or redeliver latched data. Explicit reliability/durability
avoid this automatic policy change. Deadline/liveliness are not user-configurable
in this release; middleware compatibility warnings remain visible.

### Throttling and timestamps

`throttle_hz: 0.0` means unlimited. A positive value accepts the first sample and
then at most one sample per interval using a monotonic clock. This is drop-based
throttling, not averaging or retain-latest sampling. The raw-subscription callback
throttles before deserialization, so skipped samples save decoding work. It still
receives their serialized DDS payloads; network traffic is not reduced.

`timestamp: receive` is the default: UTC Unix nanoseconds, assigned once per point.
Nanosecond ties/backwards wall-clock movement are made monotonic locally by a
minimal increment. `timestamp: header.stamp` selects a ROS Time-like field; zero
is allowed. Simulation stamps are not automatically transformed into today's
wall time. With header timestamps, equal measurement/tag-set/timestamp identities
can merge in InfluxDB. Retries always reuse the original timestamp/line.

### Buffering and delivery

One background worker owns one persistent HTTP client and synchronous batch
requests. Batch limits are count, bytes, and maximum age (default 500 ms).
Queue bounds count pending and in-flight records. Overflow drops the oldest
waiting record by default; an in-flight batch is protected, so a new record may
be dropped when all capacity is in flight. `drop_newest` is also available.

Transport errors, 408, 429, 5xx and recoverable authentication/configuration errors
(401/403/404/409) retain the batch with bounded backoff. Tokens supplied through
the environment are reread on write attempts; a process's environment is not
changed by exporting a variable in a different shell. Restart with corrected
credentials or use an application embedding that updates its own environment.
This release does not watch token files or expose a live credential-reload service.

413 batches are split; 400/422 batches are recursively isolated so valid neighboring
records can succeed. Irreducibly invalid records are dropped with counters/logs;
there is no persistent quarantine. A partly accepted batch may be resent with
the same identities. This is retry-based delivery, not an exactly-once guarantee.

**Buffering is RAM-only.** Pending records do not survive termination. Shutdown
has a drain deadline (default five seconds) and reports remaining records. An
already active HTTP call can outlive the deadline in the daemon writer thread;
the main shutdown wait remains bounded. For required lossless recording, record
the ROS data separately or add a future durable spool.

Structured `STATS` logs appear every ten seconds and `FINAL_STATS` on orderly stop.
Counters include received, throttled, serialized, skipped fields, rejected/empty
messages, acknowledged records, retries, overflow/invalid drops, pending bytes,
oldest queue age, and unresolved topics. `run --stats-file FILE` saves final JSON.
Configuration is read once at startup; runtime YAML reload is not implemented.

## Tests

Pure configuration/field/writer tests need Python dependencies only:

```bash
python -m pip install -r requirements-test.txt
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q tests/test_config_fields.py tests/test_writer.py
```

Build the independent custom-message fixture and run ROS schema/QoS tests:

```bash
python -m colcon build --base-paths src test_interfaces
source install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q tests/test_ros_schema.py
```

Run real DDS + CLI + InfluxDB integration with an InfluxDB 2.7.12 `influxd` binary:

```bash
export INFLUXD_BIN=/absolute/path/to/influxd
export ROS_DOMAIN_ID=77
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q -s tests \
  --junitxml=test-results.xml
```

The integration test starts a disposable database on free loopback ports, uses
disposable credentials, injects an HTTP outage, publishes real custom messages,
and queries stored values back. It also checks dump, validation, QoS, throttling,
latched and late publishers. Run it in an isolated ROS domain. ROS tests skip
when their dependencies are absent; DB integration skips without `INFLUXD_BIN`.
Disable pytest plugin autoload to avoid unrelated ROS launch-testing plugins
from affecting this pytest suite. The bridge itself needs no test-message package.

The included CI workflow is a reproducible recipe, not evidence of a hosted CI
run. See TEST_RESULTS.md for actual local executions and their limits.
