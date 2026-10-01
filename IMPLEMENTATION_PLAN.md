# ROS 2 to InfluxDB bridge — implementation and test plan

## Scope and decisions

Build a new Python `ament_python` package named `ros2_influx_bridge`, initially
targeting ROS 2 Jazzy and the InfluxDB 2 write API. Use `rclpy`,
`rosidl_runtime_py`, PyYAML, and `influxdb-client`. The ROS message packages
must be installed and sourced; the bridge must contain no application-message
imports or per-message branches. Start with bounded RAM buffering. SQLite and
InfluxDB 3 are outside this first release.

Preserve this plan alongside the source and execution results.

## Commands

- `bridge run --config FILE`: discover publishers, subscribe, transform and send.
- `bridge dump --output FILE --discovery-seconds 3`: capture every currently
  visible topic/type pair, including hidden/system topics. Write explicit types,
  automatic QoS, and `fields: []` (all fields). Capture once; never transmit
  telemetry. Refuse overwriting unless `--force` is given. Permit `--output -`.
- `bridge validate --config FILE --discovery-seconds 3`: syntax/schema checks,
  installed message types, field selectors/types, current publishers and QoS.
  Exit nonzero for errors. Do not initialize an InfluxDB client, require a token,
  send telemetry, or create data subscriptions. `--offline` checks configuration
  and installed message schemas without discovering the ROS graph.

Use `ros2 run ros2_influx_bridge bridge ...` or the installed `ros2-influx-bridge`
console script. Return JSON with `--json` for machine-readable validation.

## YAML contract

Top-level sections: `version: 1`, `influxdb`, `writer`, `limits`, `tags`, `topics`.
The file is application YAML, not the restricted ROS parameter-file schema.
Reject unknown keys and duplicate YAML keys with actionable errors.

Each topic has `topic`, optional ROS `type`, optional `measurement`, `fields`,
`qos`, `throttle_hz`, `tags`, and `timestamp`. Dump fills in the ROS type. Run
may infer an omitted type only if discovery finds exactly one type. Multiple
types require an explicit type. Topics appearing later are discovered periodically.

Field list entries support:

```yaml
fields:
  - latitude
  - lon: longitude
  - name: height
    field: altitude
    type: float64
  - field: position.x
    type: float64
```

Bare paths keep their entire path as the output name. Dot traversal supports
nested messages and numeric array indices (`values.0`). Selecting a nested
message or array expands its leaves, prefixed by the selected name. Missing,
null, or empty `fields` means all leaves. Numeric array indices give stable
field names; dynamic array lengths can change the emitted set. Empty arrays
emit no fields. Limits reject oversized messages visibly, never silently
truncate an "all fields" selection.

Types: `auto`, `float64`/`float`, `float32`, `int64`/`int`, `uint64`/`uint`,
`bool`, `string`. Infer the default from the ROS schema. Serialize floats with
round-trip precision; GPS doubles need no override. Explicit `float32` permits
lossy narrowing. Converting a float32 source to float64 cannot restore lost
precision. Reject unsafe integer conversions/overflow and incompatible types.
Omit NaN/Inf fields and count them; preserve other finite fields. Reject CR/LF
in names/strings/tags rather than emitting broken multiline records.

Use topic/type tags to identify source series, plus configured static tags.
Field types must remain consistent across each measurement, even with different
tags. Reject
duplicate output field names and reserved-tag overrides. Use receive Unix
nanoseconds by default, or an explicit ROS time field; retain the assigned
timestamp through retries. Header timestamps can be simulation time and can
collide; document those choices.

## QoS and throttling

QoS supports reliability, durability, history and depth. Default automatic
reliability is reliable if every matching publisher is reliable, otherwise
best effort. Automatic durability is transient-local only if every matching
publisher offers it; otherwise volatile. Report mixed durability and unknown
policies. Validate compatibility against every current matching publisher.
Re-evaluate automatic QoS as endpoints change; report subscription recreation
and its possible brief gap/latched redelivery. Explicit incompatible QoS must
be visible in validation and runtime logs.

Throttle independently per topic using a monotonic clock. Zero means unlimited;
otherwise accept the first sample and at most one per interval. Use raw
subscriptions to apply the throttle before deserialization. This saves Python
decoding work, not DDS traffic. Do not promise averaging or retain-latest semantics.

## Writer

Keep ROS callbacks independent of HTTP. Serialize accepted samples with the
InfluxDB client Point implementation and enqueue immutable lines. One worker
batches by count, bytes, or maximum age. Bound memory by record count and bytes,
including the in-flight batch; count overflow drops. Reuse one synchronous
client in the worker so acknowledgment is explicit.

Retry transient errors and retain authentication failures within queue limits.
Split 413 batches. Isolate 400/422 bad records and count irrecoverable drops.
Keep timestamps unchanged on retry. Honor bounded Retry-After/backoff and
interrupt waits on shutdown. Give shutdown a deadline; report unsent records.
RAM buffering does not survive termination. Publish structured log counters
for received, throttled, serialized, skipped fields, acknowledged, retried,
dropped, queue bytes and pending records.

## Package layout and sequence

1. Pure modules: strict config parsing, field paths, type conversion, flattening,
   timestamp handling, throttle and bounded writer.
2. ROS modules: schema introspection, graph discovery, QoS selection, validation,
   raw subscriptions and CLI.
3. Package metadata, launch file, YAML examples, installation instructions.
4. Unit/fault tests, then real ROS and InfluxDB integration, then packaging.

## Acceptance tests

| Area | Required checks |
|---|---|
| YAML | All field syntaxes; omitted/null/empty lists; duplicate keys/names; unknown keys; malformed types; negative limits/rates |
| Fields | Nested messages, arrays, aliases, indexed paths, absent array elements, full flattening, size/depth bounds |
| Precision | GPS double round-trip; no six-digit rounding; float32 narrowing explicit; uint64 boundaries; fractional-to-int rejection |
| Values | NaN/Inf preserve valid fields; quotes/backslashes/Unicode; CR/LF policy; empty messages |
| Discovery | Dump includes all visible topic/type pairs; deterministic ordering; no overwrite; dump re-validates |
| Validation | Missing topic/type package, ambiguous type, wrong field, scalar traversal, QoS mismatch, no database side effects |
| QoS | Reliable and best-effort publishers; mixed publishers; transient-local latched sample; late publishers/restarts |
| Throttle | First sample, zero/unlimited, interval boundary; wall/ROS time jumps do not affect monotonic throttle |
| Writer | Real batching; 503 then recovery; 401 retention; 413 split; mixed valid/invalid 400; byte/count bounds; bounded shutdown |
| ROS integration | Generated custom nested messages plus NavSatFix; run/dump/validate as separate processes using actual DDS |
| Database | Real InfluxDB writes/readback for field values, names, aliases, integers, precision and retry behavior |
| Performance | Representative small-message run; count delivery and throttle output. Do not infer target-hardware capacity from the disposable host. |

Run pure tests without ROS where possible. Generate a separate test interface
package for integration; do not make it a runtime dependency. Record exact
commands, versions and outcomes, distinguishing actual executions from
provided-but-unrun CI/launch examples. Keep builds, database contents and
third-party binaries out of the distributable ZIP.
