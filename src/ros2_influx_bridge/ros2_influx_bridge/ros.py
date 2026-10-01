"""ROS discovery, schema validation, QoS and raw subscriptions."""
from collections import Counter
import json
import logging
import time

from .config import ConfigError
from .fields import DataError, Extractor, Throttle, message_schema, names_overlap, serialize_point, timestamp_ns, validate_timestamp


def graph_snapshot(node):
    """Exclude endpoints created by this inspection node, not other system topics."""
    def external(info):
        return (info.node_name, info.node_namespace) != (node.get_name(), node.get_namespace())
    graph = {}
    for topic, types in node.get_topic_names_and_types():
        pubs = [i for i in node.get_publishers_info_by_topic(topic) if external(i)]
        subs = [i for i in node.get_subscriptions_info_by_topic(topic) if external(i)]
        real_types = sorted(set(i.topic_type for i in pubs + subs))
        if real_types:
            graph[topic] = {"types": real_types, "publishers": pubs}
    return graph


def discover(node, seconds):
    import rclpy
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=min(0.1, max(0, deadline - time.monotonic())))
    return graph_snapshot(node)


def choose_qos(config, publishers):
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_check_compatible, QoSCompatibility
    notes, errors = [], []
    reliability = config["reliability"]
    durability = config["durability"]
    if reliability == "auto":
        reliability = "reliable" if publishers and all(i.qos_profile.reliability == ReliabilityPolicy.RELIABLE for i in publishers) else "best_effort"
    if durability == "auto":
        durability = "transient_local" if publishers and all(i.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL for i in publishers) else "volatile"
        if len({i.qos_profile.durability for i in publishers}) > 1:
            notes.append("Mixed publisher durability: volatile subscription receives live samples, not all historical samples")
    qos = QoSProfile(depth=config["depth"],
                     history=HistoryPolicy.KEEP_ALL if config["history"] == "keep_all" else HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.RELIABLE if reliability == "reliable" else ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL if durability == "transient_local" else DurabilityPolicy.VOLATILE)
    for info in publishers:
        compatibility, reason = qos_check_compatible(info.qos_profile, qos)
        label = f"Publisher {info.node_namespace}/{info.node_name}"
        if compatibility == QoSCompatibility.ERROR:
            errors.append(label + ": incompatible QoS: " + reason)
        elif compatibility == QoSCompatibility.WARNING:
            notes.append(label + ": QoS compatibility uncertain: " + reason)
    if config["history"] == "keep_all":
        notes.append("keep_all permits an unbounded DDS history subject to middleware resource limits")
    resolved = {"reliability": reliability, "durability": durability,
                "history": config["history"], "depth": config["depth"]}
    return qos, resolved, notes, errors


def prepare_topic(entry, limits, graph=None):
    from rosidl_runtime_py.utilities import get_message
    errors, warnings = [], []
    report = {"topic": entry["topic"], "type": entry["type"], "errors": errors, "warnings": warnings}
    found = (graph or {}).get(entry["topic"], {"types": [], "publishers": []})
    ros_type = entry["type"]
    if ros_type is None:
        if len(found["types"]) == 1:
            ros_type = found["types"][0]
        else:
            errors.append("Cannot infer one message type; configure type explicitly or wait for discovery")
            return report, None
    report["type"] = ros_type
    try:
        cls = get_message(ros_type)
        schema = message_schema(cls)
        extractor = Extractor(schema, entry["fields"], limits)
        validate_timestamp(schema, entry["timestamp"])
    except (ImportError, AttributeError, ValueError, LookupError) as exc:
        errors.append(f"Message schema/field error: {exc}")
        return report, None
    publishers = [p for p in found["publishers"] if p.topic_type == ros_type]
    if graph is not None:
        if ros_type not in found["types"]:
            errors.append(f"Topic/type is not currently visible; discovered types: {found['types']}")
        elif not publishers:
            errors.append("No current publisher for this topic/type")
    qos, resolved, notes, qos_errors = choose_qos(entry["qos"], publishers)
    warnings.extend(notes)
    errors.extend(qos_errors)
    report["publishers"] = len(publishers)
    report["qos"] = resolved
    if not entry["fields"]:
        warnings.append("All fields selected: nested messages and arrays will be flattened within configured limits")
    return report, {"class": cls, "extractor": extractor, "qos": qos, "type": ros_type, "resolved_qos": resolved}


def validate_config(config, graph=None):
    prepared = [prepare_topic(entry, config["limits"], graph) for entry in config["topics"]]
    reports = [pair[0] for pair in prepared]
    measurements = {}
    for entry, (report, topic) in zip(config["topics"], prepared):
        if topic is None:
            continue
        seen = measurements.setdefault(entry["measurement"], [])
        for name, dtype in topic["extractor"].output_fields:
            dtype = "float64" if dtype == "float32" else dtype
            for prior_name, prior_type in seen:
                if dtype != prior_type and names_overlap(name, prior_name):
                    report["errors"].append(f"Message schema/field conflict: measurement {entry['measurement']!r}, field {name!r} has both {prior_type} and {dtype}; use distinct measurements or an explicit common type")
            seen.append((name, dtype))
    return {"valid": all(not r["errors"] for r in reports), "mode": "offline" if graph is None else "live", "topics": reports,
            "warnings": [] if reports else ["Configuration contains no topics"]}


def dump_config(graph):
    from .config import DEFAULTS
    return {"version": 1, "influxdb": dict(DEFAULTS["influxdb"]), "topics": [
        {"topic": name, "type": ros_type, "measurement": name.strip("/").replace("/", "_") + ("__" + ros_type.replace("/", "_") if len(graph[name]["types"]) > 1 else ""),
         "fields": [], "qos": {"reliability": "auto", "durability": "auto", "history": "keep_last", "depth": 100},
         "throttle_hz": 0.0}
        for name in sorted(graph) for ros_type in sorted(graph[name]["types"])]}


class BridgeRuntime:
    def __init__(self, node, config, writer):
        self.node, self.config, self.writer = node, config, writer
        self.log = logging.getLogger("ros2_influx_bridge")
        self.counts = Counter()
        self.topic_counts = {str(i): Counter() for i in range(len(config["topics"]))}
        self.active, self.last_errors = {}, {}
        self.last_receive_ns = 0
        self.refresh()
        self.discovery_timer = node.create_timer(1.0, self.refresh)
        self.stats_timer = node.create_timer(10.0, self.log_stats)

    def refresh(self):
        graph = graph_snapshot(self.node)
        validation = validate_config(self.config, graph)
        for i, entry in enumerate(self.config["topics"]):
            report = validation["topics"][i]
            _, prepared = prepare_topic(entry, self.config["limits"], graph)
            if report["errors"]:
                message = "; ".join(report["errors"])
                if self.last_errors.get(i) != message:
                    self.log.warning("%s: %s", entry["topic"], message)
                    self.last_errors[i] = message
                # Keep a valid subscription while publishers temporarily disappear.
                # Stop an incompatible one so errors cannot be mistaken for coverage.
                if i in self.active and any("incompatible QoS" in e or "Message schema/" in e for e in report["errors"]):
                    self.node.destroy_subscription(self.active.pop(i)[1])
                continue
            self.last_errors.pop(i, None)
            signature = (prepared["type"], tuple(prepared["resolved_qos"].items()))
            if i in self.active and self.active[i][0] == signature:
                continue
            if i in self.active:
                self.node.destroy_subscription(self.active.pop(i)[1])
                self.counts["qos_recreations"] += 1
                self.log.warning("%s: QoS changed; recreating subscription (brief gap/latched redelivery possible)", entry["topic"])
            for warning in report["warnings"]:
                self.log.warning("%s: %s", entry["topic"], warning)
            throttle = Throttle(entry["throttle_hz"])
            callback = self._callback(i, entry, prepared, throttle)
            subscription = self.node.create_subscription(prepared["class"], entry["topic"], callback, prepared["qos"], raw=True)
            self.active[i] = (signature, subscription)
            self.log.info("Subscribed %s [%s] qos=%s", entry["topic"], prepared["type"], prepared["resolved_qos"])

    def _callback(self, index, entry, prepared, throttle):
        from rclpy.serialization import deserialize_message
        tags = {**self.config["tags"], **entry["tags"], "topic": entry["topic"], "ros_type": prepared["type"]}
        counters = self.topic_counts[str(index)]

        def callback(raw):
            counters["received"] += 1
            if not throttle.accept(time.monotonic()):
                counters["throttled"] += 1
                return
            try:
                msg = deserialize_message(raw, prepared["class"])
                fields, types, skipped = prepared["extractor"].extract(msg)
                counters["skipped_fields"] += skipped
                self.last_receive_ns = max(time.time_ns(), self.last_receive_ns + 1)
                stamp = timestamp_ns(msg, entry["timestamp"], self.last_receive_ns)
                line = serialize_point(entry["measurement"], tags, fields, types, stamp)
                if not line:
                    counters["empty_messages"] += 1
                    return
                if len(line.encode("utf-8")) > self.config["limits"]["max_point_bytes"]:
                    raise DataError("max_point_bytes exceeded")
                if self.writer.submit(line):
                    counters["serialized"] += 1
            except Exception as exc:
                counters["rejected_messages"] += 1
                # Log the first and then periodic rejections, never flood at topic rate.
                if counters["rejected_messages"] == 1 or counters["rejected_messages"] % 100 == 0:
                    self.log.error("%s: rejected message (%s)", entry["topic"], exc)
        return callback

    def snapshot(self):
        return {"runtime": dict(self.counts), "topics": {self.config["topics"][int(i)]["topic"] + "#" + i: dict(c) for i, c in self.topic_counts.items()},
                "writer": self.writer.snapshot(), "unresolved_topics": len(self.last_errors)}

    def log_stats(self):
        self.log.info("STATS %s", json.dumps(self.snapshot(), sort_keys=True))
