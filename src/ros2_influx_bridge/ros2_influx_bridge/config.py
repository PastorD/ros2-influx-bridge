"""Strict application YAML, deliberately independent of ROS imports."""
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
import math
import re
from urllib.parse import urlparse

import yaml


class ConfigError(ValueError):
    pass


class UniqueLoader(yaml.SafeLoader):
    pass


def _mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if not isinstance(key, str):
            raise ConfigError(f"YAML keys must be strings (line {key_node.start_mark.line + 1})")
        if key in result:
            raise ConfigError(f"Duplicate YAML key {key!r} (line {key_node.start_mark.line + 1})")
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _mapping)
TYPE_ALIASES = {"float": "float64", "int": "int64", "uint": "uint64"}
FIELD_TYPES = {"auto", "float64", "float32", "int64", "uint64", "bool", "string"}
RESERVED_TAGS = {"topic", "ros_type"}


def text(value, where, *, empty=False):
    if not isinstance(value, str) or (not empty and not value):
        raise ConfigError(f"{where}: expected {'a' if empty else 'a nonempty'} string")
    if any(c in value for c in "\r\n\x00"):
        raise ConfigError(f"{where}: CR, LF and NUL are not supported")
    return value


def keys(value, allowed, where):
    if not isinstance(value, dict):
        raise ConfigError(f"{where}: expected a mapping")
    unknown = set(value) - set(allowed)
    if unknown:
        raise ConfigError(f"{where}: unknown keys {sorted(unknown)}")


def number(value, where, *, minimum=0, integer=False, positive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigError(f"{where}: expected a number")
    if not math.isfinite(value) or value < minimum or (positive and value <= 0):
        raise ConfigError(f"{where}: invalid value {value}")
    if integer and not isinstance(value, int):
        raise ConfigError(f"{where}: expected an integer")
    return value


def path_parts(path):
    text(path, "field path")
    parts = path.split(".")
    if not all(re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*|[0-9]+", p) for p in parts):
        raise ConfigError(f"Invalid dot field path {path!r}")
    return tuple(int(p) if p.isdigit() else p for p in parts)


@dataclass(frozen=True)
class FieldSpec:
    name: str
    path: str
    type: str = "auto"

    @property
    def parts(self):
        return path_parts(self.path)


def parse_fields(raw):
    if raw is None:
        return []
    if not isinstance(raw, list):
        raise ConfigError("fields must be a list")
    fields, names = [], set()
    for entry in raw:
        dtype = "auto"
        if isinstance(entry, str):
            name = path = entry
        elif isinstance(entry, dict) and "field" in entry:
            keys(entry, {"name", "field", "type"}, "field specification")
            path = entry["field"]
            name = entry.get("name", path)
            dtype = entry.get("type", "auto")
        elif isinstance(entry, dict) and len(entry) == 1:
            name, path = next(iter(entry.items()))
        else:
            raise ConfigError(f"Invalid field entry {entry!r}")
        text(name, "output field name")
        path_parts(path)
        dtype = TYPE_ALIASES.get(dtype, dtype) if isinstance(dtype, str) else None
        if dtype not in FIELD_TYPES:
            raise ConfigError(f"Unsupported field type {dtype!r}")
        if name in names:
            raise ConfigError(f"Duplicate output field name {name!r}")
        names.add(name)
        fields.append(FieldSpec(name, path, dtype))
    return fields


DEFAULTS = {
    "version": 1,
    "influxdb": {"url": "http://localhost:8086", "org": "robotics", "bucket": "telemetry",
                 "token_env": "INFLUXDB_TOKEN", "timeout_ms": 5000, "gzip": True},
    "writer": {"batch_size": 1000, "batch_bytes": 524288, "flush_ms": 500,
               "queue_size": 10000, "queue_bytes": 16777216, "overflow": "drop_oldest",
               "retry_initial_ms": 500, "retry_max_ms": 30000, "shutdown_timeout_sec": 5.0},
    "limits": {"max_fields": 4096, "max_array_length": 1024, "max_depth": 32,
               "max_point_bytes": 524288},
    "tags": {}, "topics": [],
}


def tags(raw, where):
    if not isinstance(raw, dict):
        raise ConfigError(f"{where}: expected key/value strings")
    for key, value in raw.items():
        text(key, where + " key")
        text(value, where + "." + key, empty=True)
        if key in RESERVED_TAGS:
            raise ConfigError(f"{where}: {key!r} is reserved")
    return dict(raw)


def parse_config(raw):
    keys(raw, DEFAULTS, "configuration")
    cfg = deepcopy(DEFAULTS)
    if type(raw.get("version", 1)) is not int or raw.get("version", 1) != 1:
        raise ConfigError("Only configuration version 1 is supported")
    for section in ("influxdb", "writer", "limits"):
        incoming = raw.get(section, {})
        allowed = set(DEFAULTS[section]) | ({"token"} if section == "influxdb" else set())
        keys(incoming, allowed, section)
        cfg[section].update(incoming)
    db = cfg["influxdb"]
    for k in ("url", "org", "bucket", "token_env"):
        text(db[k], "influxdb." + k)
    try:
        u = urlparse(db["url"])
        _ = u.port
    except ValueError as exc:
        raise ConfigError("influxdb.url has an invalid host/port") from exc
    if u.scheme not in {"http", "https"} or not u.hostname or u.username or u.password or u.query or u.fragment:
        raise ConfigError("influxdb.url must be an HTTP(S) endpoint without embedded credentials/query")
    if "token" in db:
        text(db["token"], "influxdb.token")
    number(db["timeout_ms"], "timeout_ms", integer=True, positive=True)
    if type(db["gzip"]) is not bool:
        raise ConfigError("influxdb.gzip must be boolean")
    for k, v in cfg["limits"].items():
        number(v, "limits." + k, integer=True, positive=True)
    for k, v in cfg["writer"].items():
        if k == "overflow":
            if not isinstance(v, str) or v not in {"drop_oldest", "drop_newest"}:
                raise ConfigError("writer.overflow must be drop_oldest or drop_newest")
        else:
            number(v, "writer." + k, integer=k != "shutdown_timeout_sec", positive=True)
    if cfg["writer"]["retry_initial_ms"] > cfg["writer"]["retry_max_ms"]:
        raise ConfigError("retry_initial_ms exceeds retry_max_ms")
    if cfg["writer"]["batch_bytes"] > cfg["writer"]["queue_bytes"]:
        raise ConfigError("batch_bytes exceeds queue_bytes")
    if cfg["writer"]["batch_size"] > cfg["writer"]["queue_size"]:
        raise ConfigError("batch_size exceeds queue_size")
    cfg["tags"] = tags(raw.get("tags", {}), "tags")
    topics = raw.get("topics", [])
    if not isinstance(topics, list):
        raise ConfigError("topics must be a list")
    seen = set()
    for entry in topics:
        keys(entry, {"topic", "type", "measurement", "fields", "qos", "throttle_hz", "tags", "timestamp"}, "topic entry")
        topic = text(entry.get("topic"), "topic")
        if not re.fullmatch(r"/(?:[A-Za-z_][A-Za-z0-9_]*)(?:/[A-Za-z_][A-Za-z0-9_]*)*", topic):
            raise ConfigError(f"Use a valid absolute topic name: {topic!r}")
        ros_type = entry.get("type")
        if ros_type is not None and (not isinstance(ros_type, str) or not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*/msg/[A-Za-z][A-Za-z0-9_]*", ros_type)):
            raise ConfigError(f"Invalid ROS message type {ros_type!r}")
        identity = (topic, ros_type)
        if identity in seen:
            raise ConfigError(f"Duplicate topic/type entry {identity}")
        seen.add(identity)
        measurement = text(entry.get("measurement", topic.strip("/").replace("/", "_")), "measurement")
        throttle = number(entry.get("throttle_hz", 0.0), "throttle_hz")
        timestamp = text(entry.get("timestamp", "receive"), "timestamp")
        if timestamp != "receive":
            path_parts(timestamp)
        qos = entry.get("qos", {})
        if qos == "auto":
            qos = {}
        keys(qos, {"reliability", "durability", "history", "depth"}, "qos")
        qos = {"reliability": "auto", "durability": "auto", "history": "keep_last", "depth": 100, **qos}
        for k, allowed in {"reliability": {"auto", "reliable", "best_effort"}, "durability": {"auto", "volatile", "transient_local"}, "history": {"keep_last", "keep_all"}}.items():
            if not isinstance(qos[k], str) or qos[k] not in allowed:
                raise ConfigError(f"Invalid qos.{k}: {qos[k]!r}")
        number(qos["depth"], "qos.depth", integer=True, positive=True)
        cfg["topics"].append({"topic": topic, "type": ros_type, "measurement": measurement,
                              "fields": parse_fields(entry.get("fields")), "qos": qos,
                              "throttle_hz": float(throttle), "tags": tags(entry.get("tags", {}), "topic.tags"),
                              "timestamp": timestamp})
    return cfg


def load_config(path):
    try:
        raw = yaml.load(Path(path).read_text(encoding="utf-8"), Loader=UniqueLoader)
        return parse_config(raw)
    except (OSError, yaml.YAMLError) as exc:
        raise ConfigError(str(exc)) from exc
