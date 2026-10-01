"""Message-schema traversal and loss-aware scalar conversion."""
from dataclasses import dataclass, field
from functools import lru_cache
import math
import numbers
import struct

from .config import ConfigError, path_parts, text


class DataError(ValueError):
    pass


@dataclass
class Schema:
    kind: str
    dtype: str = ""
    children: dict = field(default_factory=dict)
    element: "Schema | None" = None
    bound: int | None = None
    fixed: bool = False


@lru_cache(maxsize=256)
def message_schema(message_class):
    """Use generated ROS metadata, including nested and sequence element types."""
    from rosidl_runtime_py.utilities import get_message

    def describe(slot):
        if hasattr(slot, "value_type"):
            bound = getattr(slot, "size", getattr(slot, "maximum_size", None))
            return Schema("array", element=describe(slot.value_type), bound=bound,
                          fixed=hasattr(slot, "size"))
        if hasattr(slot, "namespaces"):
            cls = get_message("/".join([*slot.namespaces, slot.name]))
            return message_schema(cls)
        if "string" in type(slot).__name__.lower():
            return Schema("scalar", "string")
        typename = getattr(slot, "typename", "")
        if typename in {"float", "double", "float32", "float64"}:
            return Schema("scalar", "float64")
        if typename in {"boolean", "bool"}:
            return Schema("scalar", "bool")
        if typename.startswith("uint") or typename in {"byte", "octet", "char", "wchar"}:
            return Schema("scalar", "uint64")
        if typename.startswith("int"):
            return Schema("scalar", "int64")
        raise ConfigError(f"Unsupported ROS field type {slot!r}")

    return Schema("message", children={name: describe(slot) for name, slot in
                                       zip(message_class.get_fields_and_field_types(), message_class.SLOT_TYPES)})


def select_schema(schema, parts):
    for part in parts:
        if schema.kind == "message" and isinstance(part, str) and part in schema.children:
            schema = schema.children[part]
        elif schema.kind == "array" and isinstance(part, int):
            if schema.bound is not None and part >= schema.bound:
                raise ConfigError(f"Array index {part} exceeds declared bound {schema.bound}")
            schema = schema.element
        else:
            raise ConfigError(f"Cannot traverse {part!r} in {schema.kind} schema")
    return schema


def scalar_leaves(schema, prefix="", depth=0):
    """Symbolic leaf paths for collision/type checks, even for empty arrays."""
    if depth > 64:
        raise ConfigError("Schema nesting exceeds 64 levels")
    if schema.kind == "scalar":
        yield prefix, schema
    elif schema.kind == "message":
        for name, child in schema.children.items():
            yield from scalar_leaves(child, prefix + "." + name if prefix else name, depth + 1)
    else:
        yield from scalar_leaves(schema.element, prefix + ".*" if prefix else "*", depth + 1)


def compatible_cast(source, target):
    if target in {"auto", "string"}:
        return True
    if target == "bool":
        return source == "bool"
    return source in {"float64", "float32", "int64", "uint64"} and target in {"float64", "float32", "int64", "uint64"}


def names_overlap(first, second):
    a, b = first.split("."), second.split(".")
    return len(a) == len(b) and all(x == y or (x == "*" and y.isdigit()) or (y == "*" and x.isdigit()) for x, y in zip(a, b))


class Extractor:
    def __init__(self, schema, specs, limits):
        self.schema, self.limits = schema, limits
        self.selectors = [(s, s.parts, select_schema(schema, s.parts)) for s in specs]
        outputs = []
        self.output_fields = []
        if not self.selectors:
            self.output_fields = [(name, leaf.dtype) for name, leaf in scalar_leaves(schema)]
        for spec, _, child in self.selectors:
            for suffix, leaf in scalar_leaves(child):
                if not compatible_cast(leaf.dtype, spec.type):
                    raise ConfigError(f"{spec.path}: cannot convert {leaf.dtype} to {spec.type}")
                name = spec.name + ("." + suffix if suffix else "")
                for previous in outputs:
                    # Wildcard array indices can overlap explicit numeric selectors.
                    if names_overlap(previous, name):
                        raise ConfigError(f"Overlapping output field names: {previous!r}, {name!r}")
                outputs.append(name)
                self.output_fields.append((name, leaf.dtype if spec.type == "auto" else spec.type))

    def extract(self, message):
        values, types = {}, {}
        skipped = 0

        def walk(value, schema, name, dtype, depth):
            nonlocal skipped
            if depth > self.limits["max_depth"]:
                raise DataError("max_depth exceeded")
            if schema.kind == "message":
                for key, child in schema.children.items():
                    walk(getattr(value, key), child, name + "." + key if name else key, dtype, depth + 1)
            elif schema.kind == "array":
                if len(value) > self.limits["max_array_length"]:
                    raise DataError("max_array_length exceeded")
                for i in range(len(value)):
                    walk(value[i], schema.element, f"{name}.{i}", dtype, depth + 1)
            else:
                chosen = schema.dtype if dtype == "auto" else dtype
                if isinstance(value, numbers.Real) and not math.isfinite(value):
                    skipped += 1
                    return
                converted = convert_scalar(value, chosen)
                if name in values:
                    raise DataError(f"Duplicate emitted field {name}")
                if len(values) >= self.limits["max_fields"]:
                    raise DataError("max_fields exceeded")
                values[name] = converted
                types[name] = {"int64": "int", "uint64": "uint", "float64": "float", "float32": "float"}.get(chosen, chosen)

        if not self.selectors:
            walk(message, self.schema, "", "auto", 0)
        else:
            for spec, parts, schema in self.selectors:
                value = message
                try:
                    for part in parts:
                        value = value[part] if isinstance(part, int) else getattr(value, part)
                except IndexError:
                    skipped += 1
                    continue
                walk(value, schema, spec.name, spec.type, 0)
        return values, types, skipped


def convert_scalar(value, dtype):
    if dtype == "string":
        try:
            return text(str(value), "string field", empty=True)
        except ConfigError as exc:
            raise DataError(str(exc)) from exc
    if dtype == "bool":
        if not isinstance(value, bool):
            raise DataError("Only ROS booleans can be converted to bool")
        return value
    if isinstance(value, bool) or not isinstance(value, numbers.Real):
        raise DataError(f"Non-numeric value for {dtype}")
    if dtype in {"int64", "uint64"}:
        n = int(value)
        if n != value:
            raise DataError("Fractional value cannot be converted to an integer")
        lo, hi = (0, 2**64 - 1) if dtype == "uint64" else (-2**63, 2**63 - 1)
        if not lo <= n <= hi:
            raise DataError(f"Value outside {dtype} range")
        return n
    result = float(value)
    if isinstance(value, numbers.Integral) and result != value:
        raise DataError("Integer cannot be represented exactly as float64")
    if dtype == "float32":
        try:
            result = struct.unpack("!f", struct.pack("!f", result))[0]
        except OverflowError as exc:
            raise DataError("Value outside float32 range") from exc
    if not math.isfinite(result):
        raise DataError("Nonfinite converted value")
    return result


def timestamp_ns(message, path, receive_ns):
    if path == "receive":
        return receive_ns
    value = message
    try:
        for part in path_parts(path):
            value = value[part] if isinstance(part, int) else getattr(value, part)
        if not 0 <= value.nanosec < 1_000_000_000:
            raise DataError("Invalid timestamp nanosec")
        stamp = value.sec * 1_000_000_000 + value.nanosec
        if not -9223372036854775806 <= stamp <= 9223372036854775806:
            raise DataError("Timestamp outside InfluxDB range")
        return stamp
    except (AttributeError, IndexError, TypeError) as exc:
        raise DataError(f"Invalid timestamp path {path}") from exc


def validate_timestamp(schema, path):
    if path != "receive":
        child = select_schema(schema, path_parts(path))
        if child.kind != "message" or set(child.children) != {"sec", "nanosec"}:
            raise ConfigError("timestamp must select a ROS Time message with sec and nanosec")


def serialize_point(measurement, tags, values, types, stamp):
    from influxdb_client import Point, WritePrecision
    if not values:
        return None
    # Use the upstream serializer, preserving explicit signed/unsigned/float types.
    return Point.from_dict({"measurement": measurement, "tags": tags, "fields": values, "time": stamp},
                           write_precision=WritePrecision.NS, field_types=types).to_line_protocol()


class Throttle:
    def __init__(self, hz):
        self.interval = 1.0 / hz if hz else 0.0
        self.next_time = None

    def accept(self, monotonic_time):
        if self.next_time is None or monotonic_time >= self.next_time:
            self.next_time = monotonic_time + self.interval
            return True
        return False
