from copy import deepcopy
from types import SimpleNamespace as NS
import math

import pytest

from ros2_influx_bridge.config import ConfigError, DEFAULTS, load_config, parse_config, parse_fields
from ros2_influx_bridge.fields import Schema, Extractor, DataError, Throttle, convert_scalar, serialize_point, timestamp_ns


def schema():
    return Schema("message", children={
        "latitude": Schema("scalar", "float64"),
        "temperature": Schema("scalar", "float64"),
        "sequence": Schema("scalar", "uint64"),
        "label": Schema("scalar", "string"),
        "enabled": Schema("scalar", "bool"),
        "pose": Schema("message", children={"x": Schema("scalar", "float64")}),
        "readings": Schema("array", element=Schema("scalar", "float64")),
    })


def message():
    return NS(latitude=34.12345678912345, temperature=math.nan, sequence=2**64-1,
              label='drive says "OK" \\', enabled=True, pose=NS(x=12345.678901), readings=[1.0, 2.0])


@pytest.mark.parametrize("fields", [None, []])
def test_all_fields_nested_and_arrays(fields):
    values, types, skipped = Extractor(schema(), parse_fields(fields), DEFAULTS["limits"]).extract(message())
    assert values["pose.x"] == 12345.678901
    assert values["readings.1"] == 2.0
    assert values["sequence"] == 2**64-1 and types["sequence"] == "uint"
    assert "temperature" not in values and skipped == 1


def test_field_syntax_aliases_and_indices():
    fields = parse_fields(["latitude", {"x": "pose.x"}, {"name": "second", "field": "readings.1", "type": "float64"}, {"field": "sequence", "type": "uint64"}])
    values, _, _ = Extractor(schema(), fields, DEFAULTS["limits"]).extract(message())
    assert values == {"latitude": message().latitude, "x": message().pose.x, "second": 2.0, "sequence": 2**64-1}


def test_subtree_alias_and_missing_array_element():
    values, _, skipped = Extractor(schema(), parse_fields([{"p": "pose"}, "readings.10"]), DEFAULTS["limits"]).extract(message())
    assert values == {"p.x": 12345.678901} and skipped == 1


@pytest.mark.parametrize("fields", [["missing"], ["pose.x.x"], ["readings.x"], ["pose", "pose.x"], ["readings", "readings.0"], [{"field": "label", "type": "float64"}]])
def test_bad_paths_and_output_collisions(fields):
    with pytest.raises(ConfigError):
        Extractor(schema(), parse_fields(fields), DEFAULTS["limits"])


@pytest.mark.parametrize("fields", [["latitude", "latitude"], [{"a": "latitude", "b": "sequence"}], [{"field": "latitude", "type": "decimal"}], ["pose..x"], ["__class__"]])
def test_invalid_field_config(fields):
    with pytest.raises(ConfigError):
        parse_fields(fields)


def test_float_precision_and_explicit_narrowing():
    gps = message().latitude
    assert convert_scalar(gps, "float64") == gps
    assert convert_scalar(gps, "float32") != gps
    line = serialize_point("gps", {}, {"latitude": gps}, {"latitude": "float"}, 1700000000123456789)
    assert repr(gps) in line
    assert line.endswith("1700000000123456789")


@pytest.mark.parametrize("value,dtype", [(1.5,"int64"), (-1,"uint64"), (2**64,"uint64"), (2**63,"int64"), (2**53+1,"float64"), (1e40,"float32"), ("false","bool"), (True,"float64")])
def test_reject_unsafe_conversions(value, dtype):
    with pytest.raises(DataError):
        convert_scalar(value, dtype)


def test_types_and_escaping():
    values, types, _ = Extractor(schema(), [], DEFAULTS["limits"]).extract(message())
    line = serialize_point("motor test", {"robot": "a,b=c"}, values, types, 10)
    assert 'motor\\ test,robot=a\\,b\\=c' in line
    assert '18446744073709551615u' in line
    assert 'enabled=true' in line
    assert 'label="drive says \\"OK\\" \\\\"' in line


@pytest.mark.parametrize("value", ["a\nb", "a\rb", "a\x00b"])
def test_reject_multiline_strings(value):
    with pytest.raises(DataError):
        convert_scalar(value, "string")


@pytest.mark.parametrize("key,limit", [("max_fields", 2), ("max_array_length", 1), ("max_depth", 1)])
def test_limits_reject_instead_of_truncate(key, limit):
    limits = {**DEFAULTS["limits"], key: limit}
    with pytest.raises(DataError):
        Extractor(schema(), [], limits).extract(message())


def test_empty_array_and_all_nonfinite():
    msg = message(); msg.readings = []
    values, _, skipped = Extractor(schema(), parse_fields(["temperature", "readings"]), DEFAULTS["limits"]).extract(msg)
    assert values == {} and skipped == 1
    assert serialize_point("empty", {}, {}, {}, 1) is None


def test_timestamp_and_monotonic_throttle():
    msg = NS(header=NS(stamp=NS(sec=17, nanosec=123)))
    assert timestamp_ns(msg, "header.stamp", 99) == 17000000123
    assert timestamp_ns(msg, "receive", 99) == 99
    t = Throttle(10)
    assert t.accept(0)
    assert not t.accept(.099)
    assert t.accept(.1)
    assert not t.accept(.11)
    unlimited = Throttle(0)
    assert all(unlimited.accept(1) for _ in range(10))


def test_configuration_defaults():
    config = parse_config({"topics": [{"topic": "/custom"}]})
    assert config["topics"][0]["fields"] == []
    assert config["topics"][0]["qos"]["reliability"] == "auto"


@pytest.mark.parametrize("raw", [None, {}, {"version": 2}, {"typo": 1}, {"topics": "bad"},
    {"topics": [{"topic": "relative"}]}, {"topics": [{"topic": "/x", "throttle_hz": -1}]},
    {"topics": [{"topic": "/x", "qos": {"reliability": "fast"}}]},
    {"influxdb": {"url": "file:///tmp/x"}}, {"tags": {"topic": "override"}},
    {"writer": {"queue_size": 1}}, {"limits": {"max_fields": 0}},
    {"writer": {"overflow": []}}, {"topics": [{"topic": "/x", "qos": {"reliability": []}}]},
    {"influxdb": {"url": "http://localhost:bad"}}])
def test_bad_config(raw):
    if raw == {}:
        assert parse_config(raw)["topics"] == []
    else:
        with pytest.raises(ConfigError):
            parse_config(raw)


def test_duplicate_yaml_keys(tmp_path):
    path = tmp_path / "duplicate.yaml"
    path.write_text("topics: []\ntopics: []\n")
    with pytest.raises(ConfigError, match="Duplicate"):
        load_config(path)
