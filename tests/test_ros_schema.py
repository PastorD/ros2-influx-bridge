import pytest
pytest.importorskip("rclpy")
pytest.importorskip("bridge_test_msgs.msg")

from bridge_test_msgs.msg import MotorTelemetry
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import NavSatFix
from ros2_influx_bridge.config import DEFAULTS, parse_fields, parse_config
from ros2_influx_bridge.fields import Extractor, message_schema, serialize_point
from ros2_influx_bridge.ros import choose_qos, validate_config, dump_config
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from types import SimpleNamespace as NS


def test_real_custom_message_nested_arrays_and_scalar_types():
    msg = MotorTelemetry()
    msg.latitude = 34.12345678912345
    msg.sequence = 2**64 - 1
    msg.label = 'escaped "quote" and slash \\'
    msg.readings = [1., 2.]
    msg.vectors = [Vector3(x=1., y=2., z=3.)]
    msg.flags = [True, False]
    msg.names = ['alpha', 'βeta']
    msg.bytes = [0, 255]
    msg.wide_label = '位置'
    msg.fixed = [3., 4.]
    fields, types, _ = Extractor(message_schema(MotorTelemetry), [], DEFAULTS["limits"]).extract(msg)
    assert fields["vectors.0.z"] == 3.
    assert fields["flags.1"] is False
    assert fields["names.1"] == 'βeta'
    assert fields["bytes.1"] == 255
    assert fields["wide_label"] == '位置'
    assert fields["fixed.1"] == 4.
    assert fields["sequence"] == 2**64 - 1 and types["sequence"] == "uint"


def test_real_gps_roundtrip_serializer():
    msg = NavSatFix(latitude=34.12345678912345, longitude=-118.12345678912345)
    fields, types, _ = Extractor(message_schema(NavSatFix), parse_fields(["latitude", "longitude"]), DEFAULTS["limits"]).extract(msg)
    line = serialize_point("gps", {}, fields, types, 1)
    assert repr(msg.latitude) in line and repr(msg.longitude) in line


def endpoint(reliability, durability=DurabilityPolicy.VOLATILE):
    return NS(node_name="publisher", node_namespace="/", qos_profile=QoSProfile(depth=10, reliability=reliability, durability=durability))


def test_auto_qos_mixed_publishers_and_explicit_mismatch():
    publishers = [endpoint(ReliabilityPolicy.RELIABLE), endpoint(ReliabilityPolicy.BEST_EFFORT)]
    config = {"reliability": "auto", "durability": "auto", "history": "keep_last", "depth": 10}
    _, resolved, _, errors = choose_qos(config, publishers)
    assert resolved["reliability"] == "best_effort" and not errors
    _, _, _, errors = choose_qos({**config, "reliability": "reliable"}, publishers)
    assert errors and "incompatible" in errors[0]


def test_transient_local_and_mixed_durability():
    config = {"reliability": "auto", "durability": "auto", "history": "keep_last", "depth": 10}
    pubs = [endpoint(ReliabilityPolicy.RELIABLE, DurabilityPolicy.TRANSIENT_LOCAL)]
    _, qos, warnings, errors = choose_qos(config, pubs)
    assert qos["durability"] == "transient_local" and not errors
    pubs.append(endpoint(ReliabilityPolicy.RELIABLE))
    _, qos, warnings, errors = choose_qos(config, pubs)
    assert qos["durability"] == "volatile" and warnings and not errors


def test_cross_topic_field_types_in_same_measurement():
    cfg = parse_config({"topics": [
        {"topic": "/a", "type": "std_msgs/msg/Float64", "measurement": "shared"},
        {"topic": "/b", "type": "std_msgs/msg/String", "measurement": "shared"}]})
    report = validate_config(cfg)
    assert not report["valid"]
    assert "field conflict" in report["topics"][1]["errors"][0]


def test_dump_disambiguates_multiple_types():
    cfg = dump_config({"/a": {"types": ["std_msgs/msg/String", "std_msgs/msg/Float64"], "publishers": []}})
    assert len({t["measurement"] for t in cfg["topics"]}) == 2
    assert len(cfg["topics"]) == 2
