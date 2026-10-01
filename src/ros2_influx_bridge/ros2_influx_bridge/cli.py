"""run, dump and validate entry points."""
import argparse
import json
import logging
from pathlib import Path
import sys
import uuid

import yaml

from .config import ConfigError, load_config


def parser():
    p = argparse.ArgumentParser(description="ROS 2 → InfluxDB 2 telemetry bridge")
    commands = p.add_subparsers(dest="command", required=True)
    run = commands.add_parser("run", help="Forward configured topics")
    run.add_argument("--config", required=True)
    run.add_argument("--stats-file", help="Write final counters as JSON on orderly shutdown")
    run.add_argument("--discovery-seconds", type=float, default=3.0)
    dump = commands.add_parser("dump", help="Export all visible topic/type pairs to YAML")
    dump.add_argument("--output", required=True, help="Output YAML, or - for stdout")
    dump.add_argument("--force", action="store_true")
    dump.add_argument("--discovery-seconds", type=float, default=3.0)
    validate = commands.add_parser("validate", help="Check config/schema and current graph without database writes")
    validate.add_argument("--config", required=True)
    validate.add_argument("--offline", action="store_true", help="Skip graph checks; installed message packages still required")
    validate.add_argument("--json", action="store_true")
    validate.add_argument("--discovery-seconds", type=float, default=3.0)
    return p


def print_report(report, as_json):
    if as_json:
        print(json.dumps(report, indent=2))
        return
    print("VALID" if report["valid"] else "INVALID", "(" + report["mode"] + ")")
    for item in report["topics"]:
        print(f"  {item['topic']} [{item['type']}]: {'ERROR' if item['errors'] else 'OK'}")
        for message in item["errors"]:
            print("    ERROR:", message)
        for message in item["warnings"]:
            print("    WARNING:", message)
    for message in report["warnings"]:
        print("  WARNING:", message)


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    # Keep pure --help/config-error paths usable without ROS installed.
    ros_args = []
    if "--ros-args" in argv:
        split = argv.index("--ros-args")
        argv, ros_args = argv[:split], argv[split:]
    args = parser().parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s", stream=sys.stderr)
    log = logging.getLogger("ros2_influx_bridge")
    node = runtime = writer = None
    initialized = False
    exit_code = 0
    try:
        if not 0 <= args.discovery_seconds <= 300:
            raise ConfigError("discovery-seconds must be between 0 and 300")
        config = load_config(args.config) if hasattr(args, "config") else None
        if args.command == "dump" and args.output != "-" and Path(args.output).exists() and not args.force:
            raise ConfigError("Output exists; use --force to replace it")
        from .ros import discover, dump_config, validate_config, BridgeRuntime
        if args.command == "validate" and args.offline:
            report = validate_config(config)
            print_report(report, args.json)
            return 0 if report["valid"] else 2
        import rclpy
        rclpy.init(args=ros_args)
        initialized = True
        node = rclpy.create_node("ros2_influx_bridge_" + uuid.uuid4().hex[:8], enable_rosout=False, start_parameter_services=False)
        graph = discover(node, args.discovery_seconds)
        if args.command == "dump":
            content = "# Snapshot of all visible topics, including hidden/system topics. Review large arrays before running.\n" + yaml.safe_dump(dump_config(graph), sort_keys=False)
            if args.output == "-":
                print(content, end="")
            else:
                # Exclusive creation closes the overwrite race for the default mode.
                with open(args.output, "w" if args.force else "x", encoding="utf-8") as f:
                    f.write(content)
            log.info("Exported %d topic/type pairs", sum(len(x["types"]) for x in graph.values()))
        elif args.command == "validate":
            report = validate_config(config, graph)
            print_report(report, args.json)
            exit_code = 0 if report["valid"] else 2
        else:
            # Reject invalid installed schemas before creating a writer. Missing
            # publishers/types can be resolved by periodic discovery during run.
            initial = validate_config(config, graph)
            schema_errors = [e for item in initial["topics"] for e in item["errors"] if e.startswith("Message schema/")]
            if schema_errors:
                raise ConfigError("; ".join(schema_errors))
            from .writer import BatchWriter, InfluxSink
            writer = BatchWriter(config["writer"], InfluxSink(config["influxdb"]))
            runtime = BridgeRuntime(node, config, writer)
            log.info("Running with volatile buffering; pending records do not survive process termination")
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except (ConfigError, ValueError, ImportError, OSError) as exc:
        log.error("%s", exc)
        exit_code = 2
    except Exception as exc:
        # ROS signal handling may raise ExternalShutdownException rather than KeyboardInterrupt.
        if type(exc).__name__ != "ExternalShutdownException":
            log.exception("Bridge failure")
            exit_code = 1
    finally:
        if writer is not None:
            final_writer = writer.close()
            report = runtime.snapshot() if runtime is not None else {"writer": final_writer}
            log.info("FINAL_STATS %s", json.dumps(report, sort_keys=True))
            if final_writer["pending"]:
                log.warning("%d unsent records remain in volatile memory at shutdown", final_writer["pending"])
            if getattr(args, "stats_file", None):
                Path(args.stats_file).write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        if node is not None:
            node.destroy_node()
        if initialized:
            import rclpy
            if rclpy.ok():
                rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
