"""Entry point. Reads the add-on options (/data/options.json) or environment."""

from __future__ import annotations

import argparse
import asyncio
import json
import logging
import os
from pathlib import Path

from . import __version__
from .mapper import MapperConfig
from .service import NeatoService, Options


def load_options(path: Path) -> Options:
    raw = json.loads(path.read_text()) if path.exists() else {}

    def opt(key: str, env: str, default=None):
        value = raw.get(key)
        if value in (None, ""):
            value = os.environ.get(env, default)
        return value

    host = opt("bridge_host", "NEATO_BRIDGE_HOST")
    if not host:
        raise SystemExit("bridge_host is not set: enter the ESP32 bridge's IP or hostname")
    mapper = MapperConfig(
        resolution_m=float(opt("map_resolution_cm", "NEATO_MAP_RESOLUTION_CM", 3)) / 100,
        lds_offset_m=float(opt("lds_offset_mm", "NEATO_LDS_OFFSET_MM", 0)) / 1000,
        lds_angle_offset_deg=float(opt("lds_angle_offset_deg", "NEATO_LDS_ANGLE_OFFSET_DEG", 0)),
        lds_clockwise=str(opt("lds_clockwise", "NEATO_LDS_CLOCKWISE", False)).lower() == "true",
    )
    return Options(
        bridge_host=host,
        bridge_port=int(opt("bridge_port", "NEATO_BRIDGE_PORT", 6638)),
        name=opt("name", "NEATO_NAME", "Neato"),
        node_id=opt("node_id", "NEATO_NODE_ID", "neato"),
        data_dir=Path(opt("data_dir", "NEATO_DATA_DIR", "/data")),
        mqtt_host=opt("mqtt_host", "MQTT_HOST", "core-mosquitto"),
        mqtt_port=int(opt("mqtt_port", "MQTT_PORT", 1883)),
        mqtt_username=opt("mqtt_username", "MQTT_USERNAME"),
        mqtt_password=opt("mqtt_password", "MQTT_PASSWORD"),
        mapper=mapper,
    )


def main() -> None:
    parser = argparse.ArgumentParser(prog="neato-mapper")
    parser.add_argument("--options", default="/data/options.json", type=Path)
    parser.add_argument("--debug", action="store_true")
    args = parser.parse_args()
    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    logging.getLogger("neato").info("Neato Mapper %s starting", __version__)
    asyncio.run(NeatoService(load_options(args.options), __version__).run())


if __name__ == "__main__":
    main()
