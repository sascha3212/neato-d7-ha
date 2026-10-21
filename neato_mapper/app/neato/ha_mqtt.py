"""Home Assistant MQTT discovery and topics for the Neato Mapper add-on."""

from __future__ import annotations

import json
import logging
from typing import Callable

import paho.mqtt.client as mqtt

from .protocol import FAN_SPEEDS, NAVIGATION_MODES, TOGGLE_SETTINGS

_LOGGER = logging.getLogger(__name__)

DISCOVERY_PREFIX = "homeassistant"


class HaMqtt:
    def __init__(self, node_id: str, name: str, version: str) -> None:
        self.node_id = node_id
        self.base = f"neato_mapper/{node_id}"
        self.availability_topic = f"{self.base}/availability"
        self.device = {
            "identifiers": [f"neato_mapper_{node_id}"],
            "name": name,
            "manufacturer": "Neato Robotics",
            "model": "Botvac (via Neato Mapper)",
            "sw_version": version,
        }
        self.client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2, client_id=f"neato-mapper-{node_id}"
        )
        self.client.will_set(self.availability_topic, "offline", retain=True)
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.on_command: Callable[[str, str], None] = lambda topic, payload: None

    # -- topics ------------------------------------------------------------

    def topic(self, suffix: str) -> str:
        return f"{self.base}/{suffix}"

    # -- connection --------------------------------------------------------

    def connect(self, host: str, port: int, username: str | None, password: str | None) -> None:
        if username:
            self.client.username_pw_set(username, password)
        self.client.connect_async(host, port, keepalive=60)
        self.client.loop_start()

    def stop(self) -> None:
        self.client.publish(self.availability_topic, "offline", retain=True)
        self.client.loop_stop()
        self.client.disconnect()

    def _on_connect(self, client, userdata, flags, reason_code, properties) -> None:
        if reason_code.is_failure:
            _LOGGER.error("MQTT connection refused: %s", reason_code)
            return
        _LOGGER.info("Connected to MQTT broker")
        client.subscribe(self.topic("command"))
        client.subscribe(self.topic("send_command"))
        client.subscribe(self.topic("set/+"))
        client.subscribe(f"{DISCOVERY_PREFIX}/status")
        self.publish_discovery()

    def _on_message(self, client, userdata, msg) -> None:
        payload = msg.payload.decode("utf-8", errors="replace")
        if msg.topic == f"{DISCOVERY_PREFIX}/status":
            if payload == "online":  # Home Assistant restarted
                self.publish_discovery()
            return
        self.on_command(msg.topic, payload)

    # -- publishing --------------------------------------------------------

    def publish(self, suffix: str, payload, retain: bool = True) -> None:
        if isinstance(payload, (dict, list)):
            payload = json.dumps(payload)
        self.client.publish(self.topic(suffix), payload, retain=retain)

    def set_available(self, available: bool) -> None:
        self.client.publish(self.availability_topic, "online" if available else "offline", retain=True)

    def _config(self, component: str, object_id: str, config: dict) -> None:
        config = {
            "unique_id": f"neato_mapper_{self.node_id}_{object_id}",
            "device": self.device,
            "availability_topic": self.availability_topic,
            **config,
        }
        topic = f"{DISCOVERY_PREFIX}/{component}/{self.node_id}/{object_id}/config"
        self.client.publish(topic, json.dumps(config), retain=True)

    def publish_discovery(self) -> None:
        self._config("vacuum", "vacuum", {
            "name": None,
            "state_topic": self.topic("state"),
            "command_topic": self.topic("command"),
            "send_command_topic": self.topic("send_command"),
            "json_attributes_topic": self.topic("attributes"),
            "set_fan_speed_topic": self.topic("set/fan_speed"),
            "fan_speed_list": FAN_SPEEDS,
            "supported_features": [
                "start", "stop", "pause", "return_home", "status", "send_command",
                "clean_spot", "locate", "fan_speed",
            ],
        })
        self._config("select", "navigation_mode", {
            "name": "Navigation mode",
            "command_topic": self.topic("set/navigation_mode"),
            "state_topic": self.topic("settings/navigation_mode"),
            "options": NAVIGATION_MODES,
            "icon": "mdi:map-marker-path",
        })
        for object_id, name, _key, _label, icon, _inverted in TOGGLE_SETTINGS:
            self._config("switch", object_id, {
                "name": name,
                "command_topic": self.topic(f"set/{object_id}"),
                "state_topic": self.topic(f"settings/{object_id}"),
                "payload_on": "ON",
                "payload_off": "OFF",
                "entity_category": "config",
                "icon": icon,
            })
        self._config("button", "drive_test", {
            "name": "Drive test",
            "command_topic": self.topic("send_command"),
            "payload_press": "drive_test",
            "entity_category": "diagnostic",
            "icon": "mdi:steering",
        })
        self._config("sensor", "drive_test", {
            "name": "Drive test",
            "state_topic": self.topic("drive/state"),
            "json_attributes_topic": self.topic("drive/attributes"),
            "entity_category": "diagnostic",
            "icon": "mdi:steering",
        })
        self._config("button", "locate", {
            "name": "Locate",
            "command_topic": self.topic("command"),
            "payload_press": "locate",
            "icon": "mdi:map-marker-question",
        })
        self._config("camera", "map", {
            "name": "Map",
            "topic": self.topic("map/image"),
            "json_attributes_topic": self.topic("map/attributes"),
        })
        self._config("sensor", "battery", {
            "name": "Battery",
            "state_topic": self.topic("battery"),
            "device_class": "battery",
            "unit_of_measurement": "%",
            "state_class": "measurement",
        })
        self._config("sensor", "robot_state", {
            "name": "Robot state",
            "state_topic": self.topic("robot_state"),
            "icon": "mdi:robot-vacuum",
        })
        self._config("sensor", "error", {
            "name": "Error",
            "state_topic": self.topic("error"),
            "icon": "mdi:alert-circle-outline",
        })
        self._config("sensor", "map_scans", {
            "name": "Map scans",
            "state_topic": self.topic("map/scans"),
            "state_class": "total_increasing",
            "entity_category": "diagnostic",
            "icon": "mdi:radar",
        })
        self._config("sensor", "raw_response", {
            "name": "Last serial response",
            "state_topic": self.topic("raw/state"),
            "json_attributes_topic": self.topic("raw/attributes"),
            "entity_category": "diagnostic",
            "icon": "mdi:console",
        })
        self._config("button", "reset_map", {
            "name": "Reset map",
            "command_topic": self.topic("send_command"),
            "payload_press": "reset_map",
            "entity_category": "config",
            "icon": "mdi:map-marker-remove",
        })
