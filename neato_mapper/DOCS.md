# Neato Mapper

Lidar maps and local control for Neato Botvac D3-D7 robots through an ESP32
running the `neato-bridge` ESPHome config from this repository.

## Options

| Option | Default | What it does |
| --- | --- | --- |
| ESP32 bridge address | (required) | IP or hostname of the ESP32 bridge |
| Bridge port | 6638 | TCP port of the ESPHome stream server |
| Robot name | Neato | Device name in Home Assistant |
| Map resolution (cm) | 3 | Size of one map cell; changing it starts a new map |
| Lidar offset (mm) | 0 | Turret position ahead of the centre (negative = behind) |
| Lidar angle offset | 0 | Rotation correction for the lidar, in degrees |
| Lidar angles clockwise | off | Turn on if the map looks mirrored |
| Debug logging | off | Log every step |

## Entities

- `vacuum.neato`: start, pause, stop, spot clean, return to base, send_command
- `camera.neato_map`: the map, with `calibration_points` for the Xiaomi Vacuum Map Card
- `sensor.neato_battery`, `sensor.neato_robot_state`, `sensor.neato_error`
- `button.neato_reset_map`, `sensor.neato_map_scans`, `sensor.neato_last_serial_response`

## Commands

`vacuum.send_command` accepts:

- `reset_map`: forget the map and start again
- `raw` with params `{"cmd": "<serial command>"}`: send any serial command; the
  answer appears on the "Last serial response" sensor

The map is saved in the add-on's data folder and survives restarts and updates.
See the repository README for setup and first-run checks.
