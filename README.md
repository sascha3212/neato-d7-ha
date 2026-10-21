# Neato D7 for Home Assistant

Local control and lidar maps for Neato Botvac D3-D7 robots, now that Neato's
cloud is gone. An ESP32 inside the robot passes the serial debug port to Home
Assistant over Wi-Fi, and the **Neato Mapper** add-on does the rest.

![Map built in the simulator](docs/simulated-map.png)

*A map built by the add-on from a simulated two-room clean: walls from the
lidar, the cleaning path in blue, the dock in green and the robot in orange.*

## What works

- A floor map that builds up from the robot's lidar during every normal clean
  and is shown in Home Assistant as `camera.neato_map`
- A vacuum entity: start, pause, resume, stop, spot clean, return to base,
  locate, and eco / turbo power
- Navigation mode (normal, gentle, deep, quick) and switches for intense
  clean, edge cleaning, sounds, status lights and bin full detection
- Schedules through Home Assistant automations (example in the add-on docs)
- **Explore**: maps the whole house without cleaning, driving to unexplored
  places by itself and reversing back onto the dock
- Battery, robot state and error sensors
- A "Reset map" button and a raw serial command for diagnostics

Coming next: rooms ("clean the kitchen") in stage 2, and no-go lines in stage 3.

## How it fits together

```
Neato D7 --serial--> ESP32 (ESPHome stream server) --Wi-Fi/TCP--> Neato Mapper add-on --MQTT--> Home Assistant
```

The ESP32 is only a pipe; it has too little memory for mapping. The add-on
asks the robot for its own SLAM position (`GetRobotPos Smooth`) and a lidar
scan (`GetLDSScan`) a couple of times a second while it cleans, and votes each
scan into an occupancy grid. Every clean refines the same map.

## Install

You need Home Assistant OS or Supervised (for add-ons), the Mosquitto broker
add-on and the MQTT integration.

1. **Put the bridge on the ESP32, over Wi-Fi.** No need to take the ESP out:
   fang is ESPHome, so it accepts new firmware over the air. See
   [Install over Wi-Fi](#install-over-wi-fi) below. This replaces fang on the
   ESP; you can put fang back the same way at any time.
2. **Add this repository** in Settings > Add-ons > Add-on store > ⋮ >
   Repositories: `https://github.com/sascha3212/neato-d7-ha`
3. **Install Neato Mapper**, set *ESP32 bridge address* to the ESP's IP or
   hostname, and start it.
4. The robot appears under Settings > Devices > MQTT as "Neato".

## Install over Wi-Fi

Pick whichever matches how you installed fang.

**fang was installed from the ESPHome dashboard in Home Assistant**

1. In the ESPHome dashboard, open fang's device (`neato-vacuum`), click Edit,
   and replace the whole config with
   [`esphome/neato-bridge.yaml`](esphome/neato-bridge.yaml).
2. In that file: set the pins and `variant` to match fang's config for your
   board, uncomment `use_address` under `wifi` and put the ESP's current IP
   there, and make `neato_bridge_ota` in your ESPHome secrets the same password
   as fang's `neato_vacuum_ota`.
3. Click Install > Wirelessly. When it's done the ESP shows up as "Neato
   Bridge"; you can then comment out `use_address` again and remove the old
   fang device from Home Assistant.

**fang was installed with the web flasher (no ESPHome dashboard)**

1. Build the bridge firmware once: in the ESPHome dashboard (or ESPHome on a
   computer) create a device from `esphome/neato-bridge.yaml`, then Install >
   Manual download > "OTA format" to get a `.bin` file.
2. Open fang's web page (`http://neato-vacuum.local` or the ESP's IP), find
   the OTA section, upload the `.bin` and press Update.

**Safety net:** if the new firmware ever fails to start or join Wi-Fi a few
times in a row, ESPHome's safe mode keeps the ESP reachable for another OTA
upload. If Wi-Fi credentials are wrong, it opens a `neato-bridge-fallback`
hotspot where you can enter them. In both cases the ESP stays where it is.

## Show the map

The camera entity works in a picture card straight away. For a nicer view, and
for drawing rooms later, install the
[Xiaomi Vacuum Map Card](https://github.com/PiotrMachowski/lovelace-xiaomi-vacuum-map-card)
from HACS (it works with any vacuum) and use:

```yaml
type: custom:xiaomi-vacuum-map-card
entity: vacuum.neato
map_source:
  camera: camera.neato_map
calibration_source:
  camera: true
```

The add-on publishes `calibration_points` on the camera, in millimetres with
the dock at (0, 0).

## First run checks

These depend on your robot's firmware and could not be tested without one, so
please check them on the first clean:

1. **Position format.** Call `vacuum.send_command` with command `raw` and
   params `{"cmd": "GetRobotPos Smooth"}`. The "Last serial response" sensor's
   attributes show the answer. It should contain `X=`, `Y=`, `Theta=`, `Time=`.
   If the map stays empty, this is the first thing to look at.
2. **Walls line up.** After a clean, walls should be single crisp lines. If
   the map is mirrored, turn on *Lidar angles clockwise*. If walls are doubled
   or smeared, try *Lidar offset* (the turret sits behind the centre on most
   Botvacs, try -90 mm) or a small *Lidar angle offset*.
3. **Same map each time.** The robot starts every clean at the dock, so its
   coordinates should line up from one clean to the next. If a second clean
   draws a rotated copy, press "Reset map" and tell me what you see.

## Development

```bash
pip install -r neato_mapper/requirements.txt pytest
pytest -q tests
python -m tests.fake_robot --port 6638   # a simulated robot to point the add-on at
```

## Credits

- [fang](https://github.com/vacuula/fang) for the serial research and ESP32 wiring
- [OpenNeato](https://github.com/renjfk/OpenNeato) (MIT) for the serial
  protocol notes and the `SetEvent` key algorithm, ported to Python here
- [esphome-stream-server](https://github.com/oxan/esphome-stream-server)
