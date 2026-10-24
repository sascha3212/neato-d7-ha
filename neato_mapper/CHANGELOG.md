# Changelog

## 0.5.0

What the old fang automations used, so they can move over:

- "Last clean" sensor (timestamp of when the last clean started), kept
  across restarts, for "once a day" automations
- "Clear errors" button (`SetUIError clearall`)
- "Restart robot" button: power-cycles the robot like fang's PowerCycle

## 0.4.3

After the robot's serial port got out of step during the second Explore run:

- An answer that belongs to another command is no longer fatal: the add-on
  skips leftovers in front of its own echo, or waits for the line to go
  quiet, throws away what's there and asks again. It logs what came back.
- Fewer commands while driving: bumpers every 0.5 s (was 0.3 s), cliff
  sensors every other poll, a pause after each lidar scan, 150 ms between
  commands
- Explore attributes are updated even when a run fails

## 0.4.2

- Explore and Drive test read the dock state fresh, so they no longer refuse
  right after the robot was put back on its dock by hand

## 0.4.1

After the first real Explore run gave up after 4 minutes in a furnished room:

- Drives as far as the lidar shows room (at least 8 cm) instead of skipping
  every goal without half a metre of clear floor
- Goals skipped for lack of room are tried again before calling it done
- The way back no longer stops at the obstacle in front of the dock: the
  last 60 cm go straight in (bumpers watching), and a bump there means back
  off and try again

## 0.4.0

- **Explore** button: the robot maps the house without cleaning. Starting
  from the dock it drives (in TestMode, brush and vacuum off) to the nearest
  edge of what it hasn't seen yet, scans, and repeats; then it reverses back
  onto the dock. Each scan is matched against the map so far to cancel
  odometry drift. Bumpers, lifted wheels and cliff sensors stop every move;
  things only the bumper found are remembered and avoided. Stops after 40
  minutes, below 25% battery, or when you press stop (stays where it is) or
  return to base (drives back). Progress on the "Explore" sensor.
- Straight moves check the lidar in a strip as wide as the robot

## 0.3.2

- Every answer is checked against the robot's echo of the command; on a
  mismatch the add-on reconnects instead of reading answers one behind
- Drive test goes out of the dock as far as the lidar allows (12-30 cm)

## 0.3.1

- Drive test reverses the 30 cm back into the dock itself and checks the
  charging contacts, instead of "send to base": the robot only accepts that
  during a clean, and only finds its dock when the clean started there

## 0.3.0

- "Drive test" button (diagnostic): the robot drives 30 cm out of the dock in
  TestMode and turns a full circle in four steps, with brush and vacuum off,
  then goes back to the dock. It records position, wheel odometry and lidar at
  each step, to find out if a mapping drive without cleaning can work.
  Every move is checked first and watched while it runs: bumpers, lifted
  wheels, cliff sensors and lidar clearance stop it.
- A robot left in TestMode is switched back automatically

## 0.2.2

- Fix: between two scans the add-on thought the clean had ended, saved the
  whole map every few seconds and got fewer scans than intended
- `vacuum.send_command` with `raw` accepts params the way Home Assistant sends them

## 0.2.1

- Gentler on the robot while it cleans: one lidar scan every 4 s (was back
  to back) and a short pause between serial commands. A D7 on 4.4.0 stopped
  answering on serial after about a minute of continuous scans.
- Scans slow down further (up to every 30 s) after a failed command
- Shows the D7's model and software version correctly in the log

## 0.2.0

- Cleaning power as the vacuum's fan speed: eco or turbo
- Navigation mode select (Normal, Gentle, Deep, Quick), remembered by the
  add-on and sent before every house clean because the robot forgets it
- Switches for intense clean, edge cleaning, bin full detection, button
  sounds, melodies, warning sounds and status lights
- Locate: the robot plays its alert sound

## 0.1.1

- Sharper map: 3 cm cells (was 5 cm), 1 cm per pixel output
- Smooth anti-aliased walls and floor, speckles and pinholes cleaned up
- Fewer holes in walls from long grazing lidar rays
- 1 m grid, wall shadows, smoothed path and a robot-sized marker

## 0.1.0

- First version: lidar map from normal cleans, shown as a camera entity
- Vacuum entity with start, pause, stop, spot clean and return to base
- Battery, robot state, error and raw serial diagnostics
