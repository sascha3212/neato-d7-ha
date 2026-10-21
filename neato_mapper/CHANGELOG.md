# Changelog

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
