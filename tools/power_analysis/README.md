# Power analysis

Where a match's current went, for tracking down brownouts.

The robot logs every Talon FX's supply current and supply voltage once per loop into its
`.wpilog`, through Epilogue's file backend (`PowerLogging`). IO layers register their motors with
`PowerLogging.register`, which also keeps those two signals on the CAN bus at 20 Hz. Each value is
written only when it changes.

## 1. Logging on or off

`Robot.kLoggingEnabled` in the robot code. When true, DataLogManager starts with the robot program
and logs until it exits. Change it and redeploy to turn logging off.

## 2. Get the file

DataLogManager writes one `FRC_*.wpilog` per robot program run: to a USB stick at `/u/logs` if one
is plugged in, otherwise to `/home/lvuser/logs`. On the field the name includes the event and
match. AdvantageScope can download them (File > Download Logs).

- A log stays named `FRC_TBD_...` until the Driver Station connects, and DataLogManager deletes
  `FRC_TBD` files the next time the robot program starts. Connect the Driver Station before running
  anything you want to keep.
- When free space drops below 50 MB, DataLogManager deletes the oldest `FRC_` logs. It does not
  touch old `.hoot` files, so clear those out of `/home/lvuser/logs` by hand.

## 3. Open the dashboard

Open [`dashboard.html`](dashboard.html) in a browser (double-click works; nothing is uploaded) or
use the published copy: https://claude.ai/artifact/97BbKZKZ2U4QkkPaQfgfBe

Drop the match's `.wpilog` onto it. It shows:

- **Summary**: a pie chart of energy per subsystem with Wh and share, and P50/P90/P99/peak current while
  each one was running. The subsystems are Drive, Steer, Flywheel, Intake roller, Intake
  extension, Intake kicker, Dye rotor hook (spin), Dye rotor index and Turret & Hood. Drive,
  Flywheel and Intake roller get extra rows per state (from the `Robot State`, `Flywheel/State` and
  `Intake/State` entries Epilogue logs), so `Drive · SCORING` shows whether the rule in
  `RobotCurrentLimits` actually cut drive current.
- **Timeline**: battery voltage over stacked current by subsystem, sags shaded. Drag to zoom.
- **Sags**: every dip below the sag threshold, worst first, with what each subsystem drew during it
  and in the half second before.
- **Motors**: every Talon by CAN ID, with a check for missing motors and slow signals. Rate is new
  values per second in the log, so 50 Hz (the loop) is the most it can show.

**Copy summary** puts a plain-text table on the clipboard for comparing matches.

The motor-to-subsystem map (CAN IDs) is at the top of the dashboard's script. Update it there if
the wiring changes.

Hoot logs from before this change still work: convert each bus's `.hoot` with CTRE's owlet
(`owlet UpperBus.hoot UpperBus.wpilog -f wpilog -e 5`) and drop all of a match's files together.

## Reading it

- Battery voltage is the median of what every Talon reports at its input. It is what the motors
  had to work with, and it includes wiring drop, so it reads a little below the roboRIO's figure.
- Totals are motor supply current only. The roboRIO, radio and cameras add roughly 5-10 A.
- To compare two matches, look at sag count and depth and the P90/P99 columns. Total Wh barely
  moves with current limits: a limit slows the work down rather than skipping it.

## A/B testing loop time

Deploy with `Robot.kLoggingEnabled = false`, run a routine, then deploy with it true and run the
same routine again. Keep Phoenix Tuner X closed for both runs (it adds its own CAN and CPU load),
and compare `LoopTiming/maxMs` and `LoopTiming/overrunCount`. The Epilogue update runs inside the
timed part of the loop, so its cost shows up there; the log also records it on its own as
`Epilogue/Stats/Last Run` (ms).

## Files

| File | |
|---|---|
| `dashboard.html` | the analysis, in one self-contained page |
