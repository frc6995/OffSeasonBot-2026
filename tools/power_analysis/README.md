# Power analysis

Where a match's current went, for tracking down brownouts.

Power data is recorded by CTRE's hoot logger, not by robot code. Phoenix writes every Talon FX's
supply current and supply voltage into a `.hoot` file as the frames arrive on the CAN bus, in its
own thread, so logging costs the robot loop nothing. The robot code's only jobs are keeping those
two signals on the bus at 20 Hz (`CtreUtil.setPowerSignalFrequency`) and starting the logger
(`HootLogging`).

## 1. Logging on or off

`HootLogging.kEnabled` in the robot code. When true, logging starts with the robot program and
runs until it exits. Change it and redeploy to turn logging off.

## 2. Get the files

Logs go to a USB stick at `/u/logs` if one is plugged in, otherwise to `/home/lvuser/logs`. Each
robot program start makes a dated folder with one `.hoot` file per CAN bus (LowerBus, UpperBus).

## 3. Convert with owlet

The dashboard reads `.wpilog`, so convert each `.hoot` with CTRE's owlet. It ships inside Phoenix
Tuner X and AdvantageScope (on macOS, `~/Library/Application Support/AdvantageScope/owlet/`; use
the version that matches Phoenix, currently 26.3.0).

```bash
owlet UpperBus.hoot UpperBus.wpilog -f wpilog -e 5
```

`-e 5` keeps only the enabled part of the log plus 5 s either side. Without it the export holds
every signal from power-on, which gets large.

## 4. Open the dashboard

Open [`dashboard.html`](dashboard.html) in a browser (double-click works; nothing is uploaded) or
use the published copy: https://claude.ai/artifact/97BbKZKZ2U4QkkPaQfgfBe

Drop all of a match's `.wpilog` files onto it at once. It shows:

- **Summary**: energy per subsystem, and P50/P90/P99/peak current while each one was running.
  Drive, Flywheel and Intake get extra rows per state (from the `Robot State`, `Flywheel/State` and
  `Intake/State` signals `HootLogging` writes), so `Drive · SCORING` shows whether the rule in
  `RobotCurrentLimits` actually cut drive current.
- **Timeline**: battery voltage over stacked current by subsystem, sags shaded. Drag to zoom.
- **Sags**: every dip below the sag threshold, worst first, with what each subsystem drew during it
  and in the half second before.
- **Motors**: every Talon by CAN ID, with a check for missing motors and slow signals.

**Copy summary** puts a plain-text table on the clipboard for comparing matches.

The motor-to-subsystem map (CAN IDs) is at the top of the dashboard's script. Update it there if
the wiring changes.

## Reading it

- Battery voltage is the median of what every Talon reports at its input. It is what the motors
  had to work with, and it includes wiring drop, so it reads a little below the roboRIO's figure.
- Totals are motor supply current only. The roboRIO, radio and cameras add roughly 5-10 A.
- To compare two matches, look at sag count and depth and the P90/P99 columns. Total Wh barely
  moves with current limits: a limit slows the work down rather than skipping it.

## A/B testing loop time

Deploy with `HootLogging.kEnabled = false`, run a routine, then deploy with it true and run the
same routine again. Keep Phoenix Tuner X closed for both runs (it adds its own CAN and CPU load),
and compare `LoopTiming/maxMs` and `LoopTiming/overrunCount`.

## Files

| File | |
|---|---|
| `dashboard.html` | the analysis, in one self-contained page |
