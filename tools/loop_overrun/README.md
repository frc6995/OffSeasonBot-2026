# Loop-overrun diagnosis & iteration runbook

**Purpose:** a repeatable workflow for diagnosing `Loop time of 0.02s overrun`
warnings — run it, analyze where time is being wasted, fix the code, re-measure.
Written for agents and first-timers alike. Start here, follow it in order.

## What the warning actually means

- The warning comes from `TimedRobot`'s internal watchdog: the loop (default
  20 ms) missed its deadline. It is a **warning, not a crash** — the code keeps
  running, control is just late.
- **One overrun right at deploy/first enable is normal** (JIT compilation and
  class loading on the first pass). Ignore it.
- **Repeated overruns** — every loop, or bursts while driving — are a real
  problem. Follow this runbook.
- Do not confuse it with `...Output not updated often enough`: that is
  `MotorSafety` (separate 100 ms per-actuator timeout), not loop timing.

## The instrumentation (already in the repo)

| Piece | File | What it gives you |
|---|---|---|
| Loop timer | `src/main/java/frc/robot/util/LoopTiming.java` | `currentMs`, `minMs`, `avgMs`, `maxMs`, `overrunCount` published to NT table `/LoopTiming` every loop; min/avg/max roll over every ~5 s |
| Watchdog epochs | `CommandScheduler.run()` (WPILib, not our code) | `CommandScheduler` carries its own internal `Watchdog` (20 ms default) and **prints a per-subsystem-`periodic()`/per-command-`execute()` epoch breakdown automatically, only when it overruns** — no code here has to ask for it |
| SSH profiler | `tools/deploy-and-profile.sh` | deploys, then live-tails the roboRIO program log over SSH, filtering for overruns/errors |

**A second `Watchdog` used to be wired up by hand in `Robot.java`'s
`robotPeriodic()`, wrapping just the `CommandScheduler.run()` call as one
coarse epoch and calling `printEpochs()`/`reset()` unconditionally every
loop.** It has been removed. Two problems with it, found while reviewing this
tooling itself:

1. It was strictly *less* informative than what `CommandScheduler` already
   does internally (one blob epoch vs. a per-subsystem/per-command
   breakdown), so it added no diagnostic value.
2. Because it wasn't gated on `isExpired()`, it ran `Watchdog.reset()`
   (→ `enable()`) — a shared-queue mutex lock + a static `TreeSet`
   reinsertion — **every single loop, forever**, and printed a
   `DriverStation` warning roughly once a second regardless of whether the
   loop was actually overrunning. That's unconditional per-loop lock
   contention plus permanent log spam — exactly the kind of thing Step 3
   below tells you to hunt for. The tool meant to diagnose overruns was
   quietly contributing a small one of its own. Don't reintroduce a custom
   `Watchdog` in `Robot.java`; `CommandScheduler`'s own watchdog isn't
   reachable from outside to add epochs to, so if you need something finer
   than its per-subsystem breakdown, scope a one-off `Tracer` around the
   single suspect line you're bisecting and delete it afterward, rather than
   leaving a second always-on `Watchdog` running.

## How to run it

1. Be on the robot network (radio wifi, Ethernet, or USB — USB IP is
   `172.22.11.2`).
2. Deploy:
   ```bash
   ./gradlew deploy
   ```
3. Watch overruns live (any of these):
   ```bash
   ./tools/deploy-and-profile.sh          # deploy + tail the rio log
   # or, without redeploying:
   ssh admin@10.69.95.2 "tail -f /home/lvuser/logs/FRC_UserProgram.log"
   ```
4. Graph `/LoopTiming/*` live in **AdvantageScope** (NT → the robot). A healthy
   loop: avg well under 20 ms, max not touching 20 except at startup.
5. Capture a practice/match **data log** (`logs/*.wpilog`) — `LoopTiming` topics
   land in the log too, so post-match analysis needs no dashboard.

## How to analyze — in this order

### Step 1 — read the epoch dump
On every overrun, the console/log prints the watchdog's epoch table
(`CommandScheduler.run()` plus `TimedRobot`'s own stages). The largest number is
your culprit area. If `CommandScheduler.run()` dominates, go to Step 2/3; if
WPILib housekeeping dominates, suspect thread starvation/GC instead (Step 4).

### Step 2 — bisect inside the scheduler
If `CommandScheduler.run()` is the fat epoch, the cost is inside a subsystem's
`periodic()` or a running command. Bisect: comment out one subsystem's
`periodic()` body at a time (or use `SubsystemBase#setEnabled` / `remove`), and
watch `/LoopTiming/maxMs` change. The subsystem whose removal shrinks the loop
is your target.

### Step 3 — check the usual suspects (most common first)
| Suspect | Why it's slow | How to check |
|---|---|---|
| **A device with no CAN wire, polled through error-checking calls** | On this robot this was the confirmed #1 cause, not logging volume. `PowerDistribution`'s error-checking getters (`getVoltage`, `getAllCurrents`, `getTotalCurrent`, `getTotalPower`, `getTotalEnergy`, `getTemperature` — the non-`NoError` JNI variants) were called every loop against a PDP with **no CAN wire installed**; `CANBus.getStatus()` is separately documented as blocking up to 1 ms. Removing that polling (`PowerMonitor`) was the fix that actually moved the number; unbinding Epilogue logging, tested independently, was not measurable. | Check every device's CAN connection *first*, before profiling anything. If a getter can throw/report a CAN error, assume it can also block/retry when the device never answers. |
| CAN traffic / uncached signals | every `getX()` can be a bus read if the signal isn't cached at a set frequency | Phoenix 6: signals should be fetched via cached status signals and rated with `BaseStatusSignal.setUpdateFrequencyForAll` before `ParentDevice.optimizeBusUtilizationForAll`; check device utilization in **Phoenix Tuner's diagnostic server** (browser to `http://roborio-6995-frc.local`, port 1250) or read `CANBus.getStatus().BusUtilization`. **Already done for every `*IOTalonFX` class and the drivetrain in this repo** — see `CtreUtil.kMechanismSignalFrequencyHz`/`kCurrentSignalFrequencyHz` and each IO's constructor. Note this optimization reduces bus/background-thread load, not `refreshAll()` cost (documented non-blocking) — expect it to help bus headroom more than raw loop time. |
| `System.out.println` / dashboard spam | `println` is very expensive on the roboRIO; NT floods cost bandwidth | grep the codebase for `println`; count NT publishes per loop. **Already ruled out here**: `Epilogue.bind(this)` and `DataLogManager`'s data log are both commented out in `Robot.java`, and unbinding Epilogue was A/B tested with no measurable loop-time change — don't re-spend time on logging volume unless that wiring changes. |
| Vision processing on the RIO | AprilTag detection is heavy; it belongs on the coprocessor | is any Photon/Limelight solve running on the RIO thread? |
| Blocking calls | sleeps, waits, synchronous network fetches/CAN config-applies inside `periodic()` | code review of the bisected subsystem. `CurrentLimitManager` deliberately dispatches its hardware config-apply calls onto a background thread for exactly this reason (see its class javadoc) — a good pattern to point to if another subsystem is found doing a blocking config-apply inline. |

### Step 4 — GC / memory (the invisible cause)
Java GC pauses are the classic *intermittent* overrun that epoch tables don't
explain (the pause happens outside the measured epochs).

- Watch JVM GC in **VisualVM**: docs
  <https://docs.wpilib.org/en/stable/docs/software/advanced-gradlerio/profiling-with-visualvm.html>.
  Setup: set `debug = true` on the frcJava artifact in `build.gradle` (port
  1198), connect VisualVM over JMX (`10.69.95.2`, or `172.22.11.2` on USB),
  Sampler → CPU for hot methods, Monitor for GC frequency. **NOTE:
  `build.gradle` currently has no JVM args configured — defaults only.**
- Heap dumps are saved *on the rio* and fetched by SFTP (`scp`).
- Quick GC log on the robot: add JVM args in `build.gradle` and redeploy:
  ```groovy
  deploy.targets.roborio.artifacts.frcJava.jvmArgs.add("-Xlog:gc*:file=/home/lvuser/gc.log")
  ```
- If overruns correlate with GC bursts, the fix is **allocation churn**, not
  flags: no `new` in the 20 ms loop, no string formatting/concat for logging in
  the loop, reuse arrays/collections. WPILib's GC guide:
  <https://docs.wpilib.org/en/2024/docs/software/basic-programming/java-gc.html>
  (heap sizing, `System.gc()` every 5 s hack, USB swap — last resorts).

## Iterate

1. Fix the measured culprit (cache CAN signals, move work out of the loop, drop
   log verbosity, preallocate).
2. Re-deploy, re-run Step 2–3 of "How to run it", compare `/LoopTiming`
   before/after (and `diff` the epoch tables from two log tails).
3. Keep `overrunCount` on the dashboard during practice as a regression check.

## If something goes wrong

| Symptom | Fix |
|---|---|
| `./tools/deploy-and-profile.sh` can't connect | wrong network; try USB (`172.22.11.2`), check the DS shows a RIO link; the script falls back to `roboRIO-6995-FRC.local` |
| Log file missing over SSH | program hasn't run yet, or log rotated; `ls /home/lvuser/logs` to find the current file |
| No `/LoopTiming` topics in AdvantageScope | deployed code predates `LoopTiming.java`; redeploy |
| One overrun at enable only | normal, don't chase it |
| `grep` in the tail swallows output | the filter is `overrun\|error\|exception` — drop the grep for a raw tail when unsure |

## House rules (same as `tools/power_analysis`)

- Never put logs/reports in `src/main/deploy/` — files there are written to the
  rio's flash permanently.
- Keep logs, not reports; they regenerate in seconds.
