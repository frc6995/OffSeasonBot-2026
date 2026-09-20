# Loop-overrun test plan (agent + on-robot)

Companion to [README.md](README.md). This splits the work into what an agent
can verify from source alone (no robot required) and what only makes sense
run against real hardware, ordered by what this investigation has already
established is most likely to matter.

## Priors going into this test (don't re-derive these)

- **Confirmed root cause on this robot**: `PowerMonitor` polling a
  `PowerDistribution` with no CAN wire installed via error-checking JNI
  getters — removing it produced the one *measured* improvement.
- **Confirmed non-cause**: telemetry/logging volume. Unbinding
  `Epilogue.bind(this)` was A/B tested independently — no measurable change.
  `Epilogue.bind` and `DataLogManager`'s data log are already commented out.
- **Already applied, expected effect modest and on bus headroom, not loop
  time**: `optimizeBusUtilization` + explicit signal rates on every
  `*IOTalonFX` class and the drivetrain (`CtreUtil.kMechanismSignalFrequencyHz`
  / `kCurrentSignalFrequencyHz`). Measure via `CANBus.getStatus().BusUtilization`,
  not `/LoopTiming`, or you'll conclude it "did nothing" when it worked.
- **Fixed in this pass**: a second hand-rolled `Watchdog` in `Robot.java`
  that ran unconditional per-loop lock/`TreeSet` work and printed a warning
  every second regardless of overrun state. Removed; `CommandScheduler`'s own
  watchdog already gives the epoch breakdown.
- **Still wired, not yet a suspect**: `CurrentLimitManager` — reviewed here
  and its hardware pushes are already dispatched off the main thread, so it's
  a low-priority bisection target, not a high one.

## Phase 1 — static verification (agent, no robot, do this first)

Confirms nothing has regressed since the above was established, and that the
instrumentation itself is trustworthy before trusting its numbers.

1. **Compile check.**
   ```bash
   ./gradlew compileJava --offline
   ```
   Expect a clean pass (pre-existing photonlib `[removal]` deprecation
   warnings are fine; anything else is new and must be explained).

2. **Confirm logging is still off** (regression check — someone could have
   uncommented it since):
   ```bash
   grep -n "Epilogue.bind\|startDataLog" src/main/java/frc/robot/Robot.java
   ```
   Both lines should still be commented out. If someone re-enabled them,
   that's a legitimate new variable to control for before trusting any new
   loop-time measurement.

3. **Confirm no new blocking call was added to a `periodic()`.** Grep for the
   error-checking `PowerDistribution` getters and any new device reads that
   don't go through a cached, rated `StatusSignal`:
   ```bash
   grep -rn "PowerDistribution" src/main/java
   grep -rln "class .*IOTalonFX" src/main/java | xargs grep -L "optimizeBusUtilizationForAll"
   ```
   The second command should print nothing — every `*IOTalonFX` class should
   have the optimize call. If a new one is missing it, that device's
   unrated signals are silently running at CAN FD defaults, which is
   fine, but any *new* frequently-polled getter added elsewhere (not just
   TalonFX) is worth a manual look the way `PowerMonitor` was.

4. **Confirm the instrumentation compiles and only adds one watchdog.**
   ```bash
   grep -n "new Watchdog" src/main/java/frc/robot/Robot.java
   ```
   Should print nothing (removed in this pass). If it's back, re-read the
   README section on why it was removed before keeping it.

5. **Diff review.** Read the diff of this pass's changes
   (`git diff` against the branch's base) end-to-end before deploying
   anything — a static review catches most instrumentation bugs cheaper than
   a robot deploy does.

## Phase 2 — on-robot measurement (needs physical access; hand off to whoever has it)

Only meaningful once Phase 1 passes. Requires the robot, its radio/USB link,
and the PDP's CAN wire actually connected (verify this first — it's the
single highest-leverage check given the confirmed root cause above).

1. **Verify PDP CAN connectivity before anything else.** Check
   `Phoenix Tuner X` or the PDP's own status LED — a disconnected PDP is what
   caused the original overruns. Don't spend time profiling until this is
   ruled in or out.

2. **Deploy and capture a baseline.**
   ```bash
   ./gradlew deploy
   ./tools/deploy-and-profile.sh
   ```
   Let it idle in `disabled`, then run a normal teleop period exercising
   every subsystem (drive, turret, hood, flywheel, dye rotor) — the intake is
   still not physically attached on this branch, skip it.

3. **Record three numbers, not just one:**
   - `/LoopTiming/maxMs` and `overrunCount` (AdvantageScope, or the `.wpilog`)
     — the overrun metric this whole runbook exists to fix.
   - `CANBus.getStatus().BusUtilization` for each bus in use — this is what
     the `optimizeBusUtilization` change actually targets; expect an
     improvement here even if `/LoopTiming` barely moves.
   - The `CommandScheduler` epoch dump on any overrun that does occur (DS
     console / log) — read it before touching code; it names the slow
     subsystem or command directly.

4. **If overruns persist**, bisect per README Step 2 in this order (highest
   prior first, not alphabetical):
   1. Whichever subsystem the epoch dump actually names.
   2. `CurrentLimitManager` (low prior, but easy to rule out —
      `setEnabled(false)` in `RobotContainer` and redeploy).
   3. Vision (`RealATPhotonVision`/Limelight) — confirm no solve runs on the
      RIO thread, only NT reads of a coprocessor's results.
   4. GC (README Step 4) — only if the epoch dump is unremarkable but
      overruns are intermittent; that mismatch is GC's signature.

5. **Regression-proof it.** Once a fix lands, leave `/LoopTiming/overrunCount`
   on a dashboard tab for the next few practice runs rather than declaring
   victory off one deploy — the PDP wiring issue in particular is the kind of
   thing that can silently reappear (wire vibrates loose) and look like a
   code regression if nobody's watching for it.

## What "done" looks like

- Phase 1 checks all pass with no unexplained diffs.
- Phase 2 shows `overrunCount` flat (ideally zero) across a full teleop
  period with every attached subsystem active, PDP connected.
- `CANBus.getStatus().BusUtilization` recorded as a baseline for future
  comparisons, independent of whether loop time moved.
