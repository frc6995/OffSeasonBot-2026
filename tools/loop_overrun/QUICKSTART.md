# Loop overrun — quick reference

Full runbook in [README.md](README.md); step-by-step test procedure in
[TEST_PLAN.md](TEST_PLAN.md). One-time setup is already done
(`LoopTiming.java` is in the code; the per-subsystem/per-command epoch
breakdown on overrun comes free from `CommandScheduler`'s own watchdog, no
extra wiring needed).

## Run

```bash
./gradlew deploy                  # from repo root, on the robot network
./tools/deploy-and-profile.sh     # then watch overruns live over SSH
```

Or graph NT table `/LoopTiming` (`currentMs`, `avgMs`, `maxMs`, `overrunCount`)
in AdvantageScope for live plots.

## Analyze, in order

1. **Epoch table** — printed automatically in the log on every overrun; biggest
   number = culprit area.
2. **`CommandScheduler.run()` fat?** → bisect subsystem `periodic()`s, watch
   `/LoopTiming/maxMs`.
3. **Usual suspects** — `println` spam, uncached CAN reads, vision on the RIO,
   blocking calls.
4. **GC** — intermittent overruns with clean epochs: VisualVM (JMX, `debug =
   true` in `build.gradle`) or a GC log (`-Xlog:gc*`); fix allocation churn.

## Iterate

Fix → redeploy → compare `/LoopTiming` before/after. One overrun at first
enable is normal (JIT); bursts while driving are not.
