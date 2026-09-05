# Power analysis

Offline breakdown of where a match's current went, for tracking down brownouts.

```
python3 tools/power_analysis/analyze_power.py logs/FRC_20260101_120000.wpilog
python3 tools/power_analysis/analyze_power.py <log> --out report/ --csv
```

No install needed for the tables. Plots want matplotlib:
`pip install -r tools/power_analysis/requirements.txt`

## What it tells you

**A per-subsystem table**, sorted by energy, with percentiles taken over the samples where the
mechanism was actually drawing current — a percentile over the whole match is dominated by idle
time and tells you nothing. Subsystems with a state channel also get a row per state, so
`Flywheel:ACTIVE` is separated from `Flywheel:IDLE`.

```
Subsystem               Energy (Wh)   On Samples  On Time (s)  On % Enabled   P50 (A) ...
Drive                       20.9375         7499      149.980       100.00%     19.78 ...
Flywheel:ACTIVE             10.0188         1950       39.000        26.00%     97.30 ...
```

**Every voltage sag**, worst first, with what each subsystem was drawing *during* the sag and in
the half-second *before* it. Both windows matter: at the bottom of a deep sag the current limiters
have often already cut in, so the draw during understates the cause — but when a mechanism slams
on at the same instant the voltage drops, the lookback shows an idle robot and the draw during is
the whole story.

**An unaccounted-draw figure** — the PDP's total current minus the sum of the named subsystems.
That gap is the roboRIO, radio, and Limelights. A gap much larger than that means something is
drawing power that nothing in code accounts for.

**A sample-rate warning** if any current channel is logged too slowly to resolve a brownout.
Phoenix publishes current signals at 4 Hz by default, which cannot see a 200 ms event; if this
fires, `CtreUtil.setCurrentSignalFrequency` is not taking effect on that motor.

Plots, with `--out`: `overview.png` (voltage and total draw, auto/teleop bands, sags marked),
`stack.png` (stacked draw by subsystem), and `events/event_NN.png` (a ±2 s zoom per sag).

## Where the data comes from

The `Supply Current Total` getter on each subsystem, and
`frc.robot.subsystems.power.PowerMonitor` for battery voltage, brownout state, and PDP totals.
Supply current, not stator: stator current is measured on the motor side of the controller and
can be several times what is actually drawn from the battery, so a stator sum badly overstates
the power budget.

If the script exits with a list of missing channels, the log predates those getters, or
`config.minimumImportance` in `Robot.java` is back above `DEBUG`. It refuses to run rather than
charting zeros.

## Feeding results back

The state-bucketed rows show whether the rules in `RobotCurrentLimits.java` are actually taking
effect — `Drive:SCORING` should be visibly cheaper than `Drive` overall. The P90/P99 columns show
how much of each configured limit is really being used. Note that `CurrentLimitManager` is
disabled during autonomous (`Robot.autonomousInit`), which the auto/teleop split makes visible.

## Files

| File | |
|---|---|
| `analyze_power.py` | the analysis; `SUBSYSTEMS` at the top maps subsystem → log channel |
| `wpilog.py` | dependency-free WPILOG reader |
| `make_sample_log.py` | writes a synthetic match log so the analysis can be developed without a robot. **Its numbers are invented** — never read them as measurements |
