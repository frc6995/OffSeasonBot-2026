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

**A sample-rate warning** if any current channel never reaches a rate that can resolve a
brownout. The robot asks for 50 Hz (`CtreUtil.kCurrentSignalFrequencyHz`); if this fires, that
rate is not reaching the logged channel, and a 200 ms event would fall between samples.

The check uses the rate a channel reaches *while changing*, not its mean. Epilogue's backend is
lazy — it only writes a value when it changes — so a channel that sat constant for ten seconds
logs a low mean rate while being perfectly capable of 50 Hz. Only a channel that never gets fast
is actually mis-configured.

Plots, with `--out`: `overview.png` (voltage and total draw, auto/teleop bands, *every* sag
marked), `stack.png` (stacked draw by subsystem), and `events/event_NN.png` — a ±2 s zoom for
each of the sags printed in full, so `--max-events` (default 10) caps these too. Raise it or pass
`--max-events 0` to plot every sag.

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
effect — `Drive:SCORING` should be visibly cheaper than `Drive` overall. (Those rows exist
because `Drive` declares `Robot State` as its `state_channel` in `SUBSYSTEMS`; add one to any
other subsystem you want split the same way.) The P90/P99 columns show how much of each
configured limit is really being used. Note that `CurrentLimitManager` is disabled during
autonomous (`Robot.autonomousInit`), which the auto/teleop split makes visible.

## Files

| File | |
|---|---|
| `analyze_power.py` | the analysis; `SUBSYSTEMS` at the top maps subsystem → log channel |
| `wpilog.py` | dependency-free WPILOG reader |
| `requirements.txt` | matplotlib, for the plots only |
| `make_sample_log.py` | writes a synthetic match log so the analysis can be developed without a robot. **Its numbers are invented** — never read them as measurements |
