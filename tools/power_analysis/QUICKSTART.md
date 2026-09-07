# Power analysis — quick reference

Full details in [README.md](README.md). Walkthrough for first-timers:
https://claude.ai/code/artifact/442fe294-8cb7-4b80-924c-21b581045b96

## 1. Get a log off the robot

Connected to the robot's wifi, in VS Code: `Ctrl/Cmd+Shift+P` → `WPILib: Start Tool` →
`DataLogTool`. In its Download section use team number (or `roborio-6995-frc.local`), user
`lvuser`, blank password. Save into `logs/`.

## 2. Run it

Run from the repo root (the folder with `build.gradle`).

```bash
# print to terminal only, write nothing
python3 tools/power_analysis/analyze_power.py logs/FRC_20260404_143012.wpilog

# also write reports/FRC_20260404_143012/ (derived from the log name)
python3 tools/power_analysis/analyze_power.py logs/FRC_20260404_143012.wpilog --out

# with CSV for spreadsheets (--csv implies --out)
python3 tools/power_analysis/analyze_power.py logs/FRC_20260404_143012.wpilog --out --csv

# choose the directory yourself
python3 tools/power_analysis/analyze_power.py <log> --out reports/practice-3
```

Charts need matplotlib once: `pip3 install -r tools/power_analysis/requirements.txt`
(tables work without it).

## 3. Read the output

| Look at | For |
|---|---|
| **Energy (Wh)** column | which subsystem drained the battery |
| **P90 / P99 (A)** columns | how spiky it is — spiky causes brownouts |
| **`Name:STATE` rows** | e.g. `Drive:SCORING` vs `Drive` — proves `RobotCurrentLimits` fired |
| **Sag list** (`during` vs `0.5s before`) | what was pulling when the voltage dropped |
| **Unaccounted draw** | non-motor load (RIO, radio, cameras); expect ~5–10 A |

Percentiles are over samples where the mechanism was *actually running*, not the whole match.

## 4. Comparing two practice matches

Compare **sag count and depth** and the **P90/P99** columns — those respond to current-limit
changes. Total Wh mostly does not: a limit caps the *rate* of energy delivery, so the robot
does the same work more slowly and uses about the same energy either way.

`summary.txt` is plain text, so: `diff reports/matchA/summary.txt reports/matchB/summary.txt`

## Useful flags

| Flag | |
|---|---|
| `--max-events 0` | print every sag, not just the 10 worst |
| `--all-time` | include disabled time (bench testing) |
| `--sag-v 8.0` | change what counts as a sag (default 7.5 V) |
| `--on-threshold 2` | amps above which a subsystem counts as active (default 1.0) |
| `--no-by-state` | drop the `Name:STATE` rows |
| `--help` | everything, with defaults |

## If something goes wrong

| Message | Fix |
|---|---|
| `does not contain the channels...` | log predates the power logging, or `minimumImportance` in `Robot.java` is above `DEBUG` |
| `command not found: python3` | Windows: use `py`. Otherwise install from python.org |
| `can't open file ...analyze_power.py` | wrong folder — `cd` to the repo root |
| `Skipping plots: matplotlib...` | not an error; install it if you want charts |
| `robot was never enabled` | bench log — add `--all-time` |
| `channels never reach a useful rate` | real problem: 50 Hz isn't reaching that motor (`CtreUtil.kCurrentSignalFrequencyHz`) |
| `PDP reported zero current` | normal in simulation; on the robot check the PDP CAN wiring |

## Notes

- Nothing here is deployed to the robot. Only `src/` and `src/main/deploy/` reach the roboRIO —
  never put logs or reports in `src/main/deploy/`, files there stay on its flash permanently.
- `logs/`, `reports/`, `*.wpilog` and `*.hoot` are gitignored. Keep the **log**, not the report —
  reports regenerate from it in seconds.
- Practice without a robot: `python3 tools/power_analysis/make_sample_log.py sample.wpilog`
  then analyse it. **Its numbers are invented** — never quote them as measurements.
