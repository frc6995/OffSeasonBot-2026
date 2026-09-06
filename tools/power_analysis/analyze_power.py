#!/usr/bin/env python3
"""Per-subsystem power analysis of a match log, for tracking down brownouts.

Reads a .wpilog written by the robot code in this repo and reports where the current went:
which subsystem burned how much energy, what it drew at the 50th through 99th percentile while
it was actually running, and - for every voltage sag - what was drawing current in the moments
before it.

    python3 tools/power_analysis/analyze_power.py logs/FRC_20260101_120000.wpilog
    python3 tools/power_analysis/analyze_power.py <log> --out report/ --csv

The channels this reads are produced by the `Supply Current Total` getters on each subsystem and
by frc.robot.subsystems.power.PowerMonitor. A log recorded before those existed will fail the
channel check below with an explicit list of what is missing - that is the intended behaviour,
not a bug: silently charting zeros would be worse than refusing to run.

Only the plotting is optional (matplotlib); the tables are pure standard library.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from dataclasses import dataclass

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from wpilog import DataLog  # noqa: E402


# Epilogue nests everything under the Robot object and then the RobotContainer field; see
# RobotLogger/RobotContainerLogger. DataLogManager records NetworkTables topics with an "NT:"
# prefix, which is how these reach the file.
PREFIX = "NT:Robot/m_robotContainer/"


@dataclass
class Subsystem:
    """One row of the power budget: a named current channel, optionally split by robot state."""

    name: str
    channel: str
    # When set, the subsystem also gets a row per distinct value of this state channel. This is
    # what separates "the flywheel costs 6 Wh" from "the flywheel costs 6 Wh *while spun up*".
    state_channel: str | None = None


# The power budget. Drive and steer are separate rows because they fail differently - drive
# current tracks acceleration and pushing matches, steer current spikes on direction reversals.
SUBSYSTEMS = [
    # Drive is bucketed by the overall robot state because that is what RobotCurrentLimits keys
    # its drivetrain throttling off: while the superstructure is SCORING/PASSING/SAFE_SHOT the
    # drive supply limit is cut hard. Without this split there is no way to see from a log whether
    # that rule actually fired.
    Subsystem("Drive", "Swerve/Drive/Supply Current Total", "Robot State"),
    Subsystem("DriveSteer", "Swerve/Steer/Supply Current Total"),
    Subsystem("Flywheel", "Flywheel/Supply Current Total", "Flywheel/State"),
    Subsystem("Intake_Roller", "Intake/Roller/Supply Current Total", "Intake/State"),
    Subsystem("Intake_Extension", "Intake/Extension/Supply Current Total"),
    Subsystem("Intake_Kicker", "Intake/Kicker/Supply Current Total"),
    Subsystem("DyeRotor_Spin", "Dye Rotor/Spin/Supply Current Total"),
    Subsystem("DyeRotor_Index", "Dye Rotor/Index/Supply Current Total"),
    Subsystem("Turret", "Turret/Supply Current Total"),
    Subsystem("Hood", "Hood/Supply Current Total"),
]

BATTERY_VOLTAGE = "Power/Battery Voltage"
BROWNED_OUT = "Power/Browned Out"
BROWNOUT_VOLTAGE = "Power/Brownout Voltage"
PDP_TOTAL_CURRENT = "Power/PDP/Total Current"
PDP_CONNECTED = "Power/PDP/Connected"
CAN_LOWER = "Power/CAN/LowerBus Utilization"
CAN_UPPER = "Power/CAN/UpperBus Utilization"

# Logged by SuperstructureLogger directly onto the RobotContainer backend, so it sits one level
# above the per-subsystem trees. Used as Drive's state channel above.
ROBOT_STATE = "Robot State"

# Driver Station state, written by DriverStation.startDataLog() rather than by Epilogue, so these
# are unprefixed.
DS_ENABLED = "DS:enabled"
DS_AUTONOMOUS = "DS:autonomous"

# A current channel that never gets faster than this cannot resolve a brownout, which lasts a
# couple of hundred milliseconds. The robot asks for 50 Hz; hitting this warning means that rate
# is not reaching the logged channel - see CtreUtil.kCurrentSignalFrequencyHz.
MIN_USEFUL_SAMPLE_RATE_HZ = 20.0


# --------------------------------------------------------------------------------------------
# Channel resolution
# --------------------------------------------------------------------------------------------


class MissingChannels(Exception):
    def __init__(self, missing: list[str]):
        self.missing = missing
        super().__init__("missing channels: " + ", ".join(missing))


def resolve(log: DataLog, suffix: str) -> str | None:
    """Full log name for a channel, given its path below PREFIX.

    Falls back to a unique suffix match so that a change to the Epilogue nesting (renaming the
    RobotContainer field, say) degrades into a still-working script rather than a wall of missing
    channels.
    """
    exact = PREFIX + suffix
    if log.get(exact) is not None:
        return exact
    if log.get(suffix) is not None:
        return suffix
    matches = log.find("/" + suffix)
    if len(matches) == 1:
        return matches[0].name
    return None


# --------------------------------------------------------------------------------------------
# Resampling
# --------------------------------------------------------------------------------------------


def resample(times: list[float], values: list, grid: list[float], default=0.0) -> list:
    """Zero-order hold onto a uniform grid.

    Epilogue's lazy backend only writes a value when it changes, so channels land in the log at
    wildly different and irregular rates. Holding the last written value is the correct
    reconstruction for that: a channel that stopped being written did not stop having a value.
    """
    out = []
    if not times:
        return [default] * len(grid)

    index = 0
    current = default
    for t in grid:
        while index < len(times) and times[index] <= t:
            current = values[index]
            index += 1
        out.append(current)
    return out


def build_grid(t_start: float, t_end: float, dt: float) -> list[float]:
    count = max(1, int((t_end - t_start) / dt) + 1)
    return [t_start + i * dt for i in range(count)]


# --------------------------------------------------------------------------------------------
# Statistics
# --------------------------------------------------------------------------------------------


def percentile(sorted_values: list[float], fraction: float) -> float:
    """Linear-interpolated percentile of an already-sorted list."""
    if not sorted_values:
        return 0.0
    position = (len(sorted_values) - 1) * fraction
    low = math.floor(position)
    high = math.ceil(position)
    if low == high:
        return sorted_values[int(position)]
    return sorted_values[low] * (high - position) + sorted_values[high] * (position - low)


@dataclass
class Row:
    name: str
    energy_wh: float
    on_samples: int
    on_time_s: float
    on_fraction: float
    p50: float
    p75: float
    p90: float
    p99: float


def summarize(
    name: str,
    currents: list[float],
    voltages: list[float],
    mask: list[bool],
    dt: float,
    enabled_time_s: float,
    on_threshold_a: float,
) -> Row:
    """Energy and percentile stats for one current series over the samples `mask` selects.

    Percentiles are taken over *active* samples only - those above `on_threshold_a`. A percentile
    over the whole match is dominated by the time the mechanism sat idle and says nothing useful:
    a flywheel that is spun up for 20% of a match would show a P90 of roughly zero.

    Energy is integrated over every selected sample, idle included, because idle draw is real
    draw and does belong in the budget.
    """
    energy_joules = 0.0
    active: list[float] = []

    for current, voltage, selected in zip(currents, voltages, mask):
        if not selected:
            continue
        energy_joules += voltage * current * dt
        if current > on_threshold_a:
            active.append(current)

    active.sort()
    on_time = len(active) * dt
    return Row(
        name=name,
        energy_wh=energy_joules / 3600.0,
        on_samples=len(active),
        on_time_s=on_time,
        on_fraction=(on_time / enabled_time_s) if enabled_time_s > 0 else 0.0,
        p50=percentile(active, 0.50),
        p75=percentile(active, 0.75),
        p90=percentile(active, 0.90),
        p99=percentile(active, 0.99),
    )


# --------------------------------------------------------------------------------------------
# Brownout events
# --------------------------------------------------------------------------------------------


@dataclass
class SagEvent:
    start_s: float
    end_s: float
    min_voltage: float
    crossed_brownout: bool
    flagged_by_rio: bool
    # (subsystem, mean amps during the sag, mean amps over the lookback window), ranked by the
    # larger of the two. Both are reported because either can be the informative one - see
    # attribute().
    attribution: list[tuple[str, float, float]]
    peak_current_a: float


def find_sags(
    grid: list[float],
    voltages: list[float],
    browned_out: list[bool],
    sag_threshold: float,
    brownout_threshold: float,
    min_duration_s: float,
) -> list[tuple[int, int]]:
    """Index ranges where battery voltage sat below `sag_threshold`."""
    spans: list[tuple[int, int]] = []
    start: int | None = None
    dt = (grid[1] - grid[0]) if len(grid) > 1 else 0.02

    for i, voltage in enumerate(voltages):
        low = voltage < sag_threshold or browned_out[i]
        if low and start is None:
            start = i
        elif not low and start is not None:
            if (i - start) * dt >= min_duration_s:
                spans.append((start, i))
            start = None
    if start is not None and (len(voltages) - start) * dt >= min_duration_s:
        spans.append((start, len(voltages)))

    # A sag that flickers back above the threshold for a loop or two is one event, not three.
    merged: list[tuple[int, int]] = []
    gap_samples = max(1, int(0.25 / dt))
    for span in spans:
        if merged and span[0] - merged[-1][1] <= gap_samples:
            merged[-1] = (merged[-1][0], span[1])
        else:
            merged.append(span)
    return merged


def attribute(
    spans: list[tuple[int, int]],
    grid: list[float],
    voltages: list[float],
    browned_out: list[bool],
    series: dict[str, list[float]],
    total_current: list[float],
    brownout_threshold: float,
    lookback_s: float,
    dt: float,
) -> list[SagEvent]:
    """For each sag, rank subsystems by how much they were drawing, during it and just before.

    Both windows are reported because either can be the misleading one on its own. At the bottom
    of a deep sag the current limiters have often already cut in, so the draw *during* understates
    what caused it - that is what the lookback is for. But when a mechanism slams on at the same
    instant the voltage drops, the lookback shows an idle robot and the draw *during* is the whole
    story. Reading them side by side is what distinguishes "the flywheel had been loading the
    battery for half a second" from "the drivetrain spiked right now".
    """
    lookback_samples = max(1, int(lookback_s / dt))
    events = []

    for start, end in spans:
        end = max(end, start + 1)
        during = slice(start, end)
        before_start = max(0, start - lookback_samples)
        before = slice(before_start, max(before_start + 1, start))

        ranked = []
        for name, values in series.items():
            during_chunk = values[during]
            before_chunk = values[before]
            during_mean = sum(during_chunk) / len(during_chunk) if during_chunk else 0.0
            before_mean = sum(before_chunk) / len(before_chunk) if before_chunk else 0.0
            ranked.append((name, during_mean, before_mean))
        ranked.sort(key=lambda row: -max(row[1], row[2]))

        events.append(
            SagEvent(
                start_s=grid[start],
                end_s=grid[min(end, len(grid) - 1)],
                min_voltage=min(voltages[during]),
                crossed_brownout=any(v < brownout_threshold for v in voltages[during]),
                flagged_by_rio=any(browned_out[during]),
                attribution=ranked,
                peak_current_a=max(total_current[during]) if total_current[during] else 0.0,
            )
        )
    return events


# --------------------------------------------------------------------------------------------
# Report
# --------------------------------------------------------------------------------------------


def format_table(rows: list[Row]) -> str:
    header = (
        f"{'Subsystem':<22}{'Energy (Wh)':>13}{'On Samples':>13}{'On Time (s)':>13}"
        f"{'On % Enabled':>14}{'P50 (A)':>10}{'P75 (A)':>10}{'P90 (A)':>10}{'P99 (A)':>10}"
    )
    lines = [header, "-" * len(header)]
    for row in rows:
        lines.append(
            f"{row.name:<22}{row.energy_wh:>13.4f}{row.on_samples:>13d}{row.on_time_s:>13.3f}"
            f"{row.on_fraction * 100:>13.2f}%{row.p50:>10.2f}{row.p75:>10.2f}"
            f"{row.p90:>10.2f}{row.p99:>10.2f}"
        )
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Per-subsystem current/energy breakdown of a match log.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("log", help="path to a .wpilog")
    parser.add_argument("--out", help="directory for plots and CSV (created if needed)")
    parser.add_argument(
        "--csv", action="store_true",
        help="write the resampled matrix as CSV (requires --out)",
    )
    parser.add_argument("--dt", type=float, default=0.02, help="resample interval, seconds")
    parser.add_argument(
        "--on-threshold", type=float, default=1.0,
        help="amps above which a subsystem counts as active, for the percentile columns",
    )
    parser.add_argument(
        "--sag-v", type=float, default=7.5,
        help="battery voltage below which a window counts as a sag worth investigating",
    )
    parser.add_argument(
        "--brownout-v", type=float, default=6.8,
        help="fallback brownout threshold if the log does not record the roboRIO's own",
    )
    parser.add_argument(
        "--min-sag-s", type=float, default=0.05, help="ignore sags shorter than this"
    )
    parser.add_argument(
        "--lookback-s", type=float, default=0.5,
        help="window before each sag used to attribute it to subsystems",
    )
    parser.add_argument(
        "--max-events", type=int, default=10,
        help="how many sags to print in full, worst first (0 for all)",
    )
    parser.add_argument(
        "--no-by-state", action="store_true", help="omit the per-robot-state breakdown rows"
    )
    parser.add_argument(
        "--all-time", action="store_true",
        help="analyze the whole log rather than only the enabled periods",
    )
    args = parser.parse_args()

    if args.dt <= 0:
        parser.error(
            f"--dt must be greater than zero (got {args.dt}). It is the resample interval in "
            "seconds; 0.02 matches the robot's loop period."
        )
    if args.csv and not args.out:
        parser.error("--csv writes into the report directory, so it needs --out as well.")

    log = DataLog.read(args.log)
    t_start, t_end = log.span_seconds()
    print(f"Log:      {args.log}")
    print(f"Entries:  {len(log.entries)}")
    print(f"Span:     {t_end - t_start:.1f} s")
    if log.truncated:
        print("          (log ends in a torn record - robot lost power mid-write? "
              "Everything before it is still valid.)")
    if log.malformed_records:
        print(f"          ({log.malformed_records} malformed value records skipped)")
    print()

    # ---- resolve channels, and refuse to guess ----
    resolved: dict[str, str] = {}
    missing: list[str] = []
    for subsystem in SUBSYSTEMS:
        name = resolve(log, subsystem.channel)
        if name is None:
            missing.append(subsystem.channel)
        else:
            resolved[subsystem.name] = name

    voltage_channel = resolve(log, BATTERY_VOLTAGE)
    if voltage_channel is None:
        missing.append(BATTERY_VOLTAGE)

    if missing:
        print("ERROR: this log does not contain the channels the analysis needs:\n")
        for name in missing:
            print(f"  - {PREFIX}{name}")
        print(
            "\nMost likely the log predates the power-logging changes, or Epilogue's\n"
            "minimumImportance is back above DEBUG in Robot.java. Channels present in this log:\n"
        )
        for name in log.names():
            print(f"  {name}")
        return 1

    # ---- resample everything onto one grid ----
    grid = build_grid(t_start, t_end, args.dt)
    dt = args.dt

    series: dict[str, list[float]] = {}
    for subsystem in SUBSYSTEMS:
        times, values = log.series(resolved[subsystem.name])
        series[subsystem.name] = resample(times, values, grid)

    voltages = resample(*log.series(voltage_channel), grid=grid, default=12.0)

    browned_out_channel = resolve(log, BROWNED_OUT)
    browned_out = (
        resample(*log.series(browned_out_channel), grid=grid, default=False)
        if browned_out_channel
        else [False] * len(grid)
    )

    enabled = resample(*log.series(DS_ENABLED), grid=grid, default=False)
    autonomous = resample(*log.series(DS_AUTONOMOUS), grid=grid, default=False)

    mask = [True] * len(grid) if args.all_time else [bool(e) for e in enabled]
    enabled_time = sum(1 for m in mask if m) * dt
    auto_time = sum(1 for m, a in zip(mask, autonomous) if m and a) * dt
    print(f"Enabled:  {enabled_time:.1f} s  (auto {auto_time:.1f} s, teleop {enabled_time - auto_time:.1f} s)")
    if enabled_time == 0:
        print("\nWARNING: the robot was never enabled in this log. Re-run with --all-time to\n"
              "analyze it anyway; percentages relative to enabled time will be meaningless.")

    # ---- sample rate sanity check ----
    # Judged on the burst rate, not the mean: Epilogue's backend is lazy, so a channel that sat
    # constant for ten seconds logs a low mean rate while being perfectly capable of 50 Hz. Only
    # a channel that never gets fast is actually mis-configured. See Entry.burst_rate_hz.
    slow = []
    for subsystem in SUBSYSTEMS:
        entry = log.get(resolved[subsystem.name])
        rate = entry.burst_rate_hz() if entry else 0.0
        if rate < MIN_USEFUL_SAMPLE_RATE_HZ:
            slow.append((subsystem.name, rate))
    if slow:
        print("\nWARNING: these channels never reach a useful rate, even while changing, so a")
        print("         brownout would fall between samples (expected >= %.0f Hz; see" % MIN_USEFUL_SAMPLE_RATE_HZ)
        print("         CtreUtil.kCurrentSignalFrequencyHz):")
        for name, rate in slow:
            print(f"           {name:<22} {rate:6.1f} Hz")

    # ---- main table ----
    rows = [
        summarize(s.name, series[s.name], voltages, mask, dt, enabled_time, args.on_threshold)
        for s in SUBSYSTEMS
    ]

    if not args.no_by_state:
        for subsystem in SUBSYSTEMS:
            if subsystem.state_channel is None:
                continue
            state_name = resolve(log, subsystem.state_channel)
            if state_name is None:
                continue
            states = resample(*log.series(state_name), grid=grid, default="")
            for state in sorted({s for s in states if s}):
                state_mask = [m and s == state for m, s in zip(mask, states)]
                row = summarize(
                    f"{subsystem.name}:{state}",
                    series[subsystem.name], voltages, state_mask, dt,
                    enabled_time, args.on_threshold,
                )
                if row.on_samples > 0:
                    rows.append(row)

    rows.sort(key=lambda r: -r.energy_wh)
    print("\n" + format_table(rows))

    total_energy = sum(
        r.energy_wh for r in rows if ":" not in r.name
    )
    print(f"\nTotal accounted motor energy: {total_energy:.3f} Wh over {enabled_time:.1f} s enabled")

    if any(r.energy_wh < 0 for r in rows):
        print(
            "\nSome rows show negative energy: that motor put more charge back into the battery\n"
            "  than it took out, i.e. it spent the window being back-driven. Real on a mechanism\n"
            "  coasting down, but a whole subsystem negative across a match usually means a\n"
            "  simulated motor rather than a measured one."
        )

    # ---- PDP cross-check ----
    total_current = [sum(series[s.name][i] for s in SUBSYSTEMS) for i in range(len(grid))]
    pdp_channel = resolve(log, PDP_TOTAL_CURRENT)
    pdp_total = resample(*log.series(pdp_channel), grid=grid) if pdp_channel else []
    # A flat zero means nothing is answering on the bus - the PowerDistribution object constructs
    # fine in simulation and reports zeros. Comparing against that would print a nonsense negative
    # gap and read as though a load had gone missing.
    pdp_reporting = any(value != 0.0 for value in pdp_total)

    if pdp_reporting:
        gaps = [p - t for p, t, m in zip(pdp_total, total_current, mask) if m]
        if gaps:
            gaps_sorted = sorted(gaps)
            print(
                f"\nUnaccounted draw (PDP total minus the sum of the rows above):"
                f" mean {sum(gaps) / len(gaps):6.1f} A, p95 {percentile(gaps_sorted, 0.95):6.1f} A"
            )
            print(
                "  This is the roboRIO, radio, Limelights, and anything else without a motor\n"
                "  controller behind it. A large or growing gap means a load nothing accounts for."
            )
    elif pdp_channel:
        print(
            "\nNote: the PDP reported zero current for the whole log, so the sum above was not\n"
            "      cross-checked against what the battery actually delivered. Expected in\n"
            "      simulation; on the real robot it means nothing answered at the CAN ID in\n"
            "      PowerMonitor.kPdpCanId."
        )
    else:
        print(
            "\nNote: no PDP total current channel in this log, so the sum above cannot be\n"
            "      cross-checked against what the battery actually delivered."
        )

    # ---- CAN utilization ----
    for label, suffix in (("LowerBus (swerve)", CAN_LOWER), ("UpperBus (superstructure)", CAN_UPPER)):
        channel = resolve(log, suffix)
        if not channel:
            continue
        _, values = log.series(channel)
        if not values:
            continue
        peak = max(values)
        if peak == 0.0:
            # No CANivore behind it, i.e. simulation. Saying "0.0%" would read as a real
            # measurement of an idle bus.
            print(f"CAN {label:<26} not reported (no CANivore - simulation?)")
        else:
            print(f"CAN {label:<26} peak utilization {peak * 100:5.1f}%")

    # ---- sag events ----
    brownout_threshold = args.brownout_v
    brownout_channel = resolve(log, BROWNOUT_VOLTAGE)
    if brownout_channel:
        _, values = log.series(brownout_channel)
        if values:
            brownout_threshold = values[-1]

    spans = find_sags(grid, voltages, browned_out, args.sag_v, brownout_threshold, args.min_sag_s)
    # Only sags while enabled are interesting; a disabled robot draws nothing.
    spans = [(a, b) for a, b in spans if any(mask[a:b])]
    events = attribute(
        spans, grid, voltages, browned_out, series, total_current,
        brownout_threshold, args.lookback_s, dt,
    )

    brownouts = [e for e in events if e.crossed_brownout or e.flagged_by_rio]
    print(
        f"\n\nVoltage sags below {args.sag_v:.1f} V: {len(events)}"
        f"  ({len(brownouts)} reached the {brownout_threshold:.2f} V brownout threshold)"
    )
    if not events:
        print("  None. Either the battery held up or the log has no enabled period under load.")

    # Worst first: with a tired battery there can be dozens, and the deepest ones are the ones
    # worth reading. The chronological position is on each line either way.
    shown = sorted(events, key=lambda e: e.min_voltage)
    if args.max_events > 0:
        shown = shown[: args.max_events]

    for i, event in enumerate(shown, 1):
        marker = "BROWNOUT" if (event.crossed_brownout or event.flagged_by_rio) else "sag     "
        print(
            f"\n  [{i}] {marker} at t={event.start_s:7.2f}s  duration {event.end_s - event.start_s:5.2f}s"
            f"  min {event.min_voltage:5.2f} V  peak draw {event.peak_current_a:6.1f} A"
        )
        print(f"      {'':<22}{'during':>10}{'  ':>2}{f'{args.lookback_s:.1f}s before':>12}")
        for name, during, before in event.attribution[:6]:
            if max(during, before) < 0.1:
                continue
            print(f"        {name:<22}{during:8.1f} A{before:10.1f} A")

    if len(events) > len(shown):
        print(f"\n  ... and {len(events) - len(shown)} more (pass --max-events 0 to see all).")

    # ---- outputs ----
    if args.out:
        os.makedirs(args.out, exist_ok=True)
        summary_path = os.path.join(args.out, "summary.txt")
        with open(summary_path, "w") as handle:
            handle.write(format_table(rows) + "\n")
        print(f"\nWrote {summary_path}")

        if args.csv:
            csv_path = os.path.join(args.out, "resampled.csv")
            write_csv(csv_path, grid, series, voltages, total_current, enabled, autonomous)
            print(f"Wrote {csv_path}")

        make_plots(
            args.out, grid, series, voltages, total_current, enabled, autonomous,
            all_events=events, detail_events=shown,
        )

    return 0


def write_csv(path, grid, series, voltages, total_current, enabled, autonomous) -> None:
    import csv

    names = list(series)
    with open(path, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["time_s", "battery_v", "total_a", "enabled", "autonomous"] + names)
        for i, t in enumerate(grid):
            writer.writerow(
                [f"{t:.3f}", f"{voltages[i]:.3f}", f"{total_current[i]:.3f}",
                 int(bool(enabled[i])), int(bool(autonomous[i]))]
                + [f"{series[name][i]:.3f}" for name in names]
            )


def make_plots(
    out_dir, grid, series, voltages, total_current, enabled, autonomous,
    all_events, detail_events,
) -> None:
    """Every sag is shaded on the overview; only `detail_events` get their own zoom plot."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print(
            "\nSkipping plots: matplotlib is not installed.\n"
            "  pip install -r tools/power_analysis/requirements.txt"
        )
        return

    # Overview: what the battery did, and when.
    fig, (ax_v, ax_i) = plt.subplots(2, 1, sharex=True, figsize=(14, 8))
    ax_v.plot(grid, voltages, linewidth=0.8, color="tab:blue")
    ax_v.set_ylabel("Battery (V)")
    ax_v.grid(alpha=0.3)
    for event in all_events:
        ax_v.axvspan(event.start_s, event.end_s, color="tab:red", alpha=0.25)
        ax_i.axvspan(event.start_s, event.end_s, color="tab:red", alpha=0.25)
    _shade_modes(ax_v, grid, enabled, autonomous)

    ax_i.plot(grid, total_current, linewidth=0.8, color="tab:orange")
    ax_i.set_ylabel("Total supply current (A)")
    ax_i.set_xlabel("Time (s)")
    ax_i.grid(alpha=0.3)
    fig.suptitle("Battery voltage and total motor draw")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "overview.png"), dpi=120)
    plt.close(fig)

    # Stack: who is responsible for that total.
    fig, ax = plt.subplots(figsize=(14, 6))
    names = list(series)
    ax.stackplot(grid, *[series[name] for name in names], labels=names)
    ax.legend(loc="upper left", ncol=3, fontsize=8)
    ax.set_ylabel("Supply current (A)")
    ax.set_xlabel("Time (s)")
    ax.grid(alpha=0.3)
    ax.set_title("Supply current by subsystem")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "stack.png"), dpi=120)
    plt.close(fig)

    if not detail_events:
        return
    events_dir = os.path.join(out_dir, "events")
    os.makedirs(events_dir, exist_ok=True)
    # Clear plots from a previous run over a different log. Leftovers numbered above this run's
    # event count would sit there looking like part of this report.
    for stale in os.listdir(events_dir):
        if stale.startswith("event_") and stale.endswith(".png"):
            os.remove(os.path.join(events_dir, stale))
    dt = (grid[1] - grid[0]) if len(grid) > 1 else 0.02
    pad = int(2.0 / dt)
    for i, event in enumerate(detail_events, 1):
        start = max(0, int((event.start_s - grid[0]) / dt) - pad)
        end = min(len(grid), int((event.end_s - grid[0]) / dt) + pad)
        window = slice(start, end)

        fig, (ax_v, ax_i) = plt.subplots(2, 1, sharex=True, figsize=(11, 7))
        ax_v.plot(grid[window], voltages[window], color="tab:blue")
        ax_v.axvspan(event.start_s, event.end_s, color="tab:red", alpha=0.2)
        ax_v.set_ylabel("Battery (V)")
        ax_v.grid(alpha=0.3)
        # Only the subsystems that were actually drawing, so the legend stays readable.
        for name, _, _ in event.attribution[:5]:
            ax_i.plot(grid[window], series[name][window], label=name, linewidth=1.0)
        ax_i.axvspan(event.start_s, event.end_s, color="tab:red", alpha=0.2)
        ax_i.legend(fontsize=8)
        ax_i.set_ylabel("Supply current (A)")
        ax_i.set_xlabel("Time (s)")
        ax_i.grid(alpha=0.3)
        fig.suptitle(f"Sag {i}: t={event.start_s:.2f}s, min {event.min_voltage:.2f} V")
        fig.tight_layout()
        fig.savefig(os.path.join(events_dir, f"event_{i:02d}.png"), dpi=120)
        plt.close(fig)

    print(f"Wrote {len(detail_events)} event plots to {events_dir}")


def _shade_modes(ax, grid, enabled, autonomous) -> None:
    """Light background bands for autonomous and teleop, to orient the reader in the match."""
    start = None
    mode = None
    for i, t in enumerate(grid):
        current = ("auto" if autonomous[i] else "teleop") if enabled[i] else None
        if current != mode:
            if mode is not None and start is not None:
                ax.axvspan(start, t, color="tab:green" if mode == "auto" else "tab:gray", alpha=0.08)
            mode, start = current, t
    if mode is not None and start is not None:
        ax.axvspan(start, grid[-1], color="tab:green" if mode == "auto" else "tab:gray", alpha=0.08)


if __name__ == "__main__":
    sys.exit(main())
