#!/usr/bin/env python3
"""Writes a synthetic .wpilog shaped like a match, for developing analyze_power.py off-robot.

THE NUMBERS IN THE OUTPUT ARE INVENTED. This exists so the analysis can be exercised end to end
- table maths, sag detection, attribution, plots - without waiting for hardware or a practice
match. Never present its output as a measurement of anything.

    python3 tools/power_analysis/make_sample_log.py /tmp/sample.wpilog
    python3 tools/power_analysis/analyze_power.py /tmp/sample.wpilog --out /tmp/report --csv

The channel names come from analyze_power.SUBSYSTEMS, so if the robot code renames a channel and
analyze_power is updated to match, this follows automatically.
"""

from __future__ import annotations

import math
import os
import random
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import analyze_power as ap  # noqa: E402


class LogWriter:
    """Minimal WPILOG writer: fixed-width record headers, which the format permits."""

    # entry id 4 bytes, payload length 4 bytes, timestamp 8 bytes.
    _HEADER_BYTE = (4 - 1) | ((4 - 1) << 2) | ((8 - 1) << 4)

    def __init__(self, path: str):
        self.handle = open(path, "wb")
        self.handle.write(b"WPILOG" + struct.pack("<HI", 0x0100, 0))
        self.next_id = 1

    def start_entry(self, name: str, type_name: str, timestamp_us: int = 0) -> int:
        entry_id = self.next_id
        self.next_id += 1
        payload = (
            b"\x00"
            + struct.pack("<I", entry_id)
            + self._string(name)
            + self._string(type_name)
            + self._string("")
        )
        self._record(0, timestamp_us, payload)
        return entry_id

    def write_double(self, entry_id: int, timestamp_us: int, value: float) -> None:
        self._record(entry_id, timestamp_us, struct.pack("<d", value))

    def write_boolean(self, entry_id: int, timestamp_us: int, value: bool) -> None:
        self._record(entry_id, timestamp_us, b"\x01" if value else b"\x00")

    def write_string(self, entry_id: int, timestamp_us: int, value: str) -> None:
        self._record(entry_id, timestamp_us, value.encode("utf-8"))

    def close(self) -> None:
        self.handle.close()

    @staticmethod
    def _string(value: str) -> bytes:
        encoded = value.encode("utf-8")
        return struct.pack("<I", len(encoded)) + encoded

    def _record(self, entry_id: int, timestamp_us: int, payload: bytes) -> None:
        self.handle.write(
            bytes([self._HEADER_BYTE])
            + struct.pack("<IIQ", entry_id, len(payload), timestamp_us)
        )
        self.handle.write(payload)


def main() -> int:
    path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/sample.wpilog"
    random.seed(6995)

    dt = 0.02
    auto_s, teleop_s = 15.0, 135.0
    total_s = 5.0 + auto_s + 3.0 + teleop_s

    writer = LogWriter(path)
    current_ids = {
        s.name: writer.start_entry(ap.PREFIX + s.channel, "double") for s in ap.SUBSYSTEMS
    }
    voltage_id = writer.start_entry(ap.PREFIX + ap.BATTERY_VOLTAGE, "double")
    brownout_flag_id = writer.start_entry(ap.PREFIX + ap.BROWNED_OUT, "boolean")
    brownout_v_id = writer.start_entry(ap.PREFIX + ap.BROWNOUT_VOLTAGE, "double")
    pdp_total_id = writer.start_entry(ap.PREFIX + ap.PDP_TOTAL_CURRENT, "double")
    can_lower_id = writer.start_entry(ap.PREFIX + ap.CAN_LOWER, "double")
    can_upper_id = writer.start_entry(ap.PREFIX + ap.CAN_UPPER, "double")
    flywheel_state_id = writer.start_entry(ap.PREFIX + "Flywheel/State", "string")
    intake_state_id = writer.start_entry(ap.PREFIX + "Intake/State", "string")
    enabled_id = writer.start_entry(ap.DS_ENABLED, "boolean")
    autonomous_id = writer.start_entry(ap.DS_AUTONOMOUS, "boolean")

    writer.write_double(brownout_v_id, 0, 6.8)

    # A 12.6 V battery with 0.018 ohm of internal resistance plus wiring - a tired one, chosen so
    # that stacking a shot on top of hard acceleration actually sags into brownout territory.
    open_circuit_v, resistance = 12.6, 0.018
    steps = int(total_s / dt)
    last_flywheel_state = last_intake_state = None
    previous_enabled = None

    for step in range(steps):
        t = step * dt
        timestamp_us = int(t * 1e6)

        in_auto = 5.0 <= t < 5.0 + auto_s
        in_teleop = t >= 5.0 + auto_s + 3.0
        enabled = in_auto or in_teleop

        if enabled != previous_enabled:
            writer.write_boolean(enabled_id, timestamp_us, enabled)
            writer.write_boolean(autonomous_id, timestamp_us, in_auto)
            previous_enabled = enabled

        if not enabled:
            currents = {s.name: 0.0 for s in ap.SUBSYSTEMS}
            flywheel_state, intake_state = "IDLE", "RETRACTED"
        else:
            # Driving: bursts of acceleration with a slow background cruise.
            burst = 1.0 if (math.sin(t * 1.7) > 0.55) else 0.0
            drive = 18.0 + burst * 130.0 + random.uniform(-4, 4)
            steer = 6.0 + burst * 25.0 + random.uniform(-2, 2)

            # Shooting: spun up for a few seconds at a time, idling between.
            shooting = (t % 17.0) < 4.5
            flywheel_state = "ACTIVE" if shooting else "IDLE"
            flywheel = (200.0 if (t % 17.0) < 1.2 else 95.0) if shooting else 11.0
            flywheel += random.uniform(-6, 6)

            intaking = (t % 11.0) < 5.0
            intake_state = "ACTIVE" if intaking else "RETRACTED"
            roller = (26.0 + random.uniform(-4, 4)) if intaking else 0.0
            extension = 9.0 if (t % 11.0) < 0.6 else 0.3
            kicker = (30.0 + random.uniform(-5, 5)) if shooting else 0.0

            currents = {
                "Drive": max(0.0, drive),
                "DriveSteer": max(0.0, steer),
                "Flywheel": max(0.0, flywheel),
                "Intake_Roller": max(0.0, roller),
                "Intake_Extension": max(0.0, extension),
                "Intake_Kicker": max(0.0, kicker),
                "DyeRotor_Spin": 8.0 if intaking else 0.2,
                "DyeRotor_Index": 12.0 if shooting else 0.2,
                "Turret": 3.0 + 6.0 * abs(math.sin(t * 0.9)),
                "Hood": 1.5 if (t % 17.0) < 0.4 else 0.1,
            }

        motor_total = sum(currents.values())
        # Non-motor loads: roboRIO, radio, two Limelights.
        pdp_total = motor_total + 7.5 + random.uniform(-0.5, 0.5)
        voltage = open_circuit_v - pdp_total * resistance

        for name, value in currents.items():
            writer.write_double(current_ids[name], timestamp_us, value)
        writer.write_double(voltage_id, timestamp_us, voltage)
        writer.write_double(pdp_total_id, timestamp_us, pdp_total)
        writer.write_boolean(brownout_flag_id, timestamp_us, voltage < 6.8)

        if step % 25 == 0:
            writer.write_double(can_lower_id, timestamp_us, 0.31 + random.uniform(0, 0.04))
            writer.write_double(can_upper_id, timestamp_us, 0.44 + random.uniform(0, 0.05))

        if flywheel_state != last_flywheel_state:
            writer.write_string(flywheel_state_id, timestamp_us, flywheel_state)
            last_flywheel_state = flywheel_state
        if intake_state != last_intake_state:
            writer.write_string(intake_state_id, timestamp_us, intake_state)
            last_intake_state = intake_state

    writer.close()
    print(f"Wrote {path} ({os.path.getsize(path) / 1e6:.1f} MB, {total_s:.0f} s simulated)")
    print("Reminder: these numbers are invented. Do not read them as robot measurements.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
