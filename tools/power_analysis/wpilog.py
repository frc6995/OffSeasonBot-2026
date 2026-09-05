"""Dependency-free reader for WPILib data logs (.wpilog).

The format is documented at
https://github.com/wpilibsuite/allwpilib/blob/main/wpiutil/doc/datalog.adoc

Only the pieces the power analysis needs are implemented, but that covers every value type
Epilogue emits. Kept free of third-party dependencies on purpose: this has to run on whatever
laptop is in the pit, and `pip install` behind event wifi is not a plan.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from typing import Iterator

_HEADER_MAGIC = b"WPILOG"

# Control records are written to entry ID 0; the first payload byte says which kind.
_CONTROL_ENTRY_ID = 0
_CONTROL_START = 0
_CONTROL_FINISH = 1
_CONTROL_SET_METADATA = 2


class DataLogError(Exception):
    """Raised when a file is not a readable WPILOG."""


@dataclass
class Entry:
    """One logged channel: its name, its declared type, and every value written to it."""

    entry_id: int
    name: str
    type: str
    metadata: str
    # Parallel lists rather than a list of pairs: the analyzer slices whole columns.
    timestamps_us: list[int] = field(default_factory=list)
    raw_values: list[bytes] = field(default_factory=list)

    def __len__(self) -> int:
        return len(self.timestamps_us)

    def decoded(self) -> list:
        """Every value, decoded according to this entry's declared type."""
        decode = _decoder_for(self.type)
        return [decode(raw) for raw in self.raw_values]

    def sample_rate_hz(self) -> float:
        """Mean logged rate over the channel's whole span.

        Note this reads low for any channel that spends time constant, because Epilogue's backend
        is configured lazy: it only writes a value when it changes. Use burst_rate_hz() to judge
        whether a signal is actually being published fast enough.
        """
        if len(self.timestamps_us) < 2:
            return 0.0
        span_s = (self.timestamps_us[-1] - self.timestamps_us[0]) / 1e6
        if span_s <= 0:
            return 0.0
        return (len(self.timestamps_us) - 1) / span_s

    def burst_rate_hz(self) -> float:
        """The rate this channel reaches when it is actually changing.

        Derived from the 10th-percentile gap between consecutive samples rather than the mean,
        because the mean cannot tell "the motor controller is only publishing 4 times a second"
        apart from "this value sat constant for ten seconds and the lazy backend skipped it". The
        first is a real problem that makes a brownout invisible; the second is the logger working
        as designed. A channel that ever sustains 50 Hz can resolve a brownout whenever there is
        something to resolve, however quiet it is the rest of the time.
        """
        if len(self.timestamps_us) < 3:
            return 0.0
        gaps = sorted(
            (self.timestamps_us[i + 1] - self.timestamps_us[i]) / 1e6
            for i in range(len(self.timestamps_us) - 1)
        )
        gap = gaps[int((len(gaps) - 1) * 0.10)]
        return (1.0 / gap) if gap > 0 else 0.0


def _decode_double(raw: bytes) -> float:
    return struct.unpack("<d", raw)[0] if len(raw) >= 8 else 0.0


def _decode_float(raw: bytes) -> float:
    return struct.unpack("<f", raw)[0] if len(raw) >= 4 else 0.0


def _decode_int64(raw: bytes) -> int:
    return struct.unpack("<q", raw)[0] if len(raw) >= 8 else 0


def _decode_boolean(raw: bytes) -> bool:
    return bool(raw[0]) if raw else False


def _decode_string(raw: bytes) -> str:
    return raw.decode("utf-8", "replace")


def _decode_double_array(raw: bytes) -> list[float]:
    count = len(raw) // 8
    return list(struct.unpack(f"<{count}d", raw[: count * 8]))


def _decode_float_array(raw: bytes) -> list[float]:
    count = len(raw) // 4
    return list(struct.unpack(f"<{count}f", raw[: count * 4]))


def _decode_int64_array(raw: bytes) -> list[int]:
    count = len(raw) // 8
    return list(struct.unpack(f"<{count}q", raw[: count * 8]))


def _decode_boolean_array(raw: bytes) -> list[bool]:
    return [bool(b) for b in raw]


def _decode_string_array(raw: bytes) -> list[str]:
    if len(raw) < 4:
        return []
    count = struct.unpack_from("<I", raw, 0)[0]
    values, pos = [], 4
    for _ in range(count):
        if pos + 4 > len(raw):
            break
        length = struct.unpack_from("<I", raw, pos)[0]
        pos += 4
        values.append(raw[pos : pos + length].decode("utf-8", "replace"))
        pos += length
    return values


_DECODERS = {
    "double": _decode_double,
    "float": _decode_float,
    "int64": _decode_int64,
    "boolean": _decode_boolean,
    "string": _decode_string,
    "json": _decode_string,
    "double[]": _decode_double_array,
    "float[]": _decode_float_array,
    "int64[]": _decode_int64_array,
    "boolean[]": _decode_boolean_array,
    "string[]": _decode_string_array,
}


def _decoder_for(type_name: str):
    # Struct-encoded types (struct:Pose2d and friends) are left as raw bytes; nothing in the
    # power analysis reads them, and decoding them would mean parsing the schema records too.
    return _DECODERS.get(type_name, lambda raw: raw)


class DataLog:
    """A parsed .wpilog, indexed by channel name."""

    def __init__(self, entries: dict[str, Entry], start_timestamp_us: int):
        self.entries = entries
        self.start_timestamp_us = start_timestamp_us

    @classmethod
    def read(cls, path: str) -> "DataLog":
        with open(path, "rb") as handle:
            data = handle.read()
        return cls.from_bytes(data)

    @classmethod
    def from_bytes(cls, data: bytes) -> "DataLog":
        if not data.startswith(_HEADER_MAGIC):
            raise DataLogError(
                f"not a WPILOG file (expected magic {_HEADER_MAGIC!r}, got {data[:6]!r})"
            )
        extra_header_len = struct.unpack_from("<I", data, 8)[0]
        pos = 12 + extra_header_len

        by_id: dict[int, Entry] = {}
        entries: dict[str, Entry] = {}
        earliest: int | None = None
        size = len(data)

        while pos < size:
            try:
                pos, entry_id, timestamp_us, payload = _read_record(data, pos)
            except (struct.error, IndexError):
                # A log from a robot that lost power mid-write ends in a torn record. Everything
                # before it is still valid, so keep it rather than failing the whole analysis.
                break

            if entry_id == _CONTROL_ENTRY_ID:
                _apply_control_record(payload, by_id, entries)
                continue

            entry = by_id.get(entry_id)
            if entry is None:
                # A value for an entry whose start record we never saw. Not recoverable, skip.
                continue
            entry.timestamps_us.append(timestamp_us)
            entry.raw_values.append(payload)
            if earliest is None or timestamp_us < earliest:
                earliest = timestamp_us

        return cls(entries, earliest or 0)

    def get(self, name: str) -> Entry | None:
        return self.entries.get(name)

    def find(self, suffix: str) -> list[Entry]:
        """Every entry whose name ends with `suffix`, for resolving a channel by short name."""
        return [entry for name, entry in self.entries.items() if name.endswith(suffix)]

    def names(self) -> list[str]:
        return sorted(self.entries)

    def span_seconds(self) -> tuple[float, float]:
        """(first, last) timestamp in seconds, relative to the first record in the log."""
        starts = [e.timestamps_us[0] for e in self.entries.values() if e.timestamps_us]
        ends = [e.timestamps_us[-1] for e in self.entries.values() if e.timestamps_us]
        if not starts:
            return (0.0, 0.0)
        return (
            (min(starts) - self.start_timestamp_us) / 1e6,
            (max(ends) - self.start_timestamp_us) / 1e6,
        )

    def series(self, name: str) -> tuple[list[float], list]:
        """(times in seconds relative to log start, decoded values) for one channel."""
        entry = self.entries.get(name)
        if entry is None:
            return ([], [])
        times = [(ts - self.start_timestamp_us) / 1e6 for ts in entry.timestamps_us]
        return (times, entry.decoded())


def _read_record(data: bytes, pos: int) -> tuple[int, int, int, bytes]:
    """Reads one record. Returns (next position, entry id, timestamp us, payload)."""
    header = data[pos]
    pos += 1
    # The header bitfield stores each field's byte length minus one.
    id_len = (header & 0x3) + 1
    payload_len_len = ((header >> 2) & 0x3) + 1
    timestamp_len = ((header >> 4) & 0x7) + 1

    entry_id = int.from_bytes(data[pos : pos + id_len], "little")
    pos += id_len
    payload_len = int.from_bytes(data[pos : pos + payload_len_len], "little")
    pos += payload_len_len
    timestamp_us = int.from_bytes(data[pos : pos + timestamp_len], "little")
    pos += timestamp_len

    payload = data[pos : pos + payload_len]
    if len(payload) < payload_len:
        raise IndexError("truncated record payload")
    pos += payload_len
    return pos, entry_id, timestamp_us, payload


def _apply_control_record(
    payload: bytes, by_id: dict[int, Entry], entries: dict[str, Entry]
) -> None:
    if not payload:
        return
    kind = payload[0]
    if kind == _CONTROL_START:
        pos = 1
        entry_id = struct.unpack_from("<I", payload, pos)[0]
        pos += 4
        name, pos = _read_length_prefixed_string(payload, pos)
        type_name, pos = _read_length_prefixed_string(payload, pos)
        metadata, pos = _read_length_prefixed_string(payload, pos)

        # An entry can be finished and restarted under a new ID within one log (a robot code
        # restart). Reuse the existing Entry so the channel reads as one continuous series.
        entry = entries.get(name)
        if entry is None:
            entry = Entry(entry_id, name, type_name, metadata)
            entries[name] = entry
        by_id[entry_id] = entry
    elif kind == _CONTROL_FINISH:
        entry_id = struct.unpack_from("<I", payload, 1)[0]
        by_id.pop(entry_id, None)
    elif kind == _CONTROL_SET_METADATA:
        entry_id = struct.unpack_from("<I", payload, 1)[0]
        metadata, _ = _read_length_prefixed_string(payload, 5)
        entry = by_id.get(entry_id)
        if entry is not None:
            entry.metadata = metadata


def _read_length_prefixed_string(payload: bytes, pos: int) -> tuple[str, int]:
    length = struct.unpack_from("<I", payload, pos)[0]
    pos += 4
    return payload[pos : pos + length].decode("utf-8", "replace"), pos + length


def iter_entries(path: str) -> Iterator[Entry]:
    """Convenience wrapper for scripts that just want to enumerate a log's channels."""
    return iter(DataLog.read(path).entries.values())
