#!/usr/bin/env python3
"""Decode an ALARIS DAPU binary flight log (LOGnnnn.BIN).

The firmware writes a 1 kB header containing the struct format and the field
names, so this script never has to hard-code the record layout: adding a field
to log_record_t in dapu_types.h is enough for the decoder to follow.

Usage
    python decode_log.py LOG0001.BIN                    # summary + rate table
    python decode_log.py LOG0001.BIN --csv out.csv      # full CSV export
    python decode_log.py LOG0001.BIN --csv out.csv --drop-reserved
"""

import argparse
import csv
import struct
import sys

HEADER_SIZE = 1024
FILE_MAGIC = b"ALARIS-DAPU-LOG"
RECORD_MAGIC = 0xD1A5D1A5

# Sequence counters whose slope gives the measured acquisition rate of each
# sensor, paired with the rate the firmware was configured for.
RATE_FIELDS = [
    ("imu_seq", "ICM20948 accel/gyro", 200.0),
    ("bmp_seq", "BMP280", 100.0),
    ("ms_seq", "MS5611", 50.0),
    ("gps_seq", "NEO-M8N NAV-PVT", 10.0),
    ("rpm_seq", "RPM (FFT)", 1.0),
]

STATUS_BITS = [
    (1 << 0, "BMP280_OK"),
    (1 << 1, "MS5611_OK"),
    (1 << 2, "ICM20948_OK"),
    (1 << 3, "AK09916_OK"),
    (1 << 4, "GPS_FIX_3D"),
    (1 << 5, "SD_OK"),
    (1 << 6, "RPM_VALID"),
    (1 << 7, "CALIBRATED"),
    (1 << 8, "RING_OVERFLOW"),
    (1 << 9, "SD_ERROR"),
    (1 << 10, "GPS_TIMEOUT"),
    (1 << 11, "IMU_TIMEOUT"),
    (1 << 12, "BARO_TIMEOUT"),
]


def crc16_ccitt(data):
    """CRC-16/CCITT-FALSE, matching crc16_ccitt() in dapu_log.c."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


class LogFile(object):
    def __init__(self, path):
        with open(path, "rb") as handle:
            self.raw = handle.read()

        if len(self.raw) < HEADER_SIZE:
            raise ValueError("file shorter than the 1 kB header - truncated?")

        magic = self.raw[:16].rstrip(b"\x00")
        if magic != FILE_MAGIC:
            raise ValueError("bad file magic %r - not a DAPU log" % magic)

        (self.version, self.record_size,
         self.base_rate_hz, self.adc_scan_rate_hz) = struct.unpack_from("<IIff", self.raw, 16)

        schema = self.raw[48:HEADER_SIZE].split(b"\x00", 1)[0].decode("ascii")
        fmt_line, names_line = schema.split("\n", 1)
        self.fmt = fmt_line.strip()
        self.names = [n.strip() for n in names_line.strip().split(",")]

        self.struct = struct.Struct(self.fmt)
        if self.struct.size != self.record_size:
            raise ValueError("schema size %d != record size %d in header"
                             % (self.struct.size, self.record_size))
        if len(self.names) != len(self.struct.unpack(b"\x00" * self.struct.size)):
            raise ValueError("schema has %d names but the format yields %d fields"
                             % (len(self.names), len(self.struct.unpack(
                                 b"\x00" * self.struct.size))))

        self.bad_magic = 0
        self.bad_crc = 0

    def records(self, check_crc=True):
        """Yields one dict per valid record, resynchronising on the magic word
        if the file was truncated mid-write by a power cut."""
        size = self.record_size
        offset = HEADER_SIZE
        end = len(self.raw)

        while offset + size <= end:
            chunk = self.raw[offset:offset + size]

            if struct.unpack_from("<I", chunk)[0] != RECORD_MAGIC:
                self.bad_magic += 1
                offset += 4                     # records are 4 byte aligned
                continue

            values = self.struct.unpack(chunk)
            row = dict(zip(self.names, values))

            if check_crc and crc16_ccitt(chunk[:size - 4]) != row["crc16"]:
                self.bad_crc += 1
                offset += size
                continue

            yield row
            offset += size


def decode_status(status):
    names = [name for bit, name in STATUS_BITS if status & bit]
    return "|".join(names) if names else "-"


def summarise(log, rows):
    if not rows:
        print("no valid records")
        return

    first, last = rows[0], rows[-1]

    # t_us wraps every ~71 minutes; unwrap so long runs still measure correctly.
    span_us = 0
    prev = first["t_us"]
    for row in rows:
        delta = (row["t_us"] - prev) & 0xFFFFFFFF
        span_us += delta
        prev = row["t_us"]
    duration = span_us / 1e6

    print("=" * 62)
    print("records       : %d valid" % len(rows))
    if log.bad_magic:
        print("                %d bytes skipped resynchronising" % log.bad_magic)
    if log.bad_crc:
        print("                %d records rejected on CRC" % log.bad_crc)
    print("duration      : %.2f s" % duration)
    print("sample_id     : %d .. %d" % (first["sample_id"], last["sample_id"]))

    lost = (last["sample_id"] - first["sample_id"] + 1) - len(rows)
    if lost > 0:
        print("gaps          : %d records missing (ring overflow or CRC)" % lost)

    print("status (last) : 0x%04X  %s" % (last["status"], decode_status(last["status"])))
    print("-" * 62)
    print("%-24s %10s %10s %8s" % ("acquisition task", "configured", "measured", "error"))
    print("-" * 62)

    log_measured = len(rows) / duration if duration > 0 else 0.0
    print("%-24s %9.1f  %9.2f  %6.1f%%" % (
        "log record (base loop)", log.base_rate_hz, log_measured,
        100.0 * (log_measured - log.base_rate_hz) / log.base_rate_hz))

    for field, label, nominal in RATE_FIELDS:
        if field not in first:
            continue
        count = (last[field] - first[field]) & 0xFFFFFFFF
        measured = count / duration if duration > 0 else 0.0
        error = 100.0 * (measured - nominal) / nominal if nominal else 0.0
        print("%-24s %9.1f  %9.2f  %6.1f%%" % (label, nominal, measured, error))

    print("-" * 62)
    print("microphone sample rate (from header): %.4f Hz" % log.adc_scan_rate_hz)
    print("FFT bin width                       : %.4f Hz"
          % (log.adc_scan_rate_hz / 1024.0))
    print("=" * 62)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("logfile")
    parser.add_argument("--csv", metavar="OUT", help="write every record to a CSV file")
    parser.add_argument("--drop-reserved", action="store_true",
                        help="omit padding/magic/crc columns from the CSV")
    parser.add_argument("--no-crc", action="store_true",
                        help="keep records whose CRC does not match")
    args = parser.parse_args()

    try:
        log = LogFile(args.logfile)
    except (IOError, ValueError) as exc:
        sys.exit("error: %s" % exc)

    rows = list(log.records(check_crc=not args.no_crc))
    summarise(log, rows)

    if args.csv:
        columns = list(log.names)
        if args.drop_reserved:
            columns = [c for c in columns
                       if not c.startswith("reserved") and c not in ("magic", "crc16")]

        with open(args.csv, "w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=columns, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(rows)

        print("wrote %s (%d rows x %d columns)" % (args.csv, len(rows), len(columns)))


if __name__ == "__main__":
    main()
