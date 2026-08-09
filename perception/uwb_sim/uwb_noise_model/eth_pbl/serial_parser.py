#!/usr/bin/env python3
"""Vendored from ETH-PBL/UWB_DualAntenna_AoA (Scripts/serial_parser.py)."""

import gzip
import os

from . import binary_parser


class Frame:
    __slots__ = (
        "serial_timestamp",
        "serial_count",
        "frame_type",
        "sequence_number",
        "toa_data",
        "cir_analysis_ip",
        "cir_analysis_sts1",
        "cir_analysis_sts2",
        "cir",
        "twr_data",
    )

    binary_to_attr = {
        "toa": "toa_data",
        "cir analysis ip": "cir_analysis_ip",
        "cir analysis sts1": "cir_analysis_sts1",
        "cir analysis sts2": "cir_analysis_sts2",
        "cir": "cir",
        "twr": "twr_data",
    }

    def __init__(self):
        self.serial_timestamp = None
        self.serial_count = None
        self.frame_type = None
        self.sequence_number = None
        self.toa_data = None
        self.cir_analysis_ip = None
        self.cir_analysis_sts1 = None
        self.cir_analysis_sts2 = None
        self.cir = None
        self.twr_data = None


def parse_log_file(logfile: str):
    frames = []
    current_frame = Frame()
    compressed = logfile.endswith(".gz")
    file_mode = "rb" if compressed else "rt"

    with open(logfile, file_mode) as fo:
        f = gzip.open(fo, "rt") if compressed else fo
        bad_frame = False
        while True:
            line = f.readline()
            if not line:
                break

            if "New Frame" in line:
                if not bad_frame:
                    frames.append(current_frame)
                current_frame = Frame()
                bad_frame = False
                try:
                    info = line.split(":")
                    current_frame.serial_timestamp = float(info[0])
                    current_frame.frame_type = info[2].strip()
                    current_frame.sequence_number = int(info[3].strip())
                    current_frame.serial_count = len(frames)
                except (IndexError, ValueError):
                    bad_frame = True

            elif "BLOB" in line:
                blob_data = f.readline()
                try:
                    header = line.split("/")
                    title = header[1].strip()
                    version = int(header[2].strip()[1:])
                    decoder = binary_parser.decoders[title]
                    attribute = Frame.binary_to_attr[title]
                    decoded = decoder(blob_data.split(":")[2], version)
                    setattr(current_frame, attribute, decoded)
                except (KeyError, ValueError, IndexError, AttributeError):
                    pass

        frames.append(current_frame)
        if frames:
            del frames[0]

    return frames
