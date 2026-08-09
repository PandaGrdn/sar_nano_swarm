#!/usr/bin/env python3
"""Vendored from ETH-PBL/UWB_DualAntenna_AoA (Scripts/binary_parser.py)."""

import base64
import struct
from collections import namedtuple

type_mapping = {
    "u64": "Q",
    "u32": "L",
    "u16": "H",
    "i16": "h",
    "u8": "B",
}

toa_data = namedtuple(
    "toa_data",
    "cia_diag_1 ip_poa sts1_poa sts2_poa pdoa xtal_offset sts_qual_index sts_qual tdoa "
    "ip_toa ip_toast sts1_toa sts1_toast sts2_toa sts2_toast fp_th_md dgc_decision",
)

cir_analysis_data = namedtuple(
    "cir_analysis_data", "peak power F1 F2 F3 fp_index accum_count"
)

cir_data = namedtuple("cir_data", "cir_ip cir_sts1 cir_sts2")

twr_data = namedtuple(
    "twr_data", "Treply1 Treply2 Tround1 Tround2 dist_mm twr_count rotation"
)


def decode_40bit_int(buffer, negative=False):
    value = (
        buffer[0]
        + (buffer[1] << 8)
        + (buffer[2] << 16)
        + (buffer[3] << 24)
        + (buffer[4] << 32)
    )
    if negative:
        all_one = 0xFFFFFFFFFF
        value = -(value ^ all_one) - 1
    return value


def decode_24bit_int(data):
    value = (data[2] << 16) + (data[1] << 8) + data[0]
    if value & 0x800000:
        value = value ^ 0xFFFFFF - 1
    return value


def decode_48bit_complex_array(data):
    groups = zip(*([iter(data)] * 6), strict=True)
    decoded = []
    for group in groups:
        real = decode_24bit_int(group[:3])
        imag = decode_24bit_int(group[3:])
        decoded.append(real + imag * 1j)
    return decoded


def decode_blob_toa(b64_buffer, version):
    if version != 3:
        raise ValueError(f"Unsupported version: {version}")
    toa_blob_format = "< u32 u16 u16 u16 i16 i16 i16 u8 u8 5u8 5u8 u8 5u8 u8 5u8 u8 u8 u8"
    for k, v in type_mapping.items():
        toa_blob_format = toa_blob_format.replace(k, v)
    data = base64.b64decode(b64_buffer)
    unpacked = struct.unpack(toa_blob_format, data)
    return toa_data(
        cia_diag_1=unpacked[0],
        ip_poa=unpacked[1],
        sts1_poa=unpacked[2],
        sts2_poa=unpacked[3],
        pdoa=unpacked[4],
        xtal_offset=unpacked[5],
        sts_qual_index=unpacked[6],
        sts_qual=unpacked[7],
        tdoa=decode_40bit_int(unpacked[9:14], unpacked[8]),
        ip_toa=decode_40bit_int(unpacked[14:19]),
        ip_toast=unpacked[19],
        sts1_toa=decode_40bit_int(unpacked[20:25]),
        sts1_toast=unpacked[25],
        sts2_toa=decode_40bit_int(unpacked[26:31]),
        sts2_toast=unpacked[31],
        fp_th_md=unpacked[32],
        dgc_decision=unpacked[33],
    )


def decode_blob_cir_analysis(b64_buffer, version):
    if version != 1:
        raise ValueError(f"Unsupported version: {version}")
    fmt = "< u32 u32 u32 u32 u32 u16 u16"
    for k, v in type_mapping.items():
        fmt = fmt.replace(k, v)
    data = base64.b64decode(b64_buffer)
    unpacked = list(struct.unpack(fmt, data))
    unpacked[5] /= 64
    return cir_analysis_data._make(unpacked)


def decode_blob_cir(b64_buffer, version):
    del version
    data = base64.b64decode(b64_buffer)
    bytes_per_symbol = 6
    cir_preamble_start = 0 * bytes_per_symbol
    cir_preamble_length = 1016 * bytes_per_symbol
    cir_sts1_start = 1024 * bytes_per_symbol
    cir_sts2_start = 1536 * bytes_per_symbol
    cir_sts_length = 512 * bytes_per_symbol
    cir_ip_bin = data[cir_preamble_start : cir_preamble_start + cir_preamble_length]
    cir_sts1_bin = data[cir_sts1_start : cir_sts1_start + cir_sts_length]
    cir_sts2_bin = data[cir_sts2_start : cir_sts2_start + cir_sts_length]
    return cir_data(
        decode_48bit_complex_array(cir_ip_bin),
        decode_48bit_complex_array(cir_sts1_bin),
        decode_48bit_complex_array(cir_sts2_bin),
    )


def decode_blob_twr(b64_buffer, version):
    if version != 2:
        raise ValueError(f"Unsupported version: {version}")
    fmt = "< u64 u64 u64 u64 u32 u16 u16"
    for k, v in type_mapping.items():
        fmt = fmt.replace(k, v)
    data = base64.b64decode(b64_buffer)
    return twr_data._make(struct.unpack(fmt, data))


decoders = {
    "toa": decode_blob_toa,
    "cir analysis ip": decode_blob_cir_analysis,
    "cir analysis sts1": decode_blob_cir_analysis,
    "cir analysis sts2": decode_blob_cir_analysis,
    "cir": decode_blob_cir,
    "twr": decode_blob_twr,
}
