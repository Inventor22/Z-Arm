"""Read-only inspection of Windows PE files (DLL/EXE) using only the standard library.

The file is parsed as bytes and nothing in it is executed, so this is safe to
run on any vendor binary and on any operating system.
"""

from __future__ import annotations

import os
import struct
from dataclasses import dataclass
from typing import List, Tuple, Union

MACHINE_NAMES = {0x014C: "x86", 0x8664: "x64", 0xAA64: "arm64", 0x01C4: "arm"}
_IMAGE_FILE_DLL = 0x2000


class PeFormatError(ValueError):
    """The file is not a PE image this reader understands."""


@dataclass(frozen=True)
class PeInfo:
    path: str
    machine: str
    is_dll: bool
    is_dotnet: bool
    exports: Tuple[str, ...]


def _unpack(fmt: str, data: bytes, offset: int) -> tuple:
    try:
        return struct.unpack_from(fmt, data, offset)
    except struct.error as exc:
        raise PeFormatError(f"truncated PE data at offset {offset:#x}") from exc


def read_pe_info(path: Union[str, "os.PathLike[str]"]) -> PeInfo:
    with open(path, "rb") as handle:
        data = handle.read()
    if data[:2] != b"MZ":
        raise PeFormatError(f"{path}: not a PE file (no MZ header)")
    pe_offset = _unpack("<I", data, 0x3C)[0]
    if data[pe_offset : pe_offset + 4] != b"PE\0\0":
        raise PeFormatError(f"{path}: not a PE file (no PE signature)")
    machine_id, section_count = _unpack("<HH", data, pe_offset + 4)
    optional_size, characteristics = _unpack("<HH", data, pe_offset + 20)
    optional = pe_offset + 24
    magic = _unpack("<H", data, optional)[0]
    if magic == 0x10B:
        directories = optional + 96
    elif magic == 0x20B:
        directories = optional + 112
    else:
        raise PeFormatError(f"{path}: unknown optional header magic {magic:#x}")
    export_rva = _unpack("<I", data, directories)[0]
    clr_rva = _unpack("<I", data, directories + 14 * 8)[0]

    sections: List[Tuple[int, int, int, int]] = []
    table = optional + optional_size
    for index in range(section_count):
        virtual_size, virtual_address, raw_size, raw_pointer = _unpack("<IIII", data, table + index * 40 + 8)
        sections.append((virtual_address, max(virtual_size, raw_size), raw_pointer, raw_size))

    def to_offset(rva: int) -> int:
        for virtual_address, size, raw_pointer, _ in sections:
            if virtual_address <= rva < virtual_address + size:
                return rva - virtual_address + raw_pointer
        raise PeFormatError(f"{path}: RVA {rva:#x} is outside every section")

    def c_string(rva: int) -> str:
        start = to_offset(rva)
        end = data.find(b"\0", start)
        if end < 0:
            raise PeFormatError(f"{path}: unterminated string at RVA {rva:#x}")
        return data[start:end].decode("ascii", "replace")

    exports: List[str] = []
    if export_rva:
        directory = to_offset(export_rva)
        name_count = _unpack("<I", data, directory + 24)[0]
        names_rva = _unpack("<I", data, directory + 32)[0]
        names_offset = to_offset(names_rva)
        for index in range(name_count):
            exports.append(c_string(_unpack("<I", data, names_offset + 4 * index)[0]))

    return PeInfo(
        path=os.fspath(path),
        machine=MACHINE_NAMES.get(machine_id, f"unknown({machine_id:#x})"),
        is_dll=bool(characteristics & _IMAGE_FILE_DLL),
        is_dotnet=bool(clr_rva),
        exports=tuple(exports),
    )
