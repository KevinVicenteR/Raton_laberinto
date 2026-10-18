"""Formato flash TRK v1 / RMAP v1 del firmware (solo biblioteca estándar)."""

import math
import struct
import zlib
from pathlib import Path

SIZE = 40


def encode(walls, seen, corridor, cell, side_sin, deadzone):
    header = b"RMAP" + bytes((1, SIZE, SIZE, 0))
    payload = bytearray()
    for x in range(SIZE):
        for y in range(SIZE):
            payload.extend((walls.get((x, y), 0), int((x, y) in seen), 0))
    inner = header + payload
    inner += struct.pack("<I", zlib.crc32(inner))
    record = b"TRK\x01" + struct.pack("<fffI", corridor, cell, side_sin, deadzone) + inner
    return record + struct.pack("<I", zlib.crc32(record))


def decode(record):
    if len(record) != 20 + 12 + 3 * SIZE * SIZE + 4 or record[:4] != b"TRK\x01":
        raise ValueError("mapa: tamaño o versión")
    if zlib.crc32(record[:-4]) != struct.unpack("<I", record[-4:])[0]:
        raise ValueError("mapa: CRC de geometría")
    corridor, cell, side_sin, deadzone = struct.unpack("<fffI", record[4:20])
    if not all(map(math.isfinite, (corridor, cell, side_sin))) or not (
        128 <= corridor <= 388
        and abs(cell - corridor - 12) <= 0.1
        and 0.3 <= side_sin <= 0.98
        and deadzone <= 150
    ):
        raise ValueError("mapa: geometría")
    inner = record[20:-4]
    if (
        inner[:8] != b"RMAP" + bytes((1, SIZE, SIZE, 0))
        or zlib.crc32(inner[:-4]) != struct.unpack("<I", inner[-4:])[0]
    ):
        raise ValueError("mapa: CRC o versión de celdas")
    walls, seen = {}, set()
    for x in range(SIZE):
        for y in range(SIZE):
            at = 8 + 3 * (x * SIZE + y)
            bits, observed, _ = inner[at : at + 3]
            if bits > 15 or observed > 1:
                raise ValueError("mapa: celda inválida")
            walls[x, y] = bits
            if observed:
                seen.add((x, y))
    for (x, y), bits in walls.items():
        if x + 1 < SIZE and bool(bits & 2) != bool(walls[x + 1, y] & 8):
            raise ValueError("mapa: pared asimétrica")
        if y + 1 < SIZE and bool(bits & 1) != bool(walls[x, y + 1] & 4):
            raise ValueError("mapa: pared asimétrica")
    return walls, seen, (corridor, cell, side_sin, deadzone)


def save(path, *args):
    record = encode(*args)
    decode(record)
    target = Path(path)
    temporary = target.with_suffix(target.suffix + ".tmp")
    temporary.write_bytes(record)
    temporary.replace(target)


def load(path):
    return decode(Path(path).read_bytes())
