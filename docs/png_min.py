"""Lossless PNG re-encoder for the GUI screenshots in this folder.

Tries every PNG row filter (none, sub, up, average, Paeth, adaptive) with two
zlib strategies and keeps the smallest result. Saves 24-bit RGB without alpha
or colour profile. Flat UI screenshots usually compress best with no filter,
while Qt/Pillow default to filtering, so their files are larger.

Every file is checked to be pixel-identical before it is replaced.

Usage:  python docs/png_min.py docs/*.png
Needs:  numpy, Pillow
"""
import os
import struct
import sys
import zlib

import numpy as np
from PIL import Image


def _chunk(ctype: bytes, data: bytes) -> bytes:
  return (struct.pack(">I", len(data)) + ctype + data
          + struct.pack(">I", zlib.crc32(ctype + data) & 0xFFFFFFFF))


def _filtered(a: np.ndarray, ftype: int) -> np.ndarray:
  """Apply PNG filter `ftype` (0-4) to every row of an HxWx3 uint8 image."""
  h, w, c = a.shape
  x = a.astype(np.int16)
  left = np.zeros_like(x)
  left[:, 1:] = x[:, :-1]
  up = np.zeros_like(x)
  up[1:] = x[:-1]
  up_left = np.zeros_like(x)
  up_left[1:, 1:] = x[:-1, :-1]
  if ftype == 0:
    f = x
  elif ftype == 1:
    f = x - left
  elif ftype == 2:
    f = x - up
  elif ftype == 3:
    f = x - (left + up) // 2
  else:
    p = left + up - up_left
    pa, pb, pc = abs(p - left), abs(p - up), abs(p - up_left)
    pred = np.where((pa <= pb) & (pa <= pc), left, np.where(pb <= pc, up, up_left))
    f = x - pred
  return (f & 0xFF).astype(np.uint8).reshape(h, w * c)


def encode(path: str):
  """Return (png_bytes, description) of the smallest lossless RGB encoding."""
  a = np.asarray(Image.open(path).convert("RGB"))
  h, w, _ = a.shape
  rows = {t: _filtered(a, t) for t in range(5)}

  candidates = {f"filter {t}": np.hstack([np.full((h, 1), t, np.uint8), rows[t]])
                for t in range(5)}
  # Adaptive: per row, the filter with the smallest sum of |signed bytes|
  cost = np.stack([np.abs(rows[t].astype(np.int8).astype(np.int32)).sum(1) for t in range(5)])
  best = cost.argmin(0)
  candidates["adaptive"] = np.vstack([
    np.concatenate(([best[y]], rows[best[y]][y])).astype(np.uint8) for y in range(h)])

  results = []
  for name, raw in candidates.items():
    for strategy, sname in ((zlib.Z_DEFAULT_STRATEGY, "default"), (zlib.Z_FILTERED, "filtered")):
      co = zlib.compressobj(9, zlib.DEFLATED, 15, 9, strategy)
      data = co.compress(raw.tobytes()) + co.flush()
      results.append((len(data), f"{name}, zlib {sname}", data))
  _, desc, data = min(results, key=lambda r: r[0])

  png = (b"\x89PNG\r\n\x1a\n"
         + _chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
         + _chunk(b"IDAT", data)
         + _chunk(b"IEND", b""))
  return png, desc


def main(paths):
  if not paths:
    print(__doc__)
    return 1
  for path in paths:
    before = os.path.getsize(path)
    png, desc = encode(path)
    if len(png) >= before:
      print(f"{os.path.basename(path):<52} {before:>8}  already minimal, unchanged")
      continue
    tmp = path + ".tmp"
    with open(tmp, "wb") as f:
      f.write(png)
    same = np.array_equal(np.asarray(Image.open(tmp).convert("RGB")),
                          np.asarray(Image.open(path).convert("RGB")))
    if not same:
      os.remove(tmp)
      print(f"{os.path.basename(path):<52} pixel mismatch - NOT replaced")
      continue
    os.replace(tmp, path)
    print(f"{os.path.basename(path):<52} {before:>8} -> {len(png):>8}  ({desc})")
  return 0


if __name__ == "__main__":
  sys.exit(main(sys.argv[1:]))
