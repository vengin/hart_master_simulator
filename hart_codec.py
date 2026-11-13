"""HART data encodings shared by the simulator GUI and scenarios.py (no PyQt).

Packed ASCII (HART Common Tables): each character is stored as its 6 low ASCII
bits (ASCII & 0x3F), 4 characters per 3 bytes. On decode, 6-bit values below
0x20 map to 0x40-0x5F ('@', 'A'-'Z', ...), values 0x20-0x3F map to themselves
(' ', '0'-'9', punctuation). Only upper-case 0x20-0x5F characters survive.
"""


def encode_packed_ascii(text: str, nbytes: int) -> bytes:
  """Encode text into `nbytes` (multiple of 3) of HART Packed ASCII.
  The text is upper-cased and padded with spaces / truncated to nbytes*4/3
  characters."""
  if nbytes % 3:
    raise ValueError("packed ASCII length must be a multiple of 3")
  nchars = nbytes * 4 // 3
  s = text.upper().ljust(nchars)[:nchars]
  out = bytearray()
  for i in range(0, nchars, 4):
    c = [ord(ch) & 0x3F for ch in s[i:i + 4]]
    out.append(((c[0] << 2) | (c[1] >> 4)) & 0xFF)
    out.append(((c[1] << 4) | (c[2] >> 2)) & 0xFF)
    out.append(((c[2] << 6) | c[3]) & 0xFF)
  return bytes(out)


def decode_packed_ascii(data: bytes) -> str:
  """Decode HART Packed ASCII (3 bytes -> 4 characters)."""
  out = []
  for i in range(0, len(data) - 2, 3):
    b0, b1, b2 = data[i], data[i + 1], data[i + 2]
    for v in ((b0 >> 2) & 0x3F,
              ((b0 << 4) | (b1 >> 4)) & 0x3F,
              ((b1 << 2) | (b2 >> 6)) & 0x3F,
              b2 & 0x3F):
      out.append(chr(v + 0x40 if v < 0x20 else v))
  return "".join(out)


def encode_date(day: int, month: int, year: int) -> bytes:
  """HART Date: day, month, year - 1900."""
  return bytes([day, month, year - 1900])


def encode_latin1(text: str, nbytes: int) -> bytes:
  """Latin-1 text padded with spaces / truncated (e.g. 32-byte Long Tag)."""
  return text.encode("latin-1")[:nbytes].ljust(nbytes, b" ")
