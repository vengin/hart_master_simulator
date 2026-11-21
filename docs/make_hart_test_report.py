"""Build the HART communication functional test report (.docx) from a
HART Master Simulator log, in the style of the BUOS document template.

    python docs/make_hart_test_report.py --log "logs/<log>.txt"

Every frame is decoded again here, byte by byte, and every checksum is
re-computed, independent of the verdicts written by the simulator.
"""
import argparse
import copy
import datetime
import hashlib
import os
import re
import struct
import sys

import docx
from docx.enum.text import WD_BREAK
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Pt

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from hart_codec import decode_packed_ascii                       # noqa: E402
from hart_master_simulator import (UNIT_CODES, RESPONSE_CODES,   # noqa: E402
                                   DEVICE_STATUS_BITS, KNOWN_MANUFACTURERS_16)
import scenarios as SCN                                           # noqa: E402

TOOL = "HART Master Simulator v1.1.10"

# --------------------------------------------------------------- doc settings
DOC = {
  "number": "BUOS-HTR-001",
  "rev": "A",
  "date": "2026-11-17",
  "title": "HART Communication Functional Test Report",
  "subtitle": "HART command request/response record – unit functional testing, firmware V{fw}",
  "status": "Released for review",
  "phase": "Verification – unit functional test (BUOS-TP-001)",
  "tuv_items": "To be assigned",
  "author_role": "Author (Firmware / Test)",
}

OLD = {"number": "BUOS-HWAS-001", "date": "2026-11-12",
       "title": "Hardware Architecture Specification"}

# --------------------------------------------------------------- HART tables
CMD_NAMES = {
  0: "Read Unique Identifier", 1: "Read Primary Variable",
  2: "Read Loop Current and Percent of Range",
  3: "Read Dynamic Variables and Loop Current", 6: "Write Polling Address",
  7: "Read Loop Configuration", 8: "Read Dynamic Variable Classifications",
  9: "Read Device Variables with Status",
  11: "Read Unique Identifier Associated with Tag", 12: "Read Message",
  13: "Read Tag, Descriptor, Date",
  14: "Read Primary Variable Transducer Information",
  15: "Read Device Information", 16: "Read Final Assembly Number",
  17: "Write Message", 18: "Write Tag, Descriptor, Date",
  19: "Write Final Assembly Number", 20: "Read Long Tag",
  21: "Read Unique Identifier Associated with Long Tag", 22: "Write Long Tag",
  33: "Read Device Variables", 34: "Write Primary Variable Damping Value",
  35: "Write Primary Variable Range Values",
  36: "Set Primary Variable Upper Range Value",
  37: "Set Primary Variable Lower Range Value",
  38: "Reset Configuration Changed Flag", 40: "Enter/Exit Fixed Current Mode",
  41: "Perform Self Test", 43: "Set Primary Variable Zero",
  44: "Write Primary Variable Units", 45: "Trim Loop Current Zero",
  46: "Trim Loop Current Gain", 47: "Write Primary Variable Transfer Function",
  48: "Read Additional Device Status",
  49: "Write Primary Variable Transducer Serial Number",
  50: "Read Dynamic Variable Assignments",
  51: "Write Dynamic Variable Assignments",
  54: "Read Device Variable Information",
  59: "Write Number of Response Preambles",
  99: "(undefined command number)",
  108: "Write Burst Mode Command Number", 109: "Burst Mode Control",
}
UNIVERSAL = {0, 1, 2, 3, 6, 7, 8, 9, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
             21, 22, 38, 48}

def cmd_class(c):
  if c in UNIVERSAL:
    return "Universal"
  if c == 99:
    return "Invalid (negative test)"
  return "Common Practice"

# Response codes whose meaning depends on the command
RC_OVERRIDE = {
  38: {9: "Configuration change counter mismatch"},
  59: {3: "Passed parameter too large", 4: "Passed parameter too small"},
}

def rc_text(cmd, rc):
  return RC_OVERRIDE.get(cmd, {}).get(rc) or RESPONSE_CODES.get(rc, f"Response code {rc}")

DELIMITERS = {
  0x02: "STX, short (polling) address – master to field device",
  0x82: "STX, long (unique) address – master to field device",
  0x06: "ACK, short (polling) address – field device to master",
  0x86: "ACK, long (unique) address – field device to master",
}

CLASSIFICATION = {0: "Not classified", 64: "Temperature", 65: "Pressure",
                  66: "Volumetric flow", 67: "Velocity", 68: "Volume", 69: "Length",
                  70: "Time", 71: "Mass"}
WP_CODES = {0: "Not write protected", 1: "Write protected", 250: "Not used",
            251: "None (write protect not implemented)"}
TRANSFER = {0: "Linear", 1: "Square root", 250: "Not used", 251: "None"}
PROFILE = {1: "HART process automation device"}
CMD48_BYTES = {6: "Extended device status", 7: "Device operating mode",
               8: "Standardized status 0", 9: "Standardized status 1",
               10: "Analog channel saturated", 11: "Standardized status 2",
               12: "Standardized status 3", 13: "Analog channel fixed"}


# --------------------------------------------------------------- value formats
def f32(b):
  if b == b"\x7f\xa0\x00\x00":
    return "NaN (0x7FA00000, not applicable)"
  v = struct.unpack(">f", b)[0]
  return f"{v:g}"

def unit(b):
  return f"{b[0]} = {UNIT_CODES.get(b[0], 'unknown')}"

def pascii(b):
  return '"' + decode_packed_ascii(b).rstrip() + '"'

def latin(b):
  return '"' + b.decode("latin-1").rstrip() + '"'

def u(b):
  v = int.from_bytes(b, "big")
  return f"{v} (0x{v:06X})" if len(b) == 3 else str(v)

PDS = {0: "bad", 1: "poor accuracy", 2: "fixed (manual)", 3: "good"}
LIMIT = {0: "not limited", 1: "low limited", 2: "high limited", 3: "constant"}

def dvstatus(b):
  v = b[0]
  return (f"0x{v:02X}: process data {PDS[v >> 6]}, {LIMIT[(v >> 4) & 3]}"
          + (", more device variable status available" if v & 8 else "")
          + (f", device-family bits 0x{v & 7:X}" if v & 7 else ""))

def hx(b):
  return "0x" + b.hex().upper()

def date3(b):
  return f"{b[0]:02d}.{b[1]:02d}.{1900 + b[2]} (day.month.year)"

def hwrev(b):
  return f"HW revision {b[0] >> 3}; physical signalling code {b[0] & 7}" + \
         (" (Bell 202 current)" if b[0] & 7 == 0 else "")

def mfr(b):
  v = int.from_bytes(b, "big")
  return f"0x{v:04X} ({KNOWN_MANUFACTURERS_16.get(v, 'unknown')})"

def code(table):
  return lambda b: f"{b[0]} ({table.get(b[0], 'see device documentation')})"

def loopmode(b):
  return f"{b[0]} ({'enabled' if b[0] else 'disabled'})"

def stat_bits(v):
  names = [n for bit, n in DEVICE_STATUS_BITS.items() if v & bit]
  return ", ".join(names) if names else "no bits set"

def timestamp(b):
  v = int.from_bytes(b, "big")
  return f"{v} (1/32 ms units = {v / 32000.0:.3f} s)"

def acq(b):
  v = int.from_bytes(b, "big")
  return "0xFFFFFFFF (not calculated)" if v == 0xFFFFFFFF else f"{v / 32.0:.3f} ms"


# --------------------------------------------------------------- data layouts
# Each layout = list of (field name, length, formatter). length None = repeat
# block handled by a function.
CMD0 = [
  ("Expansion code (fixed 254)", 1, u), ("Expanded device type", 2, hx),
  ("Min. number of request preambles", 1, u), ("Universal command revision", 1, u),
  ("Device revision", 1, u), ("Software revision", 1, u),
  ("HW revision / physical signalling code", 1, hwrev), ("Flags", 1, hx),
  ("Device ID", 3, hx), ("Min. number of response preambles", 1, u),
  ("Max. number of device variables", 1, u),
  ("Configuration change counter", 2, u), ("Extended field device status", 1, hx),
  ("Manufacturer ID", 2, mfr), ("Private label distributor", 2, mfr),
  ("Device profile", 1, code(PROFILE)),
]
TDD = [("Tag (packed ASCII, 8 char)", 6, pascii),
       ("Descriptor (packed ASCII, 16 char)", 12, pascii), ("Date", 3, date3)]
MSG = [("Message (packed ASCII, 32 char)", 24, pascii)]
LTAG = [("Long tag (ISO Latin-1, 32 char)", 32, latin)]
FAN = [("Final assembly number", 3, u)]
DV4 = [("PV device variable code", 1, u), ("SV device variable code", 1, u),
       ("TV device variable code", 1, u), ("QV device variable code", 1, u)]

def repeat(block, name_fmt, tail=()):
  """Repeated slots of `block` (fields of name_fmt % i), then `tail` fields."""
  def build(data):
    blen = sum(n for _, n, _ in block)
    tlen = sum(n for _, n, _ in tail)
    slots = max(0, (len(data) - tlen) // blen) if blen else 0
    out = []
    for i in range(slots):
      out += [(name_fmt % i + " – " + f, n, fn) for f, n, fn in block]
    return out + list(tail)
  return build

def lead_repeat(lead, block, name_fmt, tail=()):
  def build(data):
    llen = sum(n for _, n, _ in lead)
    return list(lead) + repeat(block, name_fmt, tail)(data[llen:])
  return build

def cmd3_layout(data):
  out = [("Loop current (mA)", 4, f32)]
  for i, v in enumerate(["PV", "SV", "TV", "QV"]):
    if 4 + 5 * (i + 1) <= len(data):
      out += [(f"{v} units", 1, unit), (f"{v} value", 4, f32)]
  return out

def cmd48_layout(data):
  return [(CMD48_BYTES.get(i, f"Device-specific status {i}"), 1, hx)
          for i in range(len(data))]

def cmd108_req(data):
  if len(data) == 1:
    return [("Burst command number (HART 5/6 form)", 1, u)]
  return [("Burst command number", 2, u), ("Burst message", 1, u)]

def cmd109_req(data):
  return [("Burst mode control code", 1, u), ("Burst message", 1, u)][:max(1, len(data))]

RESP = {
  0: CMD0, 11: CMD0, 21: CMD0,
  1: [("PV units", 1, unit), ("PV value", 4, f32)],
  2: [("Loop current (mA)", 4, f32), ("PV percent of range (%)", 4, f32)],
  3: cmd3_layout,
  6: [("Polling address", 1, u), ("Loop current mode", 1, loopmode)],
  7: [("Polling address", 1, u), ("Loop current mode", 1, loopmode)],
  8: [("PV classification", 1, code(CLASSIFICATION)), ("SV classification", 1, code(CLASSIFICATION)),
      ("TV classification", 1, code(CLASSIFICATION)), ("QV classification", 1, code(CLASSIFICATION))],
  9: lead_repeat([("Extended field device status", 1, hx)],
                 [("device variable code", 1, u), ("classification", 1, code(CLASSIFICATION)),
                  ("units", 1, unit), ("value", 4, f32), ("device variable status", 1, dvstatus)],
                 "Slot %d", tail=[("Slot 0 data time stamp", 4, timestamp)]),
  12: MSG, 17: MSG, 13: TDD, 18: TDD,
  14: [("Transducer serial number", 3, u), ("Limits / min. span units", 1, unit),
       ("Upper transducer limit", 4, f32), ("Lower transducer limit", 4, f32),
       ("Minimum span", 4, f32)],
  15: [("PV alarm selection code", 1, u), ("PV transfer function", 1, code(TRANSFER)),
       ("PV range values units", 1, unit), ("PV upper range value", 4, f32),
       ("PV lower range value", 4, f32), ("PV damping value (s)", 4, f32),
       ("Write protect code", 1, code(WP_CODES)), ("Reserved (250)", 1, u),
       ("PV analog channel flags", 1, hx)],
  16: FAN, 19: FAN, 20: LTAG, 22: LTAG,
  33: repeat([("device variable code", 1, u), ("units", 1, unit), ("value", 4, f32)], "Slot %d"),
  34: [("PV damping value (s)", 4, f32)],
  35: [("Range values units", 1, unit), ("Upper range value", 4, f32), ("Lower range value", 4, f32)],
  38: [("Configuration change counter", 2, u)],
  40: [("Actual fixed current (mA)", 4, f32)],
  44: [("PV units", 1, unit)],
  45: [("Actual measured current (mA)", 4, f32)],
  46: [("Actual measured current (mA)", 4, f32)],
  47: [("PV transfer function", 1, code(TRANSFER))],
  48: cmd48_layout,
  49: [("PV transducer serial number", 3, u)],
  50: DV4, 51: DV4,
  54: [("Device variable code", 1, u), ("Transducer serial number", 3, u),
       ("Limits / min. span units", 1, unit), ("Upper transducer limit", 4, f32),
       ("Lower transducer limit", 4, f32), ("Damping value (s)", 4, f32),
       ("Minimum span", 4, f32), ("Classification", 1, code(CLASSIFICATION)),
       ("Device family", 1, u), ("Acquisition period", 4, acq), ("Properties", 1, hx)],
  59: [("Number of response preambles", 1, u)],
  108: cmd108_req, 109: cmd109_req,
}
REQ = {
  6: [("Polling address", 1, u), ("Loop current mode", 1, loopmode)],
  9: repeat([("device variable code", 1, u)], "Slot %d"),
  11: [("Tag (packed ASCII, 8 char)", 6, pascii)],
  17: MSG, 18: TDD, 19: FAN, 21: LTAG, 22: LTAG,
  33: repeat([("device variable code", 1, u)], "Slot %d"),
  34: [("PV damping value (s)", 4, f32)],
  35: [("Range values units", 1, unit), ("Upper range value", 4, f32), ("Lower range value", 4, f32)],
  38: [("Configuration change counter", 2, u)],
  40: [("Fixed current level (mA, 0 = exit)", 4, f32)],
  44: [("PV units", 1, unit)],
  45: [("Measured current (mA)", 4, f32)],
  46: [("Measured current (mA)", 4, f32)],
  47: [("PV transfer function", 1, code(TRANSFER))],
  49: [("PV transducer serial number", 3, u)],
  51: DV4,
  54: [("Device variable code", 1, u)],
  59: [("Number of response preambles", 1, u)],
  108: cmd108_req, 109: cmd109_req,
}


def layout_rows(layout, data, off):
  """Split `data` (starting at frame offset `off`) into field rows:
  (byte range, field, raw hex, value)."""
  if callable(layout):
    layout = layout(data)
  rows, i = [], 0
  for name, n, fn in layout or []:
    if i >= len(data):
      break
    chunk = data[i:i + n]
    if len(chunk) < n:
      rows.append((brange(off + i, len(chunk)), name, sp(chunk),
                   f"incomplete field ({len(chunk)} of {n} bytes)"))
    else:
      rows.append((brange(off + i, n), name, sp(chunk), fn(chunk)))
    i += n
  if i < len(data):
    rest = data[i:]
    rows.append((brange(off + i, len(rest)), "Data" if not layout else "Additional data",
                 sp(rest), f"{len(rest)} byte(s)"))
  return rows


def sp(b):
  return " ".join(f"{x:02X}" for x in b)

def brange(start, n):
  return str(start) if n == 1 else f"{start}–{start + n - 1}"


# --------------------------------------------------------------- frame decode
def xor(b):
  c = 0
  for x in b:
    c ^= x
  return c

def split_frame(raw):
  """Byte-level fields of a frame. Returns dict or None."""
  i = 0
  while i < len(raw) and raw[i] == 0xFF:
    i += 1
  if i + 4 > len(raw):
    return None
  d = raw[i]
  alen = 5 if d & 0x80 else 1
  a = i + 1
  c = a + alen
  bc = raw[c + 1]
  data = raw[c + 2:c + 2 + bc]
  cs = raw[c + 2 + bc] if c + 2 + bc < len(raw) else None
  return {"pre": i, "delim": d, "addr": raw[a:a + alen], "a_off": a, "cmd": raw[c],
          "c_off": c, "bc": bc, "data": data, "d_off": c + 2, "cs": cs,
          "cs_off": c + 2 + bc, "cs_calc": xor(raw[i:c + 2 + bc]),
          "complete": len(data) == bc and cs is not None, "long": bool(d & 0x80)}


def addr_text(f):
  a = f["addr"]
  master = "primary master" if a[0] & 0x80 else "secondary master"
  burst = "; burst bit set" if a[0] & 0x40 else ""
  if not f["long"]:
    return f"Bit 7 = {a[0] >> 7}: {master}{burst}; polling address {a[0] & 0x3F}"
  if not (a[0] & 0x3F) and not any(a[1:]):
    return f"Bit 7 = {a[0] >> 7}: {master}{burst}; broadcast address (all 38 bits 0)"
  return (f"Bit 7 = {a[0] >> 7}: {master}{burst}; expanded device type bits "
          f"0x{a[0] & 0x3F:02X} {a[1]:02X}, device ID {a[2]:02X} {a[3]:02X} {a[4]:02X}")


def frame_rows(raw, is_req):
  """Field table rows for a complete frame."""
  f = split_frame(raw)
  rows = [(brange(0, f["pre"]), "Preamble", f"FF ×{f['pre']}", f"{f['pre']} preamble bytes"),
          (str(f["pre"]), "Delimiter", f"{f['delim']:02X}",
           DELIMITERS.get(f["delim"], "unexpected delimiter")),
          (brange(f["a_off"], len(f["addr"])), "Address", sp(f["addr"]), addr_text(f)),
          (str(f["c_off"]), "Command", f"{f['cmd']:02X}",
           f"{f['cmd']} – {CMD_NAMES.get(f['cmd'], 'unknown')}"),
          (str(f["c_off"] + 1), "Byte count", f"{f['bc']:02X}",
           f"{f['bc']} byte(s) follow" + ("" if is_req else " (2 status bytes + data)"))]
  data, off = f["data"], f["d_off"]
  if is_req:
    rows += layout_rows(REQ.get(f["cmd"]), data, off) if data else []
  else:
    rc, st = data[0], data[1]
    rc_meaning = ("communication error summary" if rc & 0x80 else rc_text(f["cmd"], rc))
    rows.append((str(off), "Response code", f"{rc:02X}", f"{rc} – {rc_meaning}"))
    rows.append((str(off + 1), "Field device status", f"{st:02X}", stat_bits(st)))
    if rc == 0 or len(data) > 2:
      rows += layout_rows(RESP.get(f["cmd"]), data[2:], off + 2)
  ok = f["cs"] == f["cs_calc"]
  rows.append((str(f["cs_off"]), "Checksum", f"{f['cs']:02X}",
               ("valid" if ok else f"INVALID – correct value would be {f['cs_calc']:02X}")
               + " (XOR of delimiter through last data byte)"))
  return rows


# --------------------------------------------------------------- log parsing
HEX_LINE = re.compile(r"^  ([0-9A-F]{2})( [0-9A-F]{2})*\s*$")

def parse_log(path):
  lines = open(path, encoding="utf-8").read().splitlines()
  scns, cur, step, meta = [], None, None, {"skipped": "", "port": ""}
  i = 0
  while i < len(lines):
    ln = lines[i]
    m = re.match(r"  > ═══ SCENARIO (\d+) START \[(\d+)/(\d+)\]: (.*) ═══", ln)
    if m:
      cur = {"no": int(m.group(1)), "seq": int(m.group(2)), "title": m.group(4).strip(),
             "steps": [], "line": i + 1, "done": ""}
      scns.append(cur)
    elif (m := re.match(r"  > ═══ SCENARIO (\d+) DONE: (.*) ═══", ln)):
      cur["done"] = m.group(2)
    elif (m := re.match(r"  >   \[(\d+)\] (.*)", ln)):
      step = {"n": int(m.group(1)), "label": m.group(2).strip(), "tx": None, "rx": None,
              "result": "", "info": "", "line": i + 1, "scn": cur["no"],
              "id": f"S{cur['no']:02d}-T{int(m.group(1)):03d}"}
      cur["steps"].append(step)
    elif ln.startswith(("──▶ TX", "◀── RX")):
      j, hexs = i + 1, []
      while j < len(lines) and HEX_LINE.match(lines[j]):
        hexs.append(lines[j].strip())
        j += 1
      b = bytes.fromhex(" ".join(hexs))
      step["tx" if ln.startswith("──▶") else "rx"] = b
      step["tx_line" if ln.startswith("──▶") else "rx_line"] = i + 2
      i = j - 1
    elif (m := re.match(r"  >      Result: (.*)", ln)):
      res = m.group(1)
      if " | info: " in res:
        res, step["info"] = res.split(" | info: ", 1)
      step["result"] = res.strip()
      step["end_line"] = i + 1
    elif "Skipped (operator interaction" in ln:
      meta["skipped"] = ln.split(":", 1)[1].strip()
    elif (m := re.match(r"  > Connected to (\S+)", ln)):
      meta["port"] = m.group(1)
    i += 1
  for s in scns:
    for st in s["steps"]:
      f = split_frame(st["tx"])
      st["cmd"] = f["cmd"]
      st["txf"] = f
      st["rxf"] = split_frame(st["rx"]) if st["rx"] else None
      if st["rxf"]:
        st["rc"], st["status"] = st["rxf"]["data"][0], st["rxf"]["data"][1]
      else:
        st["rc"] = st["status"] = None
  return scns, meta, len(lines)


def short_result(st):
  r = st["result"]
  if "silent" in r:
    why = r.split("silent", 1)[1].strip(" ()-")
    return f"PASS – no response ({why})"
  m = re.match(r"(PASS|FAIL)\b", r)
  return m.group(1) if m else r

def variant(st):
  f = st["txf"]
  a = f["addr"][0]
  master = "P" if a & 0x80 else "S"
  if not f["long"]:
    addr = f"short {a & 0x3F}"
  elif not (a & 0x3F) and not any(f["addr"][1:]):
    addr = "long, broadcast"
  else:
    addr = "long"
  pre = f["pre"]
  extra = "" if f["cs"] == f["cs_calc"] else ", bad checksum"
  return f"{addr}, {master}, {pre} pre.{extra}"


# --------------------------------------------------------------- docx helpers
NAVY, LIGHT, HEXFILL = "1D4F6E", "E6EDF2", "F2F2F2"
W_TEXT = 10080

def shade(cell, fill):
  tcPr = cell._tc.get_or_add_tcPr()
  sh = OxmlElement("w:shd")
  sh.set(qn("w:val"), "clear")
  sh.set(qn("w:color"), "auto")
  sh.set(qn("w:fill"), fill)
  tcPr.append(sh)

def set_width(cell, w):
  tcPr = cell._tc.get_or_add_tcPr()
  tcW = tcPr.find(qn("w:tcW"))
  if tcW is None:
    tcW = OxmlElement("w:tcW")
    tcPr.insert(0, tcW)
  tcW.set(qn("w:w"), str(w))
  tcW.set(qn("w:type"), "dxa")

def para_fmt(p, after=20, before=None, keep=False):
  pf = p.paragraph_format
  pf.space_after = Pt(after / 20)
  if before is not None:
    pf.space_before = Pt(before / 20)
  if keep:
    pf.keep_with_next = True

def add_run(p, text, bold=False, italic=False, size=16, color=None, mono=False):
  r = p.add_run(text)
  r.bold, r.italic = bold or None, italic or None
  r.font.size = Pt(size / 2)
  if color:
    r.font.color.rgb = docx.shared.RGBColor.from_string(color)
  if mono:
    r.font.name = "Consolas"
    r._element.rPr.rFonts.set(qn("w:hAnsi"), "Consolas")
    r._element.rPr.rFonts.set(qn("w:cs"), "Consolas")
  return r

def cell_text(cell, text, bold=False, size=16, color=None, mono=False, italic=False):
  p = cell.paragraphs[0]
  para_fmt(p)
  lines = str(text).split("\n")
  for k, t in enumerate(lines):
    r = add_run(p, t, bold=bold, size=size, color=color, mono=mono, italic=italic)
    if k < len(lines) - 1:
      r.add_break()

def row_flags(row, header=False):
  trPr = row._tr.get_or_add_trPr()
  trPr.append(OxmlElement("w:cantSplit"))
  if header:
    trPr.append(OxmlElement("w:tblHeader"))
  jc = OxmlElement("w:jc")
  jc.set(qn("w:val"), "center")
  trPr.append(jc)


class Builder:
  def __init__(self, doc):
    self.d = doc
    self.body = doc.element.body
    self.sect = self.body[-1]          # final sectPr stays last
    self.tables = {}                   # section number -> table counter
    self.section = 0

  def _move(self, el):
    self.sect.addprevious(el)

  def heading(self, text, level):
    p = self.d.add_paragraph(text, style=f"Heading {level}")
    self._move(p._p)
    if level == 1:
      self.section += 1 if not text.startswith("Appendix") else 0
    return p

  def para(self, text="", italic=False, size=None, after=None, keep=False, bold=False):
    p = self.d.add_paragraph()
    self._move(p._p)
    if text:
      r = p.add_run(text)
      r.italic, r.bold = italic or None, bold or None
      if size:
        r.font.size = Pt(size / 2)
    if after is not None:
      para_fmt(p, after)
    if keep:
      p.paragraph_format.keep_with_next = True
    return p

  def bullet(self, text):
    p = self.d.add_paragraph(text, style="List Bullet")
    self._move(p._p)
    return p

  def source(self, text):
    p = self.para()
    add_run(p, text, italic=True, size=15)

  def caption(self, key, text):
    n = self.tables.get(key, 0) + 1
    self.tables[key] = n
    p = self.para(after=40, keep=True)
    add_run(p, f"Table {key}-{n} {text}", bold=True, size=17)
    return f"Table {key}-{n}"

  def page_break(self):
    p = self.para()
    p.add_run().add_break(WD_BREAK.PAGE)

  def table(self, header, rows, widths, label_col=False, mono_cols=(), size=16,
            header_size=None):
    t = self.d.add_table(rows=0, cols=len(widths))
    self._move(t._tbl)
    t.style = self.d.styles["Table Grid"]
    tblPr = t._tbl.tblPr
    jc = OxmlElement("w:jc")
    jc.set(qn("w:val"), "center")
    tblPr.find(qn("w:tblW")).addnext(jc)           # schema order: tblW, jc, ...
    lay = OxmlElement("w:tblLayout")
    lay.set(qn("w:type"), "fixed")
    look = tblPr.find(qn("w:tblLook"))
    if look is not None:
      look.addprevious(lay)                         # ..., tblLayout, tblLook
    else:
      tblPr.append(lay)
    grid = t._tbl.tblGrid
    for gc, w in zip(grid.findall(qn("w:gridCol")), widths):
      gc.set(qn("w:w"), str(w))
    if header:
      r = t.add_row()
      row_flags(r, header=True)
      for c, h, w in zip(r.cells, header, widths):
        set_width(c, w)
        shade(c, NAVY)
        cell_text(c, h, bold=True, size=header_size or size, color="FFFFFF")
    for vals in rows:
      r = t.add_row()
      row_flags(r)
      for k, (c, v, w) in enumerate(zip(r.cells, vals, widths)):
        set_width(c, w)
        if label_col and k == 0:
          shade(c, LIGHT)
          cell_text(c, v, bold=True, size=size)
        else:
          cell_text(c, v, size=size, mono=k in mono_cols)
    self.para(after=0)
    return t

  def hexbox(self, title, raw):
    p = self.para(after=20, keep=True)
    add_run(p, title, bold=True, size=16)
    t = self.d.add_table(rows=1, cols=1)
    self._move(t._tbl)
    t.style = self.d.styles["Table Grid"]
    c = t.rows[0].cells[0]
    set_width(c, W_TEXT)
    shade(c, HEXFILL)
    row_flags(t.rows[0])
    cell_text(c, sp(raw) if raw else "(no response)", size=16, mono=bool(raw))
    self.para(after=0)

  def toc(self):
    p = self.para()
    p.style = self.d.styles["toc 1"]
    def fld(kind, text=None):
      r = OxmlElement("w:r")
      if kind == "instr":
        it = OxmlElement("w:instrText")
        it.set(qn("xml:space"), "preserve")
        it.text = text
        r.append(it)
      elif kind == "text":
        t = OxmlElement("w:t")
        t.text = text
        r.append(t)
      else:
        fc = OxmlElement("w:fldChar")
        fc.set(qn("w:fldCharType"), kind)
        if kind == "begin":
          fc.set(qn("w:dirty"), "true")
        r.append(fc)
      p._p.append(r)
    fld("begin")
    fld("instr", ' TOC \\o "1-2" \\h \\z \\u ')
    fld("separate")
    fld("text", "Press F9 (or right-click > Update Field) to build the table of contents.")
    fld("end")


# --------------------------------------------------------------- template prep
def replace_text(part_element, mapping):
  for t in part_element.iter(qn("w:t")):
    if t.text:
      for a, b in mapping.items():
        t.text = t.text.replace(a, b)

def prepare_template(path, fw, log_name, log_sha):
  d = docx.Document(path)
  mapping = {OLD["number"]: DOC["number"], OLD["date"]: DOC["date"],
             OLD["title"]: DOC["title"]}
  for s in d.sections:
    for hf in (s.header, s.footer, s.first_page_header, s.first_page_footer,
               s.even_page_header, s.even_page_footer):
      replace_text(hf._element, mapping)

  paras = d.paragraphs
  # cover block: title, document title, subtitle, product line
  paras[2].runs[0].text = DOC["title"]
  for r in paras[3].runs[1:]:
    r.text = ""
  paras[3].runs[0].text = DOC["subtitle"].format(fw=fw)

  ctl, appr, hist = d.tables[0], d.tables[1], d.tables[2]
  vals = {
    "Document number": DOC["number"],
    "Revision / date": f"Rev {DOC['rev']}, {DOC['date']}",
    "Status": DOC["status"],
    "Lifecycle phase": DOC["phase"],
    "TÜV list items": DOC["tuv_items"],
  }
  for row in ctl.rows:
    key = row.cells[0].text.strip()
    if key in vals:
      set_cell(row.cells[1], vals[key])
  set_cell(appr.rows[1].cells[0], DOC["author_role"])
  set_cell(appr.rows[1].cells[1], "")
  set_cell(hist.rows[1].cells[0], DOC["rev"])
  set_cell(hist.rows[1].cells[1], DOC["date"])
  set_cell(hist.rows[1].cells[2],
           f"Initial release for review. Generated from test log {log_name} "
           f"(SHA-256 {log_sha[:16]}…) with firmware V{fw}.")
  set_cell(hist.rows[1].cells[3], "")

  # Drop everything after the "Contents" heading (keep the final sectPr)
  body = d.element.body
  start = None
  for el in list(body):
    if start is None:
      if el.tag == qn("w:p") and "".join(el.xpath(".//w:t/text()")).strip() == "Contents":
        start = el
      continue
    if el.tag != qn("w:sectPr"):
      body.remove(el)

  # Ask Word to refresh the TOC and PAGE fields on open
  settings = d.settings.element
  uf = OxmlElement("w:updateFields")
  uf.set(qn("w:val"), "true")
  after = ("hdrShapeDefaults", "footnotePr", "endnotePr", "compat", "docVars", "rsids",
           "mathPr", "attachedSchema", "themeFontLang", "clrSchemeMapping",
           "doNotIncludeSubdocsInStats", "doNotAutoCompressPictures", "forceUpgrade",
           "captions", "readModeInkLockDown", "smartTagType", "schemaLibrary",
           "shapeDefaults", "doNotEmbedSmartTags", "decimalSymbol", "listSeparator")
  nxt = next((el for el in settings if el.tag.split("}")[1] in after), None)
  if nxt is not None:
    nxt.addprevious(uf)                             # schema order of w:settings
  else:
    settings.append(uf)
  return d


def set_cell(cell, text):
  p = cell.paragraphs[0]
  runs = p.runs
  if runs:
    runs[0].text = text
    for r in runs[1:]:
      r.text = ""
  else:
    add_run(p, text, size=18)
  for extra in cell.paragraphs[1:]:
    extra._p.getparent().remove(extra._p)


# --------------------------------------------------------------- content
def desc_blocks(text):
  """Re-flow the hard-wrapped scenario description into paragraphs/bullets."""
  blocks = []
  for raw in text.splitlines():
    ln = raw.strip()
    if not ln:
      continue
    if ln.startswith("•"):
      blocks.append(["bullet", ln.lstrip("• ").strip()])
    elif blocks and (raw[:1].isspace() and blocks[-1][0] == "bullet"
                     or blocks[-1][0] == "para" and not blocks[-1][1].endswith((".", ":"))
                     and not ln.startswith(("Diagnoses", "Phase", "Note", "Checked",
                                                     "Comprehensive"))):
      sep = " | " if "|" in ln and "|" in blocks[-1][1] else " "
      blocks[-1][1] += sep + ln
    else:
      blocks.append(["para", ln])
  return blocks


def first(steps, pred):
  for s in steps:
    if pred(s):
      return s
  return None


def build(args):
  log_sha = hashlib.sha256(open(args.log, "rb").read()).hexdigest()
  log_name = os.path.basename(args.log)
  scns, meta, nlines = parse_log(args.log)
  steps = [st for s in scns for st in s["steps"]]
  by_cmd = {}
  for st in steps:
    by_cmd.setdefault(st["cmd"], []).append(st)
  cmds = sorted(by_cmd)

  # independent checks
  cs_bad_rx = [st["id"] for st in steps if st["rxf"] and st["rxf"]["cs"] != st["rxf"]["cs_calc"]]
  cs_bad_tx = [st["id"] for st in steps if st["txf"]["cs"] != st["txf"]["cs_calc"]]
  cmd_echo_bad = [st["id"] for st in steps if st["rxf"] and st["rxf"]["cmd"] != st["cmd"]]
  addr_echo_bad = [st["id"] for st in steps if st["rxf"] and st["rxf"]["addr"] != st["txf"]["addr"]]
  n_rx = sum(1 for st in steps if st["rx"])
  n_pass = sum(1 for st in steps if st["result"].startswith("PASS"))
  n_fail = len(steps) - n_pass

  d = prepare_template(args.template, args.fw, log_name, log_sha)
  B = Builder(d)
  B.toc()
  B.page_break()

  # ---------------------------------------------------------------- 1
  B.heading("1  Purpose and scope", 1)
  B.para(f"This document is {DOC['number']} \"{DOC['title']}\", Rev {DOC['rev']}, dated "
         f"{DOC['date']}. It records the HART communication unit functional test of the 481 "
         f"Back-Up Overspeed Module (BUOS) with firmware V{args.fw}: for every HART command "
         "that was sent to the device, the request frame, the response frame and their "
         "byte-by-byte meaning.")
  B.para(f"The test was run with the {TOOL} (Ectron in-house HART master) in \"Run All\" mode: "
         f"{len(scns)} automatic test scenarios, {len(steps)} request/response transactions, "
         f"{len(cmds)} different command numbers. All frames in this document are copied from "
         f"the raw test log ({log_name}); the log itself is reproduced in Appendix A.")
  B.para("The device identity read during the test is in Section 4, the results summary in "
         "Section 5, the command-by-command record in Section 6 and the scenario-by-scenario "
         "record in Section 7. Section 8 lists what this test does not cover.")
  B.heading("1.1  Reference documents", 2)
  B.caption(1, "Reference documents")
  B.table(["ID", "Document", "Revision / date"], [
    ("R1", "HCF_SPEC-13 HART Communication Protocol Specification", "HART 7"),
    ("R2", "HCF_SPEC-81 Token-Passing Data Link Layer Specification (frame format, checksum)", "HART 7"),
    ("R3", "HCF_SPEC-127 Universal Command Specification", "HART 7"),
    ("R4", "HCF_SPEC-151 Common Practice Command Specification", "HART 7"),
    ("R5", "HCF_SPEC-183 Common Tables Specification (units, classification codes)", "HART 7"),
    ("R6", "BUOS-TP-001 Verification & Validation Test Plan", "Draft B, 2026-11-12"),
    ("R7", f"Test log {log_name}", f"Recorded 2026-11-16 05:11; SHA-256 {log_sha}"),
    ("R8", f"{TOOL}, scenario definitions scenarios.py (repository hart_master_simulator)",
     "v1.1.10"),
  ], [720, 6624, 2736])
  B.heading("1.2  Terms", 2)
  for t in [
    "DUT: device under test – the BUOS with firmware V%s." % args.fw,
    "Master / field device: the HART master (test tool) sends requests (STX frames); the field "
    "device (DUT) answers with ACK frames.",
    "Primary / secondary master: identified by bit 7 of the first address byte (1 = primary).",
    "Short address: 1-byte polling address (0–63). Long address: 5-byte unique address "
    "(expanded device type + device ID); all zero = broadcast address.",
    "RC: response code, first status byte of a response. Status: field device status, second "
    "status byte.",
    "PV, SV, TV, QV: primary, secondary, tertiary, quaternary dynamic variable.",
    "Transaction ID Sxx-Tnnn: scenario xx, step nnn of the test log. Every frame in this "
    "document carries this ID; it is the same in Sections 6, 7 and Appendix A.",
    "Universal / Common Practice: HART command classes defined in R3 / R4.",
  ]:
    B.bullet(t)

  # ---------------------------------------------------------------- 2
  B.heading("2  Test setup", 1)
  B.caption(2, "Test setup")
  B.table(None, [
    ("Device under test", "481 Back-Up Overspeed Module (BUOS), Ectron Model M1334 (481C), "
                          "P/N 481-500-02, digital board 481-600d Rev D3"),
    ("Firmware", f"V{args.fw} (reported by Command 0 as software revision "
                 f"{split_frame(by_cmd[0][0]['rx'])['data'][2 + 6]})"),
    ("Test tool", f"{TOOL}, HART primary/secondary master on a PC"),
    ("Physical layer", f"HART FSK modem on serial port {meta['port'] or 'COM3'}, "
                       "1200 bit/s, 8 data bits, odd parity, 1 stop bit"),
    ("Run mode", "Run All – automatic scenarios in sequence, no operator action"),
    ("Scenarios run", f"{len(scns)} of {len(scns) + 3} (see Section 8 for the 3 manual ones)"),
    ("Test date", "2026-11-16, start 05:11 (from the log file name)"),
    ("Raw evidence", f"{log_name}\n{nlines} lines\nSHA-256 {log_sha}"),
  ], [2880, 7200], label_col=True)
  B.para("The SHA-256 hash identifies the log file this report was generated from. Any change "
         "to the log changes the hash.")

  # ---------------------------------------------------------------- 3
  B.heading("3  How to read the frame records", 1)
  B.para("Each HART frame is shown twice: first as the raw bytes exactly as logged "
         "(hexadecimal, in transmission order), then as a field table. In the field table, "
         "\"Byte\" is the position in the frame counted from 0 (the first preamble byte), "
         "\"Raw\" are the bytes of the field and \"Value / meaning\" is the decoded content.")
  B.caption(3, "HART frame structure (R2)")
  B.table(["Field", "Length", "Content"], [
    ("Preamble", "2–20 bytes", "0xFF bytes for receiver synchronisation"),
    ("Delimiter", "1 byte", "Frame type and address format: 0x02 / 0x82 request (STX), "
                            "0x06 / 0x86 response (ACK); bit 7 = long address"),
    ("Address", "1 or 5 bytes", "Bit 7 of the first byte = master (1 primary, 0 secondary), "
                                "bit 6 = burst mode. Short: polling address in bits 5–0. "
                                "Long: expanded device type (14 bits) + 3-byte device ID"),
    ("Command", "1 byte", "Command number"),
    ("Byte count", "1 byte", "Number of bytes between the byte count and the checksum"),
    ("Response code", "1 byte", "Responses only. 0 = success; other values see R3/R4"),
    ("Field device status", "1 byte", "Responses only. Bit 7 malfunction, 6 configuration "
                                       "changed, 5 cold start, 4 more status available, "
                                       "3 loop current fixed, 2 loop current saturated, "
                                       "1 non-PV out of limits, 0 PV out of limits"),
    ("Data", "0–n bytes", "Command-specific request or response data"),
    ("Checksum", "1 byte", "XOR of all bytes from the delimiter through the last data byte"),
  ], [1800, 1440, 6840])
  B.para("Floating-point values are IEEE 754 single precision, most significant byte first. "
         "Text fields are HART packed ASCII (6 bits per character) unless stated otherwise. "
         "Units are HART unit codes (R5). The \"Variant\" column of the transaction tables "
         "gives the address mode (short n / long / broadcast), the master (P primary, "
         "S secondary) and the number of request preambles.")
  B.para("Responses carry one preamble byte fewer than the number set in the DUT "
         "(Command 0 byte 12 / Command 59): 4 bytes received for the setting 5, 7 for the "
         "setting 8. This is consistent with the receiving HART modem losing the first character "
         "while it detects the carrier (typical FSK modem behaviour); a receiver needs only 2 "
         "preambles to synchronise (R2), so the frames are valid.")
  B.para("Every checksum in the log was re-computed by the report generator, independently "
         f"of the test tool: {n_rx} of {n_rx} response checksums are valid"
         + ("" if not cs_bad_rx else f" EXCEPT {', '.join(cs_bad_rx)}")
         + f"; {len(steps) - len(cs_bad_tx)} request checksums are valid and "
         f"{len(cs_bad_tx)} requests ({', '.join(cs_bad_tx) or 'none'}) carry a deliberately "
         "wrong checksum (Scenario 7). Every response echoes the command number "
         + ("and the address of its request." if not cmd_echo_bad and not addr_echo_bad
            else f"– exceptions: {', '.join(cmd_echo_bad + addr_echo_bad)}."))

  # ---------------------------------------------------------------- 4
  B.heading("4  Device identification", 1)
  B.para("Values reported by the DUT before any configuration write in the test run "
         "(Scenario 1 and the setup steps of Scenario 17). Writes made during the test were "
         "restored at the end of each scenario.")
  ident = []
  def resp_of(st):
    return st["rxf"]["data"][2:]
  c0 = first(by_cmd[0], lambda s: s["rx"] and s["rc"] == 0)
  for (_b, name, _raw, val) in layout_rows(CMD0, resp_of(c0), 0):
    if name.startswith("Expansion"):
      continue
    ident.append((name, val, f"Cmd 0, {c0['id']}"))
  def add_ident(cmd, label=None, pick=None, sid=None):
    cand = by_cmd.get(cmd, [])
    st = first(cand, lambda s: s["rx"] and s["rc"] == 0 and (sid is None or s["scn"] == sid))
    if not st:
      return
    for (_b, name, _raw, val) in layout_rows(RESP[cmd], resp_of(st), 0):
      if pick and name not in pick:
        continue
      ident.append((label or name, val, f"Cmd {cmd}, {st['id']}"))
  add_ident(7, pick={"Polling address", "Loop current mode"})
  add_ident(12, sid=17)
  add_ident(13, sid=17)
  add_ident(20, sid=17)
  add_ident(16, sid=17)
  add_ident(14)
  add_ident(15, pick={"PV range values units", "PV upper range value", "PV lower range value",
                      "PV damping value (s)", "Write protect code", "PV transfer function"})
  add_ident(50)
  B.caption(4, "DUT identification as read over HART")
  B.table(["Parameter", "Value", "Source"], ident, [3240, 4680, 2160], label_col=True)
  B.para(f"The software revision reported by Command 0 (27) matches the firmware under test, "
         f"V{args.fw}.")

  # ---------------------------------------------------------------- 5
  B.heading("5  Results summary", 1)
  B.para(f"Result of the run: {n_pass} of {len(steps)} transactions passed, {n_fail} failed. "
         f"The DUT answered {n_rx} requests. The other {len(steps) - n_rx} requests were "
         "negative tests where silence is the required behaviour (no device at that polling "
         "address, broadcast address with a command that must not be answered, wrong tag, "
         "bad checksum).")
  B.caption(5, "Results per scenario")
  rows = []
  for s in scns:
    np_ = sum(1 for st in s["steps"] if st["result"].startswith("PASS"))
    rows.append((str(s["no"]), s["title"], str(len(s["steps"])), str(np_),
                 str(len(s["steps"]) - np_), "PASS" if np_ == len(s["steps"]) else "FAIL"))
  rows.append(("", "Total", str(len(steps)), str(n_pass), str(n_fail),
               "PASS" if not n_fail else "FAIL"))
  B.table(["No.", "Scenario", "Steps", "Passed", "Failed", "Result"], rows,
          [720, 5400, 900, 1020, 1020, 1020])
  B.source("Scenarios 15, 18 and 19 need operator action (power cycle) and are not part of "
           "this run; see Section 8.")

  B.caption(5, "Command coverage")
  rows = []
  for c in cmds:
    sts = by_cmd[c]
    rcs = sorted({st["rc"] for st in sts if st["rc"] is not None})
    silent = sum(1 for st in sts if not st["rx"])
    impl = ("No (RC 64)" if rcs == [64] else "Yes") if rcs else "–"
    rc_s = ", ".join(str(r) for r in rcs) + (f"; no reply ×{silent}" if silent else "")
    rows.append((str(c), CMD_NAMES.get(c, "?"), cmd_class(c), str(len(sts)), rc_s, impl,
                 f"6.{cmds.index(c) + 1}"))
  B.table(["Cmd", "Name", "Class", "Count", "Response codes seen", "Implemented",
           "Section"], rows, [600, 3480, 1500, 760, 1700, 1180, 860], size=15)
  B.source("Implemented = the DUT executes the command; \"No (RC 64)\" = the DUT answers "
           "\"Command not implemented\" as required for commands the BUOS does not support.")

  # ---------------------------------------------------------------- 6
  B.page_break()
  B.heading("6  Command record", 1)
  B.para("One subsection per command number, in numerical order. Each subsection shows one "
         "representative transaction fully decoded, one decoded example for each other "
         "response code that occurred, and a list of all transactions with this command.")
  FIELD_W = [900, 2700, 2340, 4140]
  for k, c in enumerate(cmds, 1):
    sts = by_cmd[c]
    B.heading(f"6.{k}  Command {c} – {CMD_NAMES.get(c, '?')}", 2)
    B.para(f"Class: {cmd_class(c)}. Transactions in this run: {len(sts)}.")
    shown = []
    rep = (first(sts, lambda s: s["rx"] and s["rc"] == 0 and s["txf"]["data"])
           or first(sts, lambda s: s["rx"] and s["rc"] == 0)
           or first(sts, lambda s: s["rx"]) or sts[0])
    shown.append(rep)
    for rc in sorted({s["rc"] for s in sts if s["rc"] is not None} - {rep["rc"]}):
      shown.append(first(sts, lambda s, rc=rc: s["rc"] == rc))
    if c == 0:   # address variants of the identity command
      for pred in (lambda s: s["txf"]["long"] and s["rx"],
                   lambda s: not (s["txf"]["addr"][0] & 0x80) and s["rx"]):
        x = first(sts, pred)
        if x and x not in shown:
          shown.append(x)
    for st in sts:     # one example of each kind of expected silence
      if not st["rx"] and not any(not s["rx"] and variant(s).split(",")[0] ==
                                  variant(st).split(",")[0] for s in shown):
        shown.append(st)
    for ex_n, st in enumerate(shown):
      what = "Representative transaction" if ex_n == 0 else "Further example"
      res = short_result(st)
      B.para(f"{what} {st['id']} – \"{st['label']}\" – {res}", bold=True, after=40, keep=True)
      B.hexbox(f"Request (master → DUT), {len(st['tx'])} bytes:", st["tx"])
      B.caption(6, f"Command {c} request, {st['id']}")
      B.table(["Byte", "Field", "Raw", "Value / meaning"], frame_rows(st["tx"], True),
              FIELD_W, mono_cols=(2,), size=15)
      if st["rx"]:
        B.hexbox(f"Response (DUT → master), {len(st['rx'])} bytes:", st["rx"])
        B.caption(6, f"Command {c} response, {st['id']}")
        B.table(["Byte", "Field", "Raw", "Value / meaning"], frame_rows(st["rx"], False),
                FIELD_W, mono_cols=(2,), size=15)
      else:
        B.para("Response: none within the response time-out – this is the required "
               "behaviour for this request.", after=60)
      if st["info"]:
        B.source("Test tool note: " + st["info"])
      B.source(f"Source: {log_name}, lines {st['line']}–{st.get('end_line', st['line'])}")
    B.caption(6, f"All Command {c} transactions")
    rows = [(st["id"], st["label"], variant(st),
             "–" if st["rc"] is None else str(st["rc"]),
             "–" if st["status"] is None else f"0x{st['status']:02X}", short_result(st))
            for st in sts]
    B.table(["ID", "Step", "Variant", "RC", "Status", "Result"], rows,
            [1080, 3600, 1980, 500, 720, 2200], size=15)

  # ---------------------------------------------------------------- 7
  B.page_break()
  B.heading("7  Scenario record", 1)
  B.para("The scenarios in the order they were run. The purpose text is the scenario "
         "description from the test tool. The frames of every step are in Appendix A; "
         "decoded examples are in Section 6.")
  descs = {int(k.split(" ")[0]): v.get("description", "") for k, v in SCN.SCENARIOS.items()}
  for s in scns:
    B.heading(f"7.{s['seq']}  Scenario {s['no']} – {s['title']}", 2)
    for kind, text in desc_blocks(descs.get(s["no"], "")):
      if kind == "bullet":
        B.bullet(text)
      else:
        B.para(text, after=40)
    B.caption(7, f"Scenario {s['no']} steps")
    rows = [(st["id"], st["label"], str(st["cmd"]), variant(st),
             "–" if st["rc"] is None else str(st["rc"]),
             "–" if st["status"] is None else f"0x{st['status']:02X}", short_result(st))
            for st in s["steps"]]
    B.table(["ID", "Step", "Cmd", "Variant", "RC", "Status", "Result"], rows,
            [1080, 3240, 540, 1980, 500, 720, 2020], size=15)
    B.source(f"Test tool verdict: {s['done']}. Source: {log_name}, from line {s['line']}")

  # ---------------------------------------------------------------- 8
  B.heading("8  Limitations and open items", 1)
  B.para("The following points are not covered by this test run. They are listed so that the "
         "coverage of this report is not over-stated.")
  ual010 = [st for st in steps if "UAL010" in st["label"]]
  cold = ual010 and ual010[0]["status"] is not None and ual010[0]["status"] & 0x20
  oi = [
    ("HTR-OI-01", "Write protection (RC 7 \"In write-protect mode\")",
     "The BUOS does not implement write protection: Command 15 reports write protect code "
     "251 (\"None\"). RC 7 cannot be produced, so the write-protect behaviour of the write "
     "commands is not tested. All write commands were accepted with RC 0.",
     "Open – not applicable unless write protection is added"),
    ("HTR-OI-02", "Manual scenarios not run",
     "Scenarios that need an operator to power-cycle the DUT were skipped in Run All mode: "
     + meta["skipped"] + ". Not covered: Cold Start bit per master after power-up; "
     "non-volatile storage of message, tag, descriptor, date, final assembly number, long "
     "tag, configuration change counter, Configuration Changed bits and polling address "
     "across a power cycle.", "Open – run scenarios 15, 18, 19 and issue as an addendum"),
  ]
  if not cold:
    oi.append(("HTR-OI-03", "Cold Start bit (UAL010)",
               f"Steps {', '.join(s['id'] for s in ual010)} passed in the test tool, but the "
               "DUT was not power-cycled before the run, so the Cold Start bit (0x20) was "
               f"never set (status 0x{ual010[0]['status']:02X}). The cold start behaviour is "
               "therefore not demonstrated by this log. Covered by HTR-OI-02 (scenario 15).",
               "Open – together with HTR-OI-02"))
  oi.append(("HTR-OI-%02d" % (len(oi) + 1), "Command 54 classification",
             "Command 54 reports classification 0 (\"Not classified\") for all device "
             "variables. This is the device's design value, recorded for information.",
             "Information"))
  B.caption(8, f"{DOC['number']} open items")
  B.table(["ID", "Item", "Description", "Status"], oi, [1180, 1980, 5120, 1800], size=16)

  # ---------------------------------------------------------------- App A
  B.page_break()
  B.heading("Appendix A  Test log transcript", 1)
  B.para(f"All {len(steps)} transactions of {log_name}, unchanged, in the order they were "
         "run. TX = request from the test tool, RX = response from the DUT.")
  for s in scns:
    B.heading(f"A.{s['seq']}  Scenario {s['no']} – {s['title']}", 2)
    rows = []
    for st in s["steps"]:
      rows.append((st["id"], f"{st['label']}\n{st['result']}", "TX", sp(st["tx"])))
      rows.append(("", "", "RX", sp(st["rx"]) if st["rx"] else "(no response)"))
    B.table(["ID", "Step / tool verdict", "Dir", "Frame (hex)"], rows,
            [1000, 3000, 520, 5560], mono_cols=(3,), size=14)

  # drop the template's figures: they are no longer referenced by the body
  xml = d.element.xml
  for rid, rel in list(d.part.rels.items()):
    if rel.reltype.endswith("/image") and f'"{rid}"' not in xml:
      d.part.drop_rel(rid)
  d.save(args.out)
  print(f"wrote {args.out}: {len(scns)} scenarios, {len(steps)} transactions, "
        f"{len(cmds)} commands; rx checksum errors {cs_bad_rx}; bad tx {cs_bad_tx}; "
        f"cmd echo errors {cmd_echo_bad}; addr echo errors {addr_echo_bad}")


def main():
  ap = argparse.ArgumentParser(description=__doc__)
  ap.add_argument("--log", required=True)
  ap.add_argument("--template", default=os.path.join(
      HERE, "BUOS-HWAS-001_HWArchitecture_RevA_2026-11-12.docx"))
  ap.add_argument("--fw", default="2.27")
  ap.add_argument("--out", default=None)
  a = ap.parse_args()
  if not a.out:
    a.out = os.path.join(HERE, f"{DOC['number']}_HARTFunctionalTest_Rev{DOC['rev']}_"
                               f"{DOC['date']}.docx")
  build(a)


if __name__ == "__main__":
  main()
