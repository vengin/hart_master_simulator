import sys
import time
import struct
import threading
import queue
import serial
import serial.tools.list_ports
from datetime import datetime
from PyQt5.QtWidgets import (
  QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
  QGridLayout, QLabel, QPushButton, QComboBox, QSpinBox, QDoubleSpinBox,
  QTextEdit, QGroupBox, QTabWidget, QLineEdit, QCheckBox, QSplitter,
  QFrame, QScrollArea, QSizePolicy, QStatusBar, QAction, QFileDialog,
  QMessageBox, QTableWidget, QTableWidgetItem, QHeaderView, QProgressBar
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer, QMutex
from PyQt5.QtGui import QFont, QColor, QPalette, QTextCursor, QFontDatabase

from scenarios import SCENARIOS
from hart_codec import encode_packed_ascii, decode_packed_ascii
from hart_cp import CP_COMMANDS, CP_DECODERS, encode_cp_request

# --- HART PROTOCOL CONSTANTS ------------------------------------------------

HART_BAUD = 1200
HART_PARITY = serial.PARITY_ODD
HART_STOPBITS = serial.STOPBITS_ONE
HART_BYTESIZE = serial.EIGHTBITS

# Delimiter byte: bit 7 = long (unique-ID) address, bits 6-5 = number of
# expansion bytes, bits 2-0 = frame type. Primary vs secondary master is NOT
# in the delimiter: it is bit 7 of the (first) address byte.
FRAME_BACK = 0x01           # burst frame, slave -> masters
FRAME_STX  = 0x02           # master -> slave request (primary AND secondary)
FRAME_ACK  = 0x06           # slave -> master response
DELIM_LONG_BIT = 0x80
DELIM_SHORT_STX = FRAME_STX                    # 0x02
DELIM_LONG_STX  = DELIM_LONG_BIT | FRAME_STX   # 0x82
DELIM_SHORT_ACK = FRAME_ACK                    # 0x06
DELIM_LONG_ACK  = DELIM_LONG_BIT | FRAME_ACK   # 0x86

# Address byte 0 flags
ADDR_PRIMARY_MASTER_BIT = 0x80  # 1 = primary master, 0 = secondary master
ADDR_BURST_BIT          = 0x40  # set by a slave in burst mode (responses only)
ADDR_MASK               = 0x3F  # short polling address 0-63 / 6 LSBs of long addr

# Long frame Broadcast Address (38 zero bits): only C11/C21 may use it
BROADCAST_UNIQUE_ID = bytes(5)

# Wire time per character: 1 start + 8 data + 1 parity + 1 stop (8O1)
HART_BITS_PER_CHAR = 11

UNIT_CODES = {
  # Pressure
  1: "inH2O @ 68°F", 2: "inHg @ 0°C", 3: "ftH2O @ 68°F", 4: "mmH2O @ 68°F",
  5: "mmHg @ 0°C", 6: "psi", 7: "bar", 8: "mbar", 9: "g/cm²", 10: "kg/cm²",
  11: "Pa", 12: "kPa", 13: "torr", 14: "atm",
  # Volumetric flow / velocity
  15: "ft³/min", 16: "gal/min", 17: "l/min", 18: "impgal/min", 19: "m³/h",
  20: "ft/s", 21: "m/s", 22: "gal/s", 23: "Mgal/day", 24: "l/s", 25: "Ml/day",
  26: "ft³/s", 27: "ft³/day", 28: "m³/s", 29: "m³/day", 30: "impgal/h",
  31: "impgal/day",
  # Temperature, electrical
  32: "°C", 33: "°F", 34: "°R", 35: "K", 36: "mV", 37: "Ω", 38: "Hz", 39: "mA",
  # Volume, length, time
  40: "gal", 41: "l", 42: "impgal", 43: "m³", 44: "ft", 45: "m", 46: "bbl",
  47: "in", 48: "cm", 49: "mm", 50: "min", 51: "s", 52: "h", 53: "d",
  # Misc
  54: "cSt", 55: "cP", 56: "µS", 57: "%", 58: "V", 59: "pH",
  # Mass
  60: "g", 61: "kg", 62: "t", 63: "lb", 64: "short ton", 65: "long ton",
  237: "MPa",
  250: "Not used", 251: "None", 252: "Unknown", 253: "Special",
  # Codes 170-219 are unit-code expansion (meaning depends on the device
  # variable classification, Common Table 21) and are not decoded here.
}

# Response code byte with bit 7 set = communication error summary
COMM_STATUS_BITS = {
  0x40: "Vertical parity error",
  0x20: "Overrun error",
  0x10: "Framing error",
  0x08: "Longitudinal parity error",
  0x04: "Reserved",
  0x02: "Buffer overflow",
  0x01: "Reserved",
}

DEVICE_STATUS_BITS = {
  0x80: "Device malfunction",
  0x40: "Configuration changed",
  0x20: "Cold start",
  0x10: "More status available",
  0x08: "Analog output fixed",
  0x04: "Analog output saturated",
  0x02: "Non-primary variable OOR",
  0x01: "Primary variable OOR",
}

RESPONSE_CODES = {
  0:  "No command-specific error",
  1:  "Undefined",
  2:  "Invalid selection",
  3:  "Passed parameter too large",
  4:  "Passed parameter too small",
  5:  "Too few data bytes received",
  6:  "Device-specific command error",
  7:  "In write-protect mode",
  8:  "Update failure",
  9:  "Lower range too high",
  10: "Lower range too low",
  11: "Upper range too high",
  12: "Upper range too low",
  13: "Upper and lower range out of limits",
  14: "Span too small",
  15: "Not in correct mode",
  16: "Access restricted",
  17: "Invalid device variable index",
  18: "Invalid units code",
  19: "Device variable index not allowed",
  20: "Invalid extended device status",
  32: "Device busy",
  38: "Tag Mismatch Error",
  64: "Command not implemented",
}

# HART 7: 16-bit Manufacturer ID (C0 bytes 17-18)
KNOWN_MANUFACTURERS_16 = {
  0x61F6: "Ectron",
}

# HART 5: 8-bit Manufacturer ID (legacy C0 byte 1)
KNOWN_MANUFACTURERS = {
  0x00: "Unknown",
  0x01: "Emerson/Fisher/Rosemount",
  0x02: "Honeywell",
  0x03: "Yokogawa",
  0x04: "ABB",
  0x06: "Foxboro",
  0x13: "Endress+Hauser",
  0x1A: "Siemens",
  0x26: "Krohne",
  0x31: "Vega",
  0x44: "Pepperl+Fuchs",
  0x5A: "Rockwell/Allen-Bradley",
  0xE8: "Ectron",
}


# --- HART FRAME ENGINE ------------------------------------------------------

def calc_checksum(data: bytes) -> int:
  cs = 0
  for b in data:
    cs ^= b
  return cs


def _master_bit(master_type: str) -> int:
  return ADDR_PRIMARY_MASTER_BIT if master_type == "primary" else 0x00


def build_short_frame(address: int, command: int, data: bytes = b"",
                      preamble_count: int = 5,
                      master_type: str = "primary") -> bytes:
  """STX request with a short (polling) address. The master type is bit 7 of
  the address byte (1 = primary, 0 = secondary); the delimiter is 0x02 for both."""
  addr_byte = _master_bit(master_type) | (address & ADDR_MASK)
  frame_body = bytes([DELIM_SHORT_STX, addr_byte, command, len(data)]) + data
  cs = calc_checksum(frame_body)
  return bytes([0xFF] * preamble_count) + frame_body + bytes([cs])


def build_long_frame(unique_id: bytes, command: int, data: bytes = b"",
                     preamble_count: int = 5,
                     master_type: str = "primary") -> bytes:
  """STX request with a long (unique-ID) address.
  unique_id: 5 bytes = HART 7 Expanded Device Type (6 LSBs of MSB, LSB) +
  Device ID[3]; all zeros = Broadcast Address. Byte 0 bit 7 = primary master,
  bit 6 (burst) is always 0 in a master request."""
  addr_bytes = bytes([_master_bit(master_type) | (unique_id[0] & ADDR_MASK)]) + bytes(unique_id[1:5])
  frame_body = bytes([DELIM_LONG_STX]) + addr_bytes + bytes([command, len(data)]) + data
  cs = calc_checksum(frame_body)
  return bytes([0xFF] * preamble_count) + frame_body + bytes([cs])


def _find_frame_start(raw: bytes) -> tuple[int, int]:
  """Return (delimiter index, preamble count) of the first slave frame (ACK or
  BACK) that follows at least 2 preambles. A half-duplex modem may echo our own
  STX request first; that echo is skipped. Falls back to the first non-preamble
  byte (old behaviour) if no ACK/BACK frame is found."""
  first = None
  i = 0
  while i < len(raw):
    if raw[i] != 0xFF:
      i += 1
      continue
    j = i
    while j < len(raw) and raw[j] == 0xFF:
      j += 1
    if j >= len(raw):
      break
    if first is None:
      first = (j, j - i)
    if (j - i) >= 2 and (raw[j] & 0x07) in (FRAME_ACK, FRAME_BACK):
      return j, j - i
    i = j + 1
  if first is not None:
    return first
  # No 0xFF run at all (e.g. the modem dropped every preamble): fall back to
  # the first non-preamble byte, as the old parser did
  k = 0
  while k < len(raw) and raw[k] == 0xFF:
    k += 1
  return k, k


def parse_response(raw: bytes) -> dict | None:
  if not raw:
    return None

  i, preamble_count = _find_frame_start(raw)

  if i >= len(raw):
    return {"error": "Only preamble bytes received", "preamble_count": preamble_count}

  delimiter = raw[i]
  is_long = (delimiter & DELIM_LONG_BIT) != 0
  frame_type = delimiter & 0x07
  n_expansion = (delimiter >> 5) & 0x03

  try:
    addr_len = 5 if is_long else 1
    addr_start = i + 1
    cmd_pos = addr_start + addr_len + n_expansion
    if cmd_pos + 1 >= len(raw):
      return {"error": "Frame too short (%s addr)" % ("long" if is_long else "short"), "raw": raw}
    addr = raw[addr_start:addr_start + addr_len]
    cmd = raw[cmd_pos]
    byte_count = raw[cmd_pos + 1]
    frame_start = i
    data_start = cmd_pos + 2

    actual_payload = raw[data_start:data_start + byte_count]
    checksum_byte = raw[data_start + byte_count] if data_start + byte_count < len(raw) else None

    # Verify checksum (delimiter .. last data byte)
    frame_for_cs = raw[frame_start:data_start + byte_count]
    expected_cs = calc_checksum(frame_for_cs)
    cs_ok = (checksum_byte == expected_cs) if checksum_byte is not None else False
    complete = len(actual_payload) == byte_count

    # Status bytes
    st1 = actual_payload[0] if len(actual_payload) > 0 else 0
    st2 = actual_payload[1] if len(actual_payload) > 1 else 0
    payload = actual_payload[2:] if len(actual_payload) > 2 else b""

    # Response code byte: bit 7 set = communication error summary
    comm_error = (st1 & 0x80) != 0
    if comm_error:
      comm_errors = [desc for bit, desc in COMM_STATUS_BITS.items() if st1 & bit]
      status_text = "COMM ERROR: " + (", ".join(comm_errors) if comm_errors else "unknown")
    else:
      status_text = RESPONSE_CODES.get(st1, f"Response code {st1}")

    dev_flags = [desc for bit, desc in DEVICE_STATUS_BITS.items() if st2 & bit]

    return {
      "ok": True,
      "delimiter": delimiter,
      "frame_type": frame_type,
      "is_ack": frame_type == FRAME_ACK,
      "is_long": is_long,
      "is_burst": bool(addr and (addr[0] & ADDR_BURST_BIT)),
      "is_primary": bool(addr and (addr[0] & ADDR_PRIMARY_MASTER_BIT)),
      "address": addr,
      "cmd": cmd,
      "byte_count": byte_count,
      "complete": complete,
      "st1": st1,
      "st2": st2,
      "comm_error": comm_error,
      "status_text": status_text,
      "dev_flags": dev_flags,
      "payload": payload,
      "checksum_byte": checksum_byte,
      "expected_cs": expected_cs,
      "cs_ok": cs_ok,
      "preamble_count": preamble_count,
      "raw": raw,
    }
  except Exception as e:
    return {"error": str(e), "raw": raw}


def extract_reply(raw: bytes, request: bytes) -> bytes:
  """Return the slave's part of `raw`. A half-duplex modem may echo our own
  request on RX (possibly with a different number of preambles); a leading
  echo is removed. Returns b"" if nothing but the echo / preambles arrived."""
  if not raw:
    return b""
  body = request.lstrip(b"\xff")
  k = raw.find(body) if body else -1
  if k >= 0 and not raw[:k].strip(b"\xff"):
    raw = raw[k + len(body):]
  return raw if raw.strip(b"\xff") else b""


def hart_exchange(port, frame: bytes, timeout_s: float) -> tuple[bytes, float, bytes]:
  """Send `frame`, collect the answer. Returns (reply, latency_ms, raw):
  reply = slave bytes without any echo of the request (b"" = no response),
  latency from end of our transmission to the first reply byte."""
  port.reset_input_buffer()
  port.write(frame)
  port.flush()

  # Wait for last TX byte to leave the wire, then start latency clock.
  tx_duration_s = len(frame) * HART_BITS_PER_CHAR / HART_BAUD
  time.sleep(tx_duration_s)
  t_tx_end = time.perf_counter()

  raw = b""
  arrivals = []                      # (offset in raw, arrival time) per chunk
  deadline = t_tx_end + timeout_s
  while time.perf_counter() < deadline:
    waiting = port.in_waiting
    if waiting:
      chunk = port.read(waiting)
      arrivals.append((len(raw), time.perf_counter()))
      raw += chunk
      time.sleep(0.02)
    elif extract_reply(raw, frame):
      time.sleep(0.05)
      if not port.in_waiting:
        break
    else:
      time.sleep(0.02)

  reply = extract_reply(raw, frame)
  if reply:
    first = len(raw) - len(reply)
    t_first = [t for off, t in arrivals if off <= first][-1]
    latency_ms = (t_first - t_tx_end) * 1000.0
  else:
    latency_ms = (time.perf_counter() - t_tx_end) * 1000.0  # timeout, no response
  return reply, latency_ms, raw


# --- COMMAND DECODERS -------------------------------------------------------

# HART 7 Command 0 (22 data bytes; see hart_spec c0_read_unique_identifier)
def decode_cmd0(payload: bytes) -> dict:
  payload_len = len(payload)
  if payload_len < 11:
    return {"error": f"Invalid payload length: {payload_len} bytes. Too short."}

  # HART 6/7 expanded format: byte 0 = 254
  if payload[0] == 254:
    if payload_len < 12:
      return {"error": f"Expanded format error: Minimum 12 bytes required, got {payload_len}"}

    hart_rev = payload[4]
    exp_dev_type = (payload[1] << 8) | payload[2]
    dev_id = payload[9:12]
    p = lambda n: payload[n] if payload_len > n else None
    mfr = ((payload[17] << 8) | payload[18]) if payload_len > 18 else None
    dist = ((payload[19] << 8) | payload[20]) if payload_len > 20 else None
    result = {
      "Protocol Format": f"HART {hart_rev} (Expanded, {payload_len}B)",
      "Expanded Device Type": f"0x{exp_dev_type:04X}",
      "Min preambles (request)": p(3),
      "HART revision": hart_rev,
      "Device revision": payload[5],
      "Software revision": payload[6],
      "Hardware revision": payload[7] >> 3,
      "Physical signaling code": payload[7] & 0x07,
      "Flags": f"0x{payload[8]:02X}",
      "Device ID": f"{dev_id[0]:02X}:{dev_id[1]:02X}:{dev_id[2]:02X}",
      "Response preambles": p(12),
      "Last Device Variable code": p(13),
      "Config Change Count": ((payload[14] << 8) | payload[15]) if payload_len > 15 else None,
      "Extended Device Status": f"0x{payload[16]:02X}" if payload_len > 16 else None,
      "Manufacturer ID": (f"0x{mfr:04X} ({KNOWN_MANUFACTURERS_16.get(mfr, 'unknown')})"
                          if mfr is not None else None),
      "Private Label Distributor": f"0x{dist:04X}" if dist is not None else None,
      "Device Profile": p(21),
      # HART 7 long address: 6 LSBs of Expanded Device Type MSB, its LSB, Device ID
      "_unique_id": bytes([payload[1] & ADDR_MASK, payload[2]]) + bytes(dev_id),
    }
    if hart_rev >= 7 and payload_len < 22:
      result["Warning"] = f"HART 7 Command 0 has 22 data bytes, got {payload_len}"
    return result

  # HART 5 format (12 data bytes): byte 0 = Manufacturer ID, byte 1 = Device Type
  if payload_len < 12:
    if payload_len == 11 and payload[3] <= 6:
      mfr_id = payload[0]
      dev_type = payload[1]
      return {
        "Protocol Format": "Non-standard HART 5 (Truncated)",
        "Manufacturer ID": f"0x{mfr_id:02X}",
        "Device type code": f"0x{dev_type:02X}",
        "HART revision": payload[3],
        "_unique_id": bytes([mfr_id & ADDR_MASK, dev_type]) + payload[8:11]
      }
    return {"error": f"HART 5 format error: Expected 12 bytes, got {payload_len}"}

  hart_rev = payload[3]
  if hart_rev > 5:
    return {"error": f"HART 5 format error: byte 0 is not 254 but HART Revision is {hart_rev}"}

  mfr_id = payload[0]
  dev_type = payload[1]
  dev_id = payload[8:11]
  return {
    "Protocol Format": "HART 5 (Legacy)",
    "Manufacturer ID": f"0x{mfr_id:02X} ({KNOWN_MANUFACTURERS.get(mfr_id, 'unknown')})",
    "Device type code": f"0x{dev_type:02X}",
    "HART revision": hart_rev,
    "Device revision": payload[4],
    "Software revision": payload[5],
    "Hardware revision": payload[6] >> 3,
    "Flags": f"0x{payload[7]:02X}",
    "Device ID": f"{dev_id[0]:02X}:{dev_id[1]:02X}:{dev_id[2]:02X}",
    "_unique_id": bytes([mfr_id & ADDR_MASK, dev_type]) + bytes(dev_id),
  }
  result.update(extra)
  return result


def decode_cmd1(payload: bytes) -> dict:
  if len(payload) < 5:
    return {"error": f"Too short: {len(payload)} bytes"}
  unit = payload[0]
  value = struct.unpack(">f", payload[1:5])[0]
  return {
    "PV unit code": f"{unit} ({UNIT_CODES.get(unit, 'unknown')})",
    "PV value": f"{value:.4f}",
  }


def decode_cmd2(payload: bytes) -> dict:
  if len(payload) < 8:
    return {"error": f"Too short: {len(payload)} bytes"}
  current = struct.unpack(">f", payload[0:4])[0]
  pct = struct.unpack(">f", payload[4:8])[0]
  return {
    "Loop current": f"{current:.4f} mA",
    "PV % of range": f"{pct:.2f} %",
  }


def decode_cmd3(payload: bytes) -> dict:
  if len(payload) < 4:
    return {"error": f"Too short: {len(payload)} bytes"}
  result = {}
  current = struct.unpack(">f", payload[0:4])[0]
  result["Loop current"] = f"{current:.4f} mA"
  offset = 4
  var_names = ["PV", "SV", "TV", "QV"]
  for name in var_names:
    if offset + 5 <= len(payload):
      unit = payload[offset]
      val = struct.unpack(">f", payload[offset+1:offset+5])[0]
      result[f"{name} unit"] = f"{unit} ({UNIT_CODES.get(unit, 'unknown')})"
      result[f"{name} value"] = f"{val:.4f}"
      offset += 5
  return result


def decode_cmd6(payload: bytes) -> dict:
  """C6 response / C7: polling address + loop current mode"""
  if len(payload) < 2:
    return {"error": f"Too short: {len(payload)} bytes (need 2)"}
  return {
    "Polling address": payload[0],
    "Loop current mode": f"{payload[1]} ({'enabled' if payload[1] else 'disabled'})",
  }


def decode_cmd12(payload: bytes) -> dict:
  if len(payload) < 24:
    return {"error": f"Too short: {len(payload)} bytes"}
  msg = _decode_packed_ascii(payload[:24])
  return {"Message": msg}


def decode_cmd13(payload: bytes) -> dict:
  if len(payload) < 21:
    return {"error": f"Too short: {len(payload)} bytes"}
  tag = _decode_packed_ascii(payload[:6])
  descriptor = _decode_packed_ascii(payload[6:18])
  day = payload[18]
  month = payload[19]
  year = payload[20]
  return {
    "Tag": tag.strip(),
    "Descriptor": descriptor.strip(),
    "Date": f"{day:02d}/{month:02d}/{1900+year}",
  }


def decode_cmd14(payload: bytes) -> dict:
  """C14: transducer serial[3], limits/min span units, upper, lower, min span"""
  if len(payload) < 16:
    return {"error": f"Too short: {len(payload)} bytes (need 16)"}
  serial_no = (payload[0] << 16) | (payload[1] << 8) | payload[2]
  units = payload[3]
  upper = struct.unpack(">f", payload[4:8])[0]
  lower = struct.unpack(">f", payload[8:12])[0]
  min_span = struct.unpack(">f", payload[12:16])[0]
  return {
    "PV transducer S/N": f"{serial_no}",
    "Limits/min span units": f"{units} ({UNIT_CODES.get(units, 'unknown')})",
    "Upper transducer limit": f"{upper:.4f}",
    "Lower transducer limit": f"{lower:.4f}",
    "Minimum span": f"{min_span:.4f}",
  }


def decode_cmd16(payload: bytes) -> dict:
  if len(payload) < 3:
    return {"error": f"Too short: {len(payload)} bytes (need 3)"}
  return {"Final assembly number": (payload[0] << 16) | (payload[1] << 8) | payload[2]}


def decode_cmd20(payload: bytes) -> dict:
  if len(payload) != 32:
    return {"error": f"Invalid payload length for Cmd20: {len(payload)} bytes (need 32)"}

  try:
    long_tag = payload.decode('latin-1').strip()
  except Exception as e:
    return {"error": f"Decoding failed: {str(e)}"}

  return {
    "Long Tag": long_tag
  }


def decode_cmd38(payload: bytes) -> dict:
  if len(payload) < 2:
    return {"error": f"Too short: {len(payload)} bytes (need 2)"}
  return {"Config Change Count": (payload[0] << 8) | payload[1]}


def decode_cmd59(payload: bytes) -> dict:
  if len(payload) < 1:
    return {"error": "No payload"}
  return {"Response preambles": payload[0]}


def decode_cmd48(payload: bytes) -> dict:
  if not payload:
    return {"error": "No payload"}
  lines = {}
  for i, b in enumerate(payload):
    lines[f"Extended status byte {i}"] = f"0x{b:02X} ({b:08b}b)"
  return lines

def _decode_packed_ascii(data: bytes) -> str:
  """Decode HART packed-ASCII (6-bit encoding, 3 bytes -> 4 chars)."""
  return decode_packed_ascii(data)


def _encode_packed_ascii(tag: str) -> bytes:
  """Encode a tag into 6-byte HART Packed ASCII (8 chars, space padded)."""
  return encode_packed_ascii(tag, 6)


def decode_cmd11(payload: bytes) -> dict:
  """Command 11 response data is identical to Command 0 (HART 7 spec)."""
  return decode_cmd0(payload)


def decode_response(cmd: int, payload: bytes) -> dict | None:
  decoders = {
    0:  decode_cmd0,
    1:  decode_cmd1,
    2:  decode_cmd2,
    3:  decode_cmd3,
    6:  decode_cmd6,
    7:  decode_cmd6,
    11: decode_cmd11,
    12: decode_cmd12,
    13: decode_cmd13,
    14: decode_cmd14,
    16: decode_cmd16,
    17: decode_cmd12,   # response echoes the message
    18: decode_cmd13,   # response echoes tag/descriptor/date
    19: decode_cmd16,
    20: decode_cmd20,
    21: decode_cmd0,
    22: decode_cmd20,   # response echoes the long tag
    38: decode_cmd38,
    48: decode_cmd48,
    59: decode_cmd59,
  }
  fn = decoders.get(cmd)
  if fn:
    return fn(payload)
  cp = CP_DECODERS.get(cmd)          # Common Practice commands (hart_cp.py)
  return cp(payload, UNIT_CODES) if cp else None


# --- SERIAL WORKER THREAD ---------------------------------------------------

class HartWorker(QThread):
  frame_logged = pyqtSignal(str, str, bytes, float)  # dir, label, data, latency_ms
  response_ready = pyqtSignal(dict)
  error_occurred = pyqtSignal(str)

  def __init__(self):
    super().__init__()
    self._port = None
    self._queue = queue.Queue()
    self._running = False
    self._mutex = QMutex()

  def connect_port(self, port_name: str, preambles: int = 5,
                   master_type: str = "primary", timeout: float = 2.0) -> bool:
    try:
      self._port = serial.Serial(
        port=port_name,
        baudrate=HART_BAUD,
        parity=HART_PARITY,
        stopbits=HART_STOPBITS,
        bytesize=HART_BYTESIZE,
        timeout=timeout
      )
      self._preambles = preambles
      self._master_type = master_type
      self._timeout = timeout
      return True
    except Exception as e:
      self.error_occurred.emit(str(e))
      return False

  def disconnect_port(self):
    if self._port and self._port.is_open:
      self._port.close()
      self._port = None

  def queue_command(self, address, command, data=b"",
                    use_long=False, unique_id=None, label=""):
    self._queue.put((address, command, data, use_long, unique_id, label))

  def run(self):
    self._running = True
    while self._running:
      try:
        item = self._queue.get(timeout=0.1)
      except queue.Empty:
        continue

      address, command, data, use_long, unique_id, label = item

      if not self._port or not self._port.is_open:
        self.error_occurred.emit("Port not open")
        continue

      try:
        if use_long and unique_id:
          frame = build_long_frame(unique_id, command, data,
                                   self._preambles, self._master_type)
        else:
          frame = build_short_frame(address, command, data,
                                    self._preambles, self._master_type)

        self.frame_logged.emit("TX", label, frame, 0.0)
        raw, latency_ms, _wire = hart_exchange(self._port, frame, self._timeout)
        self.frame_logged.emit("RX", label, raw, latency_ms)

        parsed = parse_response(raw)
        if parsed:
          parsed["_command"] = command
          parsed["_label"] = label
          parsed["_latency_ms"] = latency_ms
          decoded = decode_response(command, parsed.get("payload", b""))
          if decoded:
            parsed["_decoded"] = decoded
        self.response_ready.emit(parsed or {"error": "No response", "_label": label})

      except Exception as e:
        self.error_occurred.emit(f"IO error: {e}")

  def stop(self):
    self._running = False
    self.wait()


# --- AUTO-POLL WORKER -------------------------------------------------------

class AutoPollWorker(QThread):
  poll_tick = pyqtSignal(int, int, bytes)  # address, command, data

  def __init__(self, commands, interval_s):
    super().__init__()
    self._commands = commands
    self._interval = interval_s
    self._running = False

  def run(self):
    self._running = True
    while self._running:
      for addr, cmd, data in self._commands:
        if not self._running:
          break
        self.poll_tick.emit(addr, cmd, data)
        time.sleep(self._interval)

  def stop(self):
    self._running = False
    self.wait()



# --- STYLE ------------------------------------------------------------------
STYLE = ""
# Color tags for log text
LOG_TX_COLOR = "#60a5fa"
LOG_RX_OK_COLOR = "#34d399"
LOG_RX_ERR_COLOR = "#f87171"
LOG_INFO_COLOR = "#fbbf24"
LOG_DECODED_COLOR = "#93c5fd"
LOG_MUTED = "#94a3b8"


# --- SCENARIO ENGINE --------------------------------------------------------
#
# Optional step keys (besides those listed in scenarios.py):
#   expect_rc            int | list   response code(s) required
#   expect_status_set    int          device status (st2) bits that must be set
#   expect_status_clear  int          device status bits that must be clear
#   expect_payload       ref          response data must equal ref
#   expect_payload_one_of [ref,...]   response data must equal one of the refs
#   payload_slice        [a, b]       compare only response data[a:b]
#   expect_u16           {offset, ref, ref_offset, delta}
#                                     Unsigned-16 at data[offset] must equal the
#                                     one in saved ref at ref_offset plus delta
#                                     (int or list of ints, modulo 2^16)
#   save_as              str          store the response data under this name
#   data_from            ref          request data taken from a saved response
#   data_u16             {ref, offset, delta}
#                                     request data = Unsigned-16 at saved ref
#                                     [offset] plus delta (mod 2^16)
#   broadcast            bool         long frame to the Broadcast Address
#   retries / retry_delay int / float resend while there is no response
#   manual               str          no frame: ask the operator, wait for OK
#
# A ref is bytes, a saved name, or (name, start, end) for a slice of it.
# Steps with any expect_*/save_as key are strict: silence is always a failure.

STEP_EXPECT_KEYS = ("expect_rc", "reject_rc", "check", "expect_status_set",
                    "expect_status_clear", "expect_payload", "expect_payload_one_of",
                    "expect_u16", "save_as")

# Separates an informational remark (warning that does not fail the step) from
# the result text in evaluate_step()'s reason.
INFO_SEP = " | info: "


def resolve_ref(ref, saved: dict) -> bytes:
  if isinstance(ref, (bytes, bytearray)):
    return bytes(ref)
  if isinstance(ref, str):
    if ref not in saved:
      raise KeyError(f"no saved response '{ref}' (earlier step failed?)")
    return saved[ref]
  if isinstance(ref, (tuple, list)) and len(ref) == 3:
    return resolve_ref(ref[0], saved)[ref[1]:ref[2]]
  raise ValueError(f"bad reference {ref!r}")


def _bits_text(mask: int) -> str:
  names = [desc for bit, desc in DEVICE_STATUS_BITS.items() if mask & bit]
  return ", ".join(names) if names else f"0x{mask:02X}"


def step_is_strict(step: dict) -> bool:
  return any(k in step for k in STEP_EXPECT_KEYS)


def evaluate_step(step: dict, raw: bytes, parsed: dict | None, poll_addr: int,
                  saved: dict, request_uid: bytes | None = None) -> tuple[bool, str]:
  """Decide whether one scenario step passed. Pure function (no I/O, no Qt).
  Returns (ok, reason)."""
  cmd = step.get("cmd")
  use_long = bool(step.get("use_long", False))
  master = step.get("master", "primary")

  if step.get("expect_no_response", False):
    if not raw:
      return True, "silent (expected)"
    return False, "got a response, expected silence"

  step_addr = step.get("_addr", poll_addr)
  if not use_long and step_addr != poll_addr and not step_is_strict(step):
    # Multi-drop sweep: only the configured address may answer
    if not raw:
      return True, "silent (no device at this address)"
    return False, f"device answered at polling address {step_addr} (configured: {poll_addr})"

  if not raw:
    return False, "no response"
  if not parsed or not parsed.get("ok"):
    return False, f"parse error: {(parsed or {}).get('error', 'unknown')}"
  if not parsed.get("is_ack"):
    return False, f"not an ACK frame (delimiter 0x{parsed['delimiter']:02X})"
  if not parsed.get("complete", True):
    return False, "truncated response"
  if not parsed.get("cs_ok"):
    return False, "checksum error"
  if parsed.get("cmd") != cmd:
    return False, f"response is for command {parsed.get('cmd')}, request was {cmd}"
  if parsed.get("is_long") != use_long:
    return False, "response frame format (short/long) differs from the request"
  if parsed.get("is_primary") != (master == "primary"):
    return False, (f"response addressed to the {'primary' if parsed.get('is_primary') else 'secondary'}"
                   f" master, request came from the {master} master")
  addr = parsed.get("address", b"")
  if not use_long:
    if addr and (addr[0] & ADDR_MASK) != (step_addr & ADDR_MASK):
      return False, f"response polling address {addr[0] & ADDR_MASK}, expected {step_addr}"
  elif request_uid is not None and len(addr) == 5:
    if bytes([addr[0] & ADDR_MASK]) + bytes(addr[1:]) != bytes([request_uid[0] & ADDR_MASK]) + bytes(request_uid[1:5]):
      return False, "response long address differs from the request"
  if parsed.get("comm_error"):
    return False, parsed.get("status_text", "communication error")

  rc = parsed["st1"]
  st2 = parsed["st2"]
  infos = []
  exp_rc = step.get("expect_rc")
  if callable(exp_rc):
    # decided at run time from saved responses: (list | None, info)
    exp_rc, info = exp_rc(saved)
    if info:
      infos.append(info)
  if exp_rc is None and step.get("check") is not None:
    exp_rc = 0                             # a content check needs a successful reply
  if exp_rc is not None:
    allowed = list(exp_rc) if isinstance(exp_rc, (list, tuple)) else [exp_rc]
    if rc not in allowed:
      return False, f"RC {rc} ({RESPONSE_CODES.get(rc, '?')}), expected {allowed}"
  rej_rc = step.get("reject_rc")
  if rej_rc is not None:
    rejected = list(rej_rc) if isinstance(rej_rc, (list, tuple)) else [rej_rc]
    if rc in rejected:
      return False, f"RC {rc} ({RESPONSE_CODES.get(rc, '?')}), must not be {rejected}"

  set_mask = step.get("expect_status_set", 0)
  if (st2 & set_mask) != set_mask:
    return False, f"device status 0x{st2:02X}: missing {_bits_text(set_mask & ~st2)}"
  clr_mask = step.get("expect_status_clear", 0)
  if st2 & clr_mask:
    return False, f"device status 0x{st2:02X}: unexpected {_bits_text(st2 & clr_mask)}"

  payload = parsed.get("payload", b"")
  sl = step.get("payload_slice")
  view = payload[sl[0]:sl[1]] if sl else payload
  try:
    if "expect_payload" in step:
      ref = resolve_ref(step["expect_payload"], saved)
      if view != ref:
        return False, f"data {view.hex(' ')} != expected {ref.hex(' ')}"
    if "expect_payload_one_of" in step:
      refs = [resolve_ref(r, saved) for r in step["expect_payload_one_of"]]
      if view not in refs:
        return False, f"data {view.hex(' ')} matches none of {len(refs)} expected values"
    if "expect_u16" in step:
      e = step["expect_u16"]
      off = e["offset"]
      ref = resolve_ref(e["ref"], saved)
      roff = e.get("ref_offset", off)
      if len(payload) < off + 2 or len(ref) < roff + 2:
        return False, "data too short for Unsigned-16 check"
      val = (payload[off] << 8) | payload[off + 1]
      base = (ref[roff] << 8) | ref[roff + 1]
      deltas = e.get("delta", 0)
      deltas = list(deltas) if isinstance(deltas, (list, tuple)) else [deltas]
      if ((val - base) & 0xFFFF) not in [d & 0xFFFF for d in deltas]:
        return False, f"value {val} (0x{val:04X}), expected {base} + {deltas}"
  except (KeyError, ValueError) as ex:
    return False, str(ex)

  check = step.get("check")
  if check is not None:
    # callable(parsed, saved) -> (ok, message); message on success = info
    ok, msg = check(parsed, saved)
    if not ok:
      return False, msg
    if msg:
      infos.append(msg)

  reason = f"OK (RC {rc}, status 0x{st2:02X})"
  if infos:
    reason += INFO_SEP + " ".join(infos)
  return True, reason


class ScenarioRunner(QThread):
  """Executes a scenario step list sequentially on the serial port."""
  step_started  = pyqtSignal(int, str, str)       # step_idx, label, note
  step_done     = pyqtSignal(int, str, bytes, bytes, float)  # step_idx, label, tx, rx, latency_ms
  step_result   = pyqtSignal(int, bool, str)       # step_idx, ok, reason
  step_error    = pyqtSignal(int, str, str)        # step_idx, label, error_msg
  uid_learned   = pyqtSignal(bytes)                # long address learned from Cmd0
  manual_prompt = pyqtSignal(int, str)             # step_idx, instruction for the operator
  scenario_done = pyqtSignal(int, int)             # steps_ok, steps_fail

  RESPONSE_TIMEOUT_S = 2.0

  def __init__(self, port: serial.Serial, steps: list, unique_id: bytes | None,
               poll_addr: int):
    super().__init__()
    self._port = port
    self._steps = steps
    self._unique_id = unique_id
    self._poll_addr = poll_addr
    self._abort = False
    self._saved = {}
    self._manual_event = threading.Event()
    self._manual_ok = False

  def abort(self):
    self._abort = True
    self._manual_event.set()

  def confirm_manual(self, ok: bool):
    """Called by the GUI when the operator answers a manual step."""
    self._manual_ok = ok
    self._manual_event.set()

  def _exchange(self, frame: bytes) -> tuple[bytes, float]:
    reply, latency_ms, _wire = hart_exchange(self._port, frame, self.RESPONSE_TIMEOUT_S)
    return reply, latency_ms

  def _run_manual(self, idx: int, step: dict) -> tuple[bool, str]:
    self._manual_ok = False
    self._manual_event.clear()
    self.manual_prompt.emit(idx, step["manual"])
    self._manual_event.wait()
    if self._abort or not self._manual_ok:
      self._abort = True
      return False, "cancelled by operator"
    time.sleep(step.get("delay_post", 0.0))
    return True, "confirmed by operator"

  def run(self):
    ok_count = 0
    fail_count = 0

    for idx, step in enumerate(self._steps):
      if self._abort:
        break

      label    = step.get("label", f"Step {idx}")
      note     = step.get("note", "")
      self.step_started.emit(idx, label, note)

      if "manual" in step:
        ok, reason = self._run_manual(idx, step)
        ok_count += ok
        fail_count += not ok
        self.step_result.emit(idx, ok, reason)
        continue

      cmd      = step["cmd"]
      use_long = step.get("use_long", False)
      preambles = step.get("preambles", 5)
      master   = step.get("master", "primary")
      delay_pre  = step.get("delay_pre", 0.3)
      delay_post = step.get("delay_post", 0.2)
      corrupt_cs        = step.get("_corrupt_cs", False)
      expect_no_resp    = step.get("expect_no_response", False)
      step_addr = step.get("_addr", self._poll_addr)

      time.sleep(delay_pre)

      try:
        data = step.get("data", b"")
        data_from = step.get("data_from", step.get("_data_from"))
        if data_from is not None:
          try:
            data = resolve_ref(data_from, self._saved)
          except KeyError:
            fail_count += 1
            self.step_result.emit(idx, False,
              f"no saved value '{data_from}' to send (the earlier read failed)")
            continue
        if "data_u16" in step:
          e = step["data_u16"]
          ref = resolve_ref(e["ref"], self._saved)
          off = e.get("offset", 0)
          val = (((ref[off] << 8) | ref[off + 1]) + e.get("delta", 0)) & 0xFFFF
          data = bytes([val >> 8, val & 0xFF])

        uid = None
        if use_long:
          uid = BROADCAST_UNIQUE_ID if step.get("broadcast") else self._unique_id
          if uid is None:
            fail_count += 1
            self.step_result.emit(idx, False, "no unique ID known - run a Cmd0 step first")
            continue
          frame = build_long_frame(uid, cmd, data, preambles, master)
        else:
          frame = build_short_frame(step_addr, cmd, data, preambles, master)

        if corrupt_cs:
          frame = frame[:-1] + bytes([frame[-1] ^ 0xFF])

        attempts = 1 + (0 if expect_no_resp else step.get("retries", 0))
        for attempt in range(attempts):
          raw, latency_ms = self._exchange(frame)
          if raw or self._abort:
            break
          if attempt + 1 < attempts:
            time.sleep(step.get("retry_delay", 0.5))
        self.step_done.emit(idx, label, frame, raw, latency_ms)

        parsed = parse_response(raw)
        ok, reason = evaluate_step(step, raw, parsed, self._poll_addr, self._saved,
                                   request_uid=uid)
        if ok and parsed and parsed.get("ok"):
          if "save_as" in step:
            self._saved[step["save_as"]] = parsed.get("payload", b"")
          if "_save" in step and parsed.get("st1") == 0:
            self._saved[step["_save"]] = parsed.get("payload", b"")
          # Learn the long address only from a correctly addressed Cmd0 reply
          if (cmd == 0 and not corrupt_cs and not step.get("broadcast")
              and (use_long or step_addr == self._poll_addr)):
            decoded = decode_response(0, parsed.get("payload", b""))
            if decoded and "_unique_id" in decoded:
              self._unique_id = decoded["_unique_id"]
              self.uid_learned.emit(self._unique_id)

        ok_count += ok
        fail_count += not ok
        self.step_result.emit(idx, ok, reason)

        time.sleep(delay_post)

      except Exception as e:
        self.step_error.emit(idx, label, str(e))
        fail_count += 1

    self.scenario_done.emit(ok_count, fail_count)

class ScenarioTableWidget(QTableWidget):
  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self._last_w = 0
    self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

  def showEvent(self, event):
    super().showEvent(event)
    self._last_w = self.viewport().width()

  def resizeEvent(self, event):
    super().resizeEvent(event)
    vw = self.viewport().width()
    if self._last_w > 0 and vw != self._last_w:
      delta = vw - self._last_w
      w1 = self.columnWidth(1) + delta
      self.setColumnWidth(1, max(w1, 100))
    self._last_w = vw


# --- MAIN WINDOW ------------------------------------------------------------

class HartMasterSim(QMainWindow):
  def __init__(self):
    super().__init__()
    self.setWindowTitle("HART Master Simulator  v1.1.7")
    self.setMinimumSize(1400, 750)
    self.resize(1482, 860)

    self._worker = HartWorker()
    self._worker.frame_logged.connect(self._on_frame_logged)
    self._worker.response_ready.connect(self._on_response)
    self._worker.error_occurred.connect(self._on_error)
    self._worker.start()

    self._connected = False
    self._unique_id = None  # 5-byte long address learned from Cmd0
    self._auto_poller = None
    self._scenario_runner = None
    self._run_all_queue = []
    self._run_all_total = 0
    self._run_all_ok = 0
    self._run_all_fail = 0
    self._log_lines = 0
    self._max_log_lines = 2000
    self._session_log = []

    self._build_ui()
#    self.setStyleSheet(STYLE)
    try:
      with open("style.qss", "r") as f:
        self.setStyleSheet(f.read())
    except Exception as e:
      print(f"Failed to load style.qss: {e}")
    self._refresh_ports()

    self._status_bar = QStatusBar()
    self.setStatusBar(self._status_bar)
    self._status_bar.showMessage("Disconnected  |  HART 1200 baud 8O1")

    self._port_timer = QTimer()
    self._port_timer.timeout.connect(self._refresh_ports)
    self._port_timer.start(3000)

  # -- UI construction ----------------------------------------------------

  def _build_ui(self):
    central = QWidget()
    self.setCentralWidget(central)
    root = QHBoxLayout(central)
    root.setContentsMargins(8, 8, 8, 8)
    root.setSpacing(8)

    splitter = QSplitter(Qt.Horizontal)
    root.addWidget(splitter)

    # Left panel
    left = QWidget()
    left.setMaximumWidth(340)
    left.setMinimumWidth(280)
    left_layout = QVBoxLayout(left)
    left_layout.setContentsMargins(0, 0, 0, 0)
    left_layout.setSpacing(6)

    left_layout.addWidget(self._build_connection_panel())
    left_layout.addWidget(self._build_address_panel())
    left_layout.addWidget(self._build_commands_panel())
    left_layout.addWidget(self._build_autopoll_panel())
    left_layout.addStretch()

    splitter.addWidget(left)

    # Right panel: tabs
    right = QWidget()
    right_layout = QVBoxLayout(right)
    right_layout.setContentsMargins(0, 0, 0, 0)

    self._tabs = QTabWidget()
    self._tabs.addTab(self._build_log_tab(), "Frame Log")
    self._tabs.addTab(self._build_scenarios_tab(), "Scenarios")
    self._tabs.addTab(self._build_decode_tab(), "Decoded Response")
    self._tabs.addTab(self._build_common_practice_tab(), "Common Practice")
    self._tabs.addTab(self._build_custom_tab(), "Custom Frame")
    self._tabs.addTab(self._build_timing_tab(), "Timing / Stats")
    self._tabs.setUsesScrollButtons(True)
    right_layout.addWidget(self._tabs)
    splitter.addWidget(right)

    splitter.setStretchFactor(0, 0)
    splitter.setStretchFactor(1, 1)

  def _build_connection_panel(self):
    grp = QGroupBox("CONNECTION")
    layout = QGridLayout(grp)
    layout.setSpacing(4)

    layout.addWidget(QLabel("Port"), 0, 0)
    self._cb_port = QComboBox()
    self._cb_port.setMinimumWidth(110)
    layout.addWidget(self._cb_port, 0, 1)

    btn_refresh = QPushButton("↺")
    btn_refresh.setFixedWidth(28)
    btn_refresh.setToolTip("Refresh ports")
    btn_refresh.clicked.connect(self._refresh_ports)
    layout.addWidget(btn_refresh, 0, 2)

    layout.addWidget(QLabel("Preambles"), 1, 0)
    self._spin_preambles = QSpinBox()
    self._spin_preambles.setRange(2, 30)
    self._spin_preambles.setValue(5)
    self._spin_preambles.setToolTip(
      "HART spec minimum is 5.\nRockwell PLCs often send 20.\nTry increasing if slave doesn't respond.")
    layout.addWidget(self._spin_preambles, 1, 1, 1, 2)

    layout.addWidget(QLabel("Master type"), 2, 0)
    self._cb_master = QComboBox()
    self._cb_master.addItems(["Primary (0x02)", "Secondary (0x01)"])
    self._cb_master.setToolTip(
      "Primary master: delimiter 0x02\nSecondary master: delimiter 0x01")
    layout.addWidget(self._cb_master, 2, 1, 1, 2)

    layout.addWidget(QLabel("Timeout (s)"), 3, 0)
    self._spin_timeout = QDoubleSpinBox()
    self._spin_timeout.setRange(0.5, 10.0)
    self._spin_timeout.setValue(2.0)
    self._spin_timeout.setSingleStep(0.5)
    layout.addWidget(self._spin_timeout, 3, 1, 1, 2)

    btn_row = QHBoxLayout()
    self._btn_connect = QPushButton("CONNECT")
    self._btn_connect.setObjectName("btn_connect")
    self._btn_connect.clicked.connect(self._do_connect)
    btn_row.addWidget(self._btn_connect)

    self._btn_disconnect = QPushButton("DISCONNECT")
    self._btn_disconnect.setObjectName("btn_disconnect")
    self._btn_disconnect.setEnabled(False)
    self._btn_disconnect.clicked.connect(self._do_disconnect)
    btn_row.addWidget(self._btn_disconnect)

    layout.addLayout(btn_row, 4, 0, 1, 3)

    self._lbl_conn_status = QLabel("● OFFLINE")
    self._lbl_conn_status.setObjectName("err")
    self._lbl_conn_status.setAlignment(Qt.AlignCenter)
    layout.addWidget(self._lbl_conn_status, 5, 0, 1, 3)


    return grp

  def _build_address_panel(self):
    grp = QGroupBox("DEVICE ADDRESS")
    layout = QGridLayout(grp)
    layout.setSpacing(4)

    layout.addWidget(QLabel("Address mode"), 0, 0)
    self._cb_addr_mode = QComboBox()
    self._cb_addr_mode.addItems(["Short (polling addr)", "Long (unique ID)"])
    self._cb_addr_mode.currentIndexChanged.connect(self._update_addr_mode)
    layout.addWidget(self._cb_addr_mode, 0, 1)

    layout.addWidget(QLabel("Poll addr (0-63)"), 1, 0)
    self._spin_addr = QSpinBox()
    self._spin_addr.setRange(0, 63)  # HART 7 polling addresses 0-63
    layout.addWidget(self._spin_addr, 1, 1)

    layout.addWidget(QLabel("Unique ID"), 2, 0)
    self._le_unique_id = QLineEdit()
    self._le_unique_id.setPlaceholderText("e.g. 27 8B 01 23 45")
    self._le_unique_id.setEnabled(False)
    self._le_unique_id.setToolTip(
      "5 bytes hex (HART 7): ExpDevType MSB (6 bits), ExpDevType LSB, DevID[3]\n"
      "Learned automatically from Cmd0 response.\nOr enter manually.")
    layout.addWidget(self._le_unique_id, 2, 1)

    self._lbl_unique_learned = QLabel("(not learned)")
    layout.addWidget(self._lbl_unique_learned, 3, 0, 1, 2)

    return grp

  def _build_commands_panel(self):
    grp = QGroupBox("UNIVERSAL COMMANDS")
    layout = QVBoxLayout(grp)
    layout.setSpacing(3)

    commands = [
      (0, "Cmd0 - Read Unique Identifier"),
      (1, "Cmd1 - Read Primary Variable"),
      (2, "Cmd2 - Read Loop Current + %"),
      (3, "Cmd3 - Read Dynamic Variables"),
      (6, "Cmd6 - Write Polling Address"),
      (11, "Cmd11 - Read Unique ID by Tag"),
      (12, "Cmd12 - Read Message"),
      (13, "Cmd13 - Read Tag/Descriptor/Date"),
      (14, "Cmd14 - Read PV Info"),
      (15, "Cmd15 - Read Output Info"),
      (16, "Cmd16 - Read Final Assembly"),
      (20, "Cmd20 - Read Long Tag"),
      (48, "Cmd48 - Read Additional Status"),
      (54, "Cmd54 - Read Device Variable Info"),
    ]

    for cmd_num, label in commands:
      btn = QPushButton(label)
      btn.setFixedHeight(22)
      if cmd_num == 11:
        btn.clicked.connect(self._send_cmd11_prompt)
      elif cmd_num == 54:
        btn.clicked.connect(self._send_cmd54_prompt)
      else:
        btn.clicked.connect(lambda checked, c=cmd_num, l=label: self._send_command(c, b"", l))
      layout.addWidget(btn)

    return grp

  def _send_cmd11_prompt(self):
    """Command 11 - Read Unique Identifier by Tag (HART 7).

    Prompts for an 8-character Packed ASCII tag (Bytes 0-5 of request),
    encodes it to 6 bytes Packed ASCII, and sends via broadcast short frame
    (address 0) or long frame if a unique ID is already learned, per spec.
    Response is identical to Command 0.
    """
    from PyQt5.QtWidgets import QInputDialog
    tag_str, ok = QInputDialog.getText(
      self,
      "Command 11 - Read Unique ID by Tag",
      "Enter Tag (up to 8 chars, Packed ASCII, uppercase):",
      text="TAG1"
    )
    if not ok:
      return
    tag_data = _encode_packed_ascii(tag_str)
    # Spec: the device's long frame address or the Broadcast Address (the
    # all-zero long address). Use the long address if learned, else broadcast.
    if self._unique_id:
      self._worker.queue_command(
        0, 11, tag_data,
        use_long=True, unique_id=self._unique_id,
        label="Cmd11 - Read Unique ID by Tag"
      )
    else:
      self._worker.queue_command(
        0, 11, tag_data,
        use_long=True, unique_id=BROADCAST_UNIQUE_ID,
        label="Cmd11 - Read Unique ID by Tag (broadcast)"
      )
    encoded_hex = " ".join(f"{b:02X}" for b in tag_data)
    self._log_info(
      f"  Cmd11: tag={repr(tag_str.upper().ljust(8)[:8])}  "
      f"packed={encoded_hex}  "
      f"frame={'long' if self._unique_id else 'long broadcast'}"
    )

  def _send_cmd54_prompt(self):
    from PyQt5.QtWidgets import QInputDialog
    # Set Device Variable Code as PV_CODE = 246 by default
    dv_code, ok = QInputDialog.getInt(self, "Command 54", "Enter Device Variable Code (0-255):", 246, 0, 255, 1)
    if ok:
      self._send_command(54, bytes([dv_code]), "Cmd54 - Read Device Variable Info")

  def _build_autopoll_panel(self):
    grp = QGroupBox("AUTO-POLL")
    layout = QGridLayout(grp)
    layout.setSpacing(4)

    layout.addWidget(QLabel("Command"), 0, 0)
    self._cb_poll_cmd = QComboBox()
    self._cb_poll_cmd.addItems(["0 - Identify", "1 - PV", "2 - Current+%", "3 - All vars"])
    layout.addWidget(self._cb_poll_cmd, 0, 1)

    layout.addWidget(QLabel("Interval (s)"), 1, 0)
    self._spin_poll_interval = QDoubleSpinBox()
    self._spin_poll_interval.setRange(0.5, 30.0)
    self._spin_poll_interval.setValue(2.0)
    self._spin_poll_interval.setSingleStep(0.5)
    layout.addWidget(self._spin_poll_interval, 1, 1)

    btn_row = QHBoxLayout()
    self._btn_poll_start = QPushButton("▶ START")
    self._btn_poll_start.setObjectName("btn_poll_start")
    self._btn_poll_start.clicked.connect(self._start_autopoll)
    btn_row.addWidget(self._btn_poll_start)

    self._btn_poll_stop = QPushButton("■ STOP")
    self._btn_poll_stop.setObjectName("btn_poll_stop")
    self._btn_poll_stop.setEnabled(False)
    self._btn_poll_stop.clicked.connect(self._stop_autopoll)
    btn_row.addWidget(self._btn_poll_stop)

    layout.addLayout(btn_row, 2, 0, 1, 2)

    return grp

  def _build_log_tab(self):
    w = QWidget()
    layout = QVBoxLayout(w)
    layout.setContentsMargins(4, 4, 4, 4)

    # Toolbar
    tb = QHBoxLayout()
    btn_clear = QPushButton("Clear")
    btn_clear.clicked.connect(self._clear_log)
    tb.addWidget(btn_clear)

    btn_save = QPushButton("Save log…")
    btn_save.clicked.connect(self._save_log)
    tb.addWidget(btn_save)

    self._chk_show_preamble = QCheckBox("Show preambles")
    self._chk_show_preamble.setChecked(False)
    tb.addWidget(self._chk_show_preamble)

    self._chk_timestamps = QCheckBox("Timestamps")
    self._chk_timestamps.setChecked(False)
    tb.addWidget(self._chk_timestamps)

    self._chk_annotate = QCheckBox("Annotate bytes")
    self._chk_annotate.setChecked(True)
    tb.addWidget(self._chk_annotate)

    tb.addStretch()
    layout.addLayout(tb)

    self._log = QTextEdit()
    self._log.setReadOnly(True)
    # Force document-level layout font properties
    font = QFont("Consolas", 10)
    self._log.setFont(font)
    self._log.document().setDefaultFont(font)
    layout.addWidget(self._log)

    return w

  def _build_decode_tab(self):
    w = QWidget()
    layout = QVBoxLayout(w)
    layout.setContentsMargins(4, 4, 4, 4)

    self._decode_table = QTableWidget(0, 2)
    self._decode_table.setHorizontalHeaderLabels(["Field", "Value"])
    self._decode_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
    self._decode_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
    self._decode_table.verticalHeader().setVisible(False)
    self._decode_table.setAlternatingRowColors(True)
    self._decode_table.setEditTriggers(QTableWidget.NoEditTriggers)
    layout.addWidget(self._decode_table)

    return w

  def _build_common_practice_tab(self):
    """One row per Common Practice command: button + request data field.
    Sent with the address mode selected on the left (long frame for HART 7)."""
    w = QWidget()
    outer = QVBoxLayout(w)
    outer.setContentsMargins(8, 8, 8, 8)
    outer.addWidget(QLabel("HART 7 Common Practice commands (request data, decimal):"))
    scroll = QScrollArea()
    scroll.setWidgetResizable(True)
    inner = QWidget()
    grid = QGridLayout(inner)
    grid.setSpacing(3)
    self._cp_inputs = {}
    for row, (cmd, label, kind, default, tip) in enumerate(CP_COMMANDS):
      btn = QPushButton(label)
      btn.setFixedHeight(22)
      grid.addWidget(btn, row, 0)
      if kind != "none":
        le = QLineEdit(default)
        le.setToolTip(tip)
        le.setPlaceholderText(tip)
        grid.addWidget(le, row, 1)
        self._cp_inputs[cmd] = le
      btn.clicked.connect(lambda checked, c=cmd, l=label: self._send_cp_command(c, l))
    grid.setColumnStretch(1, 1)
    scroll.setWidget(inner)
    outer.addWidget(scroll)
    return w

  def _send_cp_command(self, cmd: int, label: str):
    le = self._cp_inputs.get(cmd)
    try:
      data = encode_cp_request(cmd, le.text() if le else "")
    except (ValueError, OverflowError) as e:
      self._log_info(f"{label}: bad request data ({e})")
      return
    self._send_command(cmd, data, label)

  def _build_custom_tab(self):
    w = QWidget()
    layout = QVBoxLayout(w)
    layout.setContentsMargins(8, 8, 8, 8)
    layout.setSpacing(8)

    layout.addWidget(QLabel("Build and send a raw HART frame:"))

    g1 = QGridLayout()
    g1.addWidget(QLabel("Command (0-255)"), 0, 0)
    self._spin_custom_cmd = QSpinBox()
    self._spin_custom_cmd.setRange(0, 255)
    g1.addWidget(self._spin_custom_cmd, 0, 1)

    g1.addWidget(QLabel("Data bytes (hex)"), 1, 0)
    self._le_custom_data = QLineEdit()
    self._le_custom_data.setPlaceholderText("e.g.  01 02 03  (leave blank for no data)")
    g1.addWidget(self._le_custom_data, 1, 1)

    layout.addLayout(g1)

    # Preview
    self._lbl_frame_preview = QLabel("Frame preview: —")
    self._lbl_frame_preview.setObjectName("val")
    self._lbl_frame_preview.setWordWrap(True)
    layout.addWidget(self._lbl_frame_preview)

    self._spin_custom_cmd.valueChanged.connect(self._update_frame_preview)
    self._le_custom_data.textChanged.connect(self._update_frame_preview)

    btn_row = QHBoxLayout()
    btn_send = QPushButton("▶  SEND CUSTOM FRAME")
    btn_send.setObjectName("btn_send")
    btn_send.clicked.connect(self._send_custom)
    btn_row.addWidget(btn_send)

    btn_inject_bad = QPushButton("Send bad checksum")
    btn_inject_bad.setToolTip("Send frame with deliberate checksum error (tests slave error handling)")
    btn_inject_bad.clicked.connect(self._send_bad_checksum)
    btn_row.addWidget(btn_inject_bad)

    layout.addLayout(btn_row)

    layout.addStretch()
    return w

  def _build_timing_tab(self):
    w = QWidget()
    layout = QVBoxLayout(w)
    layout.setContentsMargins(8, 8, 8, 8)
    layout.setSpacing(8)

    layout.addWidget(QLabel("Response latency statistics (ms):"))

    self._timing_table = QTableWidget(0, 4)
    self._timing_table.setHorizontalHeaderLabels(["Command", "Last (ms)", "Min (ms)", "Max (ms)"])
    self._timing_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
    self._timing_table.verticalHeader().setVisible(False)
    self._timing_table.setEditTriggers(QTableWidget.NoEditTriggers)
    layout.addWidget(self._timing_table)

    self._timing_data = {}  # cmd → [latencies]

    layout.addWidget(QLabel("HART spec: ≥308 ms response window at 1200 baud"))

    layout.addStretch()
    return w

  # -- Slots --------------------------------------------------------------

  def _refresh_ports(self):
    current = self._cb_port.currentText()
    ports = [p.device for p in serial.tools.list_ports.comports()]
    self._cb_port.clear()
    if ports:
      self._cb_port.addItems(ports)
      if current in ports:
        self._cb_port.setCurrentText(current)
      elif "COM3" in ports:
        self._cb_port.setCurrentText("COM3")
    else:
      self._cb_port.addItem("(no ports)")

  def _update_addr_mode(self, idx):
    long_mode = idx == 1
    self._spin_addr.setEnabled(not long_mode)
    self._le_unique_id.setEnabled(long_mode)

  def _do_connect(self):
    port = self._cb_port.currentText()
    if not port or port == "(no ports)":
      self._log_info("No port selected")
      return
    master_map = {0: "primary", 1: "secondary"}
    master_type = master_map[self._cb_master.currentIndex()]
    ok = self._worker.connect_port(
      port,
      preambles=self._spin_preambles.value(),
      master_type=master_type,
      timeout=self._spin_timeout.value()
    )
    if ok:
      self._connected = True
      self._btn_connect.setEnabled(False)
      self._btn_disconnect.setEnabled(True)
      self._lbl_conn_status.setText("● ONLINE")
      self._lbl_conn_status.setObjectName("ok")
      self._lbl_conn_status.setStyleSheet("color: #4caf50;")
      self._status_bar.showMessage(
        f"Connected: {port}  |  1200 baud 8O1  |  "
        f"{self._spin_preambles.value()} preambles  |  {master_type} master"
      )
      self._log_info(f"Connected to {port}")

  def _do_disconnect(self):
    self._stop_autopoll()
    self._worker.disconnect_port()
    self._connected = False
    self._btn_connect.setEnabled(True)
    self._btn_disconnect.setEnabled(False)
    self._lbl_conn_status.setText("● OFFLINE")
    self._lbl_conn_status.setStyleSheet("color: #f44336;")
    self._status_bar.showMessage("Disconnected")
    self._log_info("Disconnected")

  def _send_command(self, cmd: int, data: bytes = b"", label: str = ""):
    if not self._connected:
      self._log_info("Not connected - cannot send")
      return
    use_long = self._cb_addr_mode.currentIndex() == 1
    unique_id = self._get_unique_id() if use_long else None
    self._worker.queue_command(
      self._spin_addr.value(), cmd, data,
      use_long=use_long, unique_id=unique_id, label=label or f"CMD {cmd}"
    )

  def _get_unique_id(self) -> bytes | None:
    text = self._le_unique_id.text().strip()
    if not text:
      return self._unique_id
    try:
      parts = [int(x, 16) for x in text.split()]
      if len(parts) != 5:
        raise ValueError("Need 5 bytes")
      return bytes(parts)
    except Exception:
      return self._unique_id

  def _send_custom(self):
    cmd = self._spin_custom_cmd.value()
    data_text = self._le_custom_data.text().strip()
    try:
      data = bytes.fromhex(data_text.replace(" ", "")) if data_text else b""
    except ValueError:
      self._log_info("Invalid hex data - check your input")
      return
    self._send_command(cmd, data, f"CUSTOM CMD {cmd}")

  def _send_bad_checksum(self):
    if not self._connected:
      return
    cmd = self._spin_custom_cmd.value()
    addr = self._spin_addr.value()
    frame = build_short_frame(addr, cmd, b"", self._spin_preambles.value())
    # Corrupt the last byte (checksum)
    frame = frame[:-1] + bytes([(frame[-1] ^ 0xFF)])
    self._log_info("Injecting BAD CHECKSUM frame - testing slave error response")
    self._worker._port.write(frame)
    self._worker._port.flush()
    self._log_frame("TX", "BAD CHECKSUM TEST", frame, 0)

  def _update_frame_preview(self):
    cmd = self._spin_custom_cmd.value()
    data_text = self._le_custom_data.text().strip()
    try:
      data = bytes.fromhex(data_text.replace(" ", "")) if data_text else b""
      frame = build_short_frame(self._spin_addr.value(), cmd, data,
                                self._spin_preambles.value())
      self._lbl_frame_preview.setText(
        "Frame:  " + " ".join(f"{b:02X}" for b in frame)
      )
    except Exception:
      self._lbl_frame_preview.setText("Frame preview: (invalid data)")

  def _start_autopoll(self):
    if not self._connected:
      return
    cmd_map = [0, 1, 2, 3]
    cmd = cmd_map[self._cb_poll_cmd.currentIndex()]
    interval = self._spin_poll_interval.value()
    addr = self._spin_addr.value()
    self._auto_poller = AutoPollWorker([(addr, cmd, b"")], interval)
    self._auto_poller.poll_tick.connect(
      lambda a, c, d: self._send_command(c, d, f"AUTO CMD {c}")
    )
    self._auto_poller.start()
    self._btn_poll_start.setEnabled(False)
    self._btn_poll_stop.setEnabled(True)
    self._log_info(f"Auto-poll started: CMD {cmd} every {interval:.1f}s")

  def _stop_autopoll(self):
    if self._auto_poller:
      self._auto_poller.stop()
      self._auto_poller = None
    self._btn_poll_start.setEnabled(True)
    self._btn_poll_stop.setEnabled(False)


  def _clear_log(self):
    self._log.clear()
    self._log_lines = 0
    self._session_log.clear()

  def _save_log(self):
    default_name = f"hart_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
    filename, _ = QFileDialog.getSaveFileName(self, "Save Session Log", default_name, "Text Files (*.txt);;All Files (*)")
    if not filename:
      return
    try:
      with open(filename, "w", encoding="utf-8") as f:
        f.write("\n".join(self._session_log))
      self._log_info(f"Log saved to {filename}")
    except Exception as e:
      QMessageBox.critical(self, "Save Error", f"Could not save log: {e}")
  # -- Signal handlers ----------------------------------------------------

  def _on_frame_logged(self, direction: str, label: str, data: bytes, latency_ms: float):
    # Add empty line before a new TX frame to separate transfer pairs
    if direction == "TX":
      self._append_log("")
    self._log_frame(direction, label, data, latency_ms)

  def _on_response(self, parsed: dict):
    if "error" in parsed:
      self._log_info(f"⚠  {parsed.get('_label', '')} - {parsed['error']}", color=LOG_RX_ERR_COLOR)
      return

    cmd = parsed.get("cmd", "?")
    label = parsed.get("_label", f"CMD {cmd}")
    latency = parsed.get("_latency_ms", 0)

    # Update timing stats
    key = str(cmd)
    if key not in self._timing_data:
      self._timing_data[key] = []
    self._timing_data[key].append(latency)
    self._refresh_timing_table()

    # Learn unique ID from Cmd0
    decoded = parsed.get("_decoded")
    if cmd == 0 and decoded:
      if "error" in decoded:
        self._log_info(
          f"⚠  Cmd0 decode error: {decoded['error']}",
          color=LOG_RX_ERR_COLOR,
        )
        self._lbl_unique_learned.setText("✗ Decode failed")
        self._lbl_unique_learned.setStyleSheet("color: #f87171;")
      elif "_unique_id" in decoded:
        uid = decoded["_unique_id"]
        self._unique_id = uid
        uid_str = " ".join(f"{b:02X}" for b in uid)
        self._le_unique_id.setText(uid_str)
        self._lbl_unique_learned.setText(f"✓ Learned: {uid_str}")
        self._lbl_unique_learned.setStyleSheet("color: #4caf50;")

    # Append parsed info to log
    cs_status = "✓ Checksum OK" if parsed.get("cs_ok") else "✗ Checksum FAIL"
    cs_color = LOG_RX_OK_COLOR if parsed.get("cs_ok") else LOG_RX_ERR_COLOR

    status_text = parsed['status_text']
    if not parsed.get("cs_ok"):
      status_text += ", CheckSum FAIL"

    lines = [
      (f"  └─ CMD={cmd}  st1=0x{parsed['st1']:02X}  st2=0x{parsed['st2']:02X}  "
       f"{cs_status}  latency={latency:.0f}ms", cs_color),
      (f"     Status: {status_text}", LOG_DECODED_COLOR if parsed.get("cs_ok") else LOG_RX_ERR_COLOR),
    ]
    if parsed["dev_flags"]:
      lines.append((f"     Device: {', '.join(parsed['dev_flags'])}", LOG_INFO_COLOR))

    for text, color in lines:
      self._append_log(text, color)

    # Fill decode table
    self._populate_decode_table(parsed)

  def _on_error(self, msg: str):
    self._log_info(f"ERROR: {msg}", color=LOG_RX_ERR_COLOR)


  # -- Log helpers --------------------------------------------------------
  def _log_frame(self, direction: str, label: str, data: bytes, latency_ms: float,
                 expected_silence: bool = False):
    if not data:
      if expected_silence:
        self._append_log(f"  [{direction}] {label} - (no response, expected)", LOG_RX_OK_COLOR)
      else:
        self._append_log(f"  [{direction}] {label} - (no data)", LOG_RX_ERR_COLOR)
      return

    color = LOG_TX_COLOR if direction == "TX" else LOG_RX_OK_COLOR
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3] if self._chk_timestamps.isChecked() else ""
    ts_part = f"[{ts}] " if ts else ""

    # Optionally hide preamble bytes
    display_data = data
    if not self._chk_show_preamble.isChecked():
      i = 0
      while i < len(data) and data[i] == 0xFF:
        i += 1
      display_data = data[i:]

    hex_str = " ".join(f"{b:02X}" for b in display_data)
    dir_arrow = "──▶" if direction == "TX" else "◀──"

    header = f"{ts_part}{dir_arrow} {direction}  {label}  ({len(data)}B)"
    self._append_log(header, color)

    if self._chk_annotate.isChecked():
      hex_aligned, ann_aligned = self._get_aligned_annotations(display_data, direction)
      self._append_log(f"  {hex_aligned}", color)
      self._append_log(f"  {ann_aligned}", LOG_MUTED)
    else:
      self._append_log(f"  {hex_str}", color)

    self._session_log.append(f"{header}\n  {hex_str}")


  def _get_aligned_annotations(self, data: bytes, direction: str) -> tuple[str, str]:
    raw_tokens = []
    ann_tokens = []
    i = 0

    # Preambles
    while i < len(data) and data[i] == 0xFF:
      raw_tokens.append(f"{data[i]:02X}")
      ann_tokens.append("PRE")
      i += 1

    if i < len(data):
      delimiter = data[i]
      is_long = (delimiter & 0x80) != 0
      raw_tokens.append(f"{delimiter:02X}")
      ann_tokens.append("DLM")
      i += 1

      if is_long:
        for j in range(5):
          if i < len(data):
            raw_tokens.append(f"{data[i]:02X}")
            ann_tokens.append(f"A{j}")
            i += 1
      else:
        if i < len(data):
          raw_tokens.append(f"{data[i]:02X}")
          ann_tokens.append("ADR")
          i += 1

      if i < len(data):
        raw_tokens.append(f"{data[i]:02X}")
        ann_tokens.append("CMD")
        i += 1

      if i < len(data):
        raw_tokens.append(f"{data[i]:02X}")
        cnt = data[i]
        ann_tokens.append("CNT")
        i += 1
      else:
        cnt = 0

    # For slave responses: first 2 data bytes are status
      if direction == "RX" and cnt >= 2:
        if i < len(data):
          raw_tokens.append(f"{data[i]:02X}")
          ann_tokens.append("ST1")
          i += 1
        if i < len(data):
          raw_tokens.append(f"{data[i]:02X}")
          ann_tokens.append("ST2")
          i += 1
        cnt -= 2

      for j in range(cnt):
        if i < len(data):
          raw_tokens.append(f"{data[i]:02X}")
          ann_tokens.append(f"D{j}")
          i += 1

      if i < len(data):
        raw_tokens.append(f"{data[i]:02X}")
        ann_tokens.append("CS")
        i += 1

    # Remaining trailing bytes
    while i < len(data):
      raw_tokens.append(f"{data[i]:02X}")
      ann_tokens.append("??")
      i += 1

    hex_out = []
    ann_out = []
    for r, a in zip(raw_tokens, ann_tokens):
      width = max(len(r), len(a))
      hex_out.append(f"{r:<{width}}")
      ann_out.append(f"{a:<{width}}")

    return "  ".join(hex_out), "  ".join(ann_out)


  def _log_info(self, msg: str, color: str = LOG_INFO_COLOR):
    leading = len(msg) - len(msg.lstrip("\n"))
    for _ in range(leading):
      self._append_log("", color)
      self._session_log.append("")
    clean = msg.lstrip("\n")
    self._append_log(f"  > {clean}", color)
    self._session_log.append(f"  > {clean}")

  def _append_log(self, text: str, color: str = "#c8cdd3"):
    cursor = self._log.textCursor()
    cursor.movePosition(QTextCursor.End)
    fmt = cursor.charFormat()
    fmt.setForeground(QColor(color))
    fmt.setFontFamily("Consolas")
    fmt.setFontPointSize(10)

    cursor.setCharFormat(fmt)
    cursor.insertText(text + "\n")
    self._log.setTextCursor(cursor)
    self._log.ensureCursorVisible()
    self._log_lines += 1
    if self._log_lines > self._max_log_lines:
      # Trim top
      cursor.movePosition(QTextCursor.Start)
      cursor.movePosition(QTextCursor.Down, QTextCursor.KeepAnchor, 50)
      cursor.removeSelectedText()
      self._log_lines -= 50


  def _populate_decode_table(self, parsed: dict):
    self._decode_table.setRowCount(0)

    rows = []

    # Header info
    rows.append(("Command", str(parsed.get("cmd", "?"))))
    rows.append(("Address mode", "Long (unique ID)" if parsed.get("is_long") else "Short (polling)"))
    rows.append(("Burst mode", "Yes" if parsed.get("is_burst") else "No"))
    rows.append(("Byte count", str(parsed.get("byte_count", "?"))))
    rows.append(("Status byte 1 (comm)", f"0x{parsed['st1']:02X} - {parsed['status_text']}"))
    rows.append(("Status byte 2 (device)", f"0x{parsed['st2']:02X}"))

    dev_flags = parsed.get("dev_flags", [])
    rows.append(("Device flags", ", ".join(dev_flags) if dev_flags else "(none)"))

    expected_cs_val = parsed.get("expected_cs")
    checksum_byte_val = parsed.get("checksum_byte")

    expected_cs_hex = f"0x{expected_cs_val:02X}" if expected_cs_val is not None else "None"
    checksum_byte_hex = f"0x{checksum_byte_val:02X}" if checksum_byte_val is not None else "None"

    cs_label = "✓ OK" if parsed.get("cs_ok") else f"✗ FAIL (got {checksum_byte_hex}, expected {expected_cs_hex})"
    rows.append(("Checksum", cs_label))
    rows.append(("Preambles received", str(parsed.get("preamble_count", "?"))))
    rows.append(("Latency", f"{parsed.get('_latency_ms', 0):.1f} ms"))
    rows.append(("Raw payload (hex)", parsed.get("payload", b"").hex(" ")))

    decoded = parsed.get("_decoded")
    if decoded:
      rows.append(("", ""))  # separator
      if "error" in decoded:
        rows.append(("⚠ DECODE ERROR", decoded["error"]))
      else:
        rows.append(("-- DECODED FIELDS --", ""))
        for k, v in decoded.items():
          if not k.startswith("_"):
            rows.append((k, str(v)))

    self._decode_table.setRowCount(len(rows))
    for i, (k, v) in enumerate(rows):
      item_k = QTableWidgetItem(k)
      item_v = QTableWidgetItem(v)

      # Boosted key column visibility from #94a3b8 (dark gray) to #cbd5e1 (light gray)
      item_k.setForeground(QColor("#cbd5e1"))

      if k.startswith("--") or k == "":
        item_k.setForeground(QColor("#60a5fa"))
        item_v.setForeground(QColor("#60a5fa"))
      elif "FAIL" in v.upper() or "ERROR" in v.upper():
        if "0x00" in v:
          item_v.setForeground(QColor("#93c5fd"))
        else:
          item_v.setForeground(QColor("#f87171"))
      elif v.startswith("✓"):
        # Boosted green from #34d399 to high-visibility #4ade80
        item_v.setForeground(QColor("#4ade80"))
      else:
        item_v.setForeground(QColor("#f1f5f9"))

      self._decode_table.setItem(i, 0, item_k)
      self._decode_table.setItem(i, 1, item_v)


  def _refresh_timing_table(self):
    self._timing_table.setRowCount(0)
    rows = []
    for cmd, latencies in sorted(self._timing_data.items()):
      rows.append((
        f"CMD {cmd}",
        f"{latencies[-1]:.0f}",
        f"{min(latencies):.0f}",
        f"{max(latencies):.0f}",
      ))
    self._timing_table.setRowCount(len(rows))
    for i, (cmd, last, mn, mx) in enumerate(rows):
      for j, val in enumerate((cmd, last, mn, mx)):
        item = QTableWidgetItem(val)
        item.setForeground(QColor("#c8cdd3"))
        if j == 3 and float(mx) > 500:
          item.setForeground(QColor("#ff9800"))
        self._timing_table.setItem(i, j, item)

  def _build_scenarios_tab(self):
    w = QWidget()
    layout = QVBoxLayout(w)
    layout.setContentsMargins(8, 8, 8, 8)
    layout.setSpacing(6)

    # Scenario selector
    sel_row = QHBoxLayout()
    sel_row.addWidget(QLabel("Scenario:"))
    self._cb_scenario = QComboBox()
    self._cb_scenario.setMinimumWidth(340)
    for name in SCENARIOS:
      self._cb_scenario.addItem(name)
    self._cb_scenario.setMaxVisibleItems(len(SCENARIOS))
    self._cb_scenario.currentIndexChanged.connect(self._on_scenario_changed)
    sel_row.addWidget(self._cb_scenario, 1)
    layout.addLayout(sel_row)

    # Description box
    self._lbl_scenario_desc = QTextEdit()
    self._lbl_scenario_desc.setReadOnly(True)
    self._lbl_scenario_desc.setMaximumHeight(100)
    self._lbl_scenario_desc.setStyleSheet(
      "QTextEdit { background: #0f172a; color: #f1f5f9; font-size: 11px; border: 1px solid #334155; }")
    layout.addWidget(self._lbl_scenario_desc)

    # Vertical splitter for table and hints box
    splitter = QSplitter(Qt.Vertical)
    layout.addWidget(splitter)

    # Upper container (table, progress, summary, buttons, rand_row)
    upper_w = QWidget()
    upper_layout = QVBoxLayout(upper_w)
    upper_layout.setContentsMargins(0, 0, 0, 0)
    upper_layout.setSpacing(6)

    # Step table
    self._scenario_table = ScenarioTableWidget(0, 4)
    self._scenario_table.setHorizontalHeaderLabels(["#", "Label", "Result", "Latency"])
    header = self._scenario_table.horizontalHeader()
    header.setSectionResizeMode(QHeaderView.Interactive)
    self._scenario_table.setColumnWidth(0, 45)
    self._scenario_table.setColumnWidth(1, 840)
    self._scenario_table.setColumnWidth(2, 160)
    self._scenario_table.setColumnWidth(3, 90)
    self._scenario_table.verticalHeader().setVisible(False)
    self._scenario_table.setEditTriggers(QTableWidget.NoEditTriggers)
    self._scenario_table.setAlternatingRowColors(True)
    upper_layout.addWidget(self._scenario_table)

    # Progress bar
    self._scenario_progress = QProgressBar()
    self._scenario_progress.setValue(0)
    self._scenario_progress.setTextVisible(True)
    self._scenario_progress.setStyleSheet(
      "QProgressBar { border: 1px solid #334155; background: #0f172a; color: #f1f5f9; height: 16px; }"
      "QProgressBar::chunk { background: #8D49B2; }"
    )
    upper_layout.addWidget(self._scenario_progress)

    # Summary label
    self._lbl_scenario_summary = QLabel("Ready")
    self._lbl_scenario_summary.setAlignment(Qt.AlignCenter)
    upper_layout.addWidget(self._lbl_scenario_summary)

    # Buttons
    btn_row = QHBoxLayout()
    self._btn_scenario_run = QPushButton("▶  RUN SCENARIO")
    self._btn_scenario_run.setObjectName("btn_send")
    self._btn_scenario_run.clicked.connect(self._run_scenario)
    btn_row.addWidget(self._btn_scenario_run)

    self._btn_scenario_run_all = QPushButton("▶▶  RUN ALL")
    self._btn_scenario_run_all.setObjectName("btn_send")
    self._btn_scenario_run_all.clicked.connect(self._run_all_scenarios)
    btn_row.addWidget(self._btn_scenario_run_all)

    self._btn_scenario_abort = QPushButton("■  ABORT")
    self._btn_scenario_abort.setObjectName("btn_disconnect")
    self._btn_scenario_abort.setEnabled(False)
    self._btn_scenario_abort.clicked.connect(self._abort_scenario)
    btn_row.addWidget(self._btn_scenario_abort)

    upper_layout.addLayout(btn_row)

    # Random order option + run-all progress label
    rand_row = QHBoxLayout()
    self._chk_random_order = QCheckBox("Random order")
    self._chk_random_order.setToolTip(
      "Shuffle scenario order using a time-based seed.\n"
      "Each Run All session uses a different shuffle.")
    rand_row.addWidget(self._chk_random_order)
    rand_row.addStretch()
    self._lbl_run_all_progress = QLabel("")
    self._lbl_run_all_progress.setObjectName("val")
    rand_row.addWidget(self._lbl_run_all_progress)
    upper_layout.addLayout(rand_row)

    splitter.addWidget(upper_w)

    # Diagnostic hints box
    self._scenario_hints = QTextEdit()
    self._scenario_hints.setReadOnly(True)
    self._scenario_hints.setStyleSheet(
      "QTextEdit { background: #0f172a; color: #ffff00; font-size: 11px; border: 1px solid #334155; }")
    palette = self._scenario_hints.palette()
    palette.setColor(QPalette.PlaceholderText, QColor("#d0d064"))
    self._scenario_hints.setPalette(palette)
    self._scenario_hints.setPlaceholderText("Diagnostic hints will appear here as steps run…")
    splitter.addWidget(self._scenario_hints)

    splitter.setStretchFactor(0, 1)
    splitter.setStretchFactor(1, 0)

    # Init description
    self._on_scenario_changed(0)
    return w

  def _on_scenario_changed(self, idx):
    name = self._cb_scenario.currentText()
    sc = SCENARIOS.get(name, {})
    self._lbl_scenario_desc.setPlainText(sc.get("description", ""))
    steps = sc.get("steps", [])
    self._scenario_table.setRowCount(len(steps))
    for i, step in enumerate(steps):
      self._scenario_table.setItem(i, 0, QTableWidgetItem(str(i + 1)))
      lbl_item = QTableWidgetItem(step.get("label", ""))
      lbl_item.setForeground(QColor("#f1f5f9"))
      self._scenario_table.setItem(i, 1, lbl_item)
      self._scenario_table.setItem(i, 2, QTableWidgetItem("-"))
      self._scenario_table.setItem(i, 3, QTableWidgetItem("-"))
    self._scenario_progress.setValue(0)
    self._scenario_progress.setMaximum(max(len(steps), 1))
    self._lbl_scenario_summary.setText(f"{len(steps)} steps - press RUN to start")
    self._scenario_hints.clear()

  def _start_runner(self, name: str, steps: list, done_slot):
    """Create and start a ScenarioRunner for `steps` (shared by Run / Run All)."""
    self._on_scenario_changed(self._cb_scenario.currentIndex())
    self._scenario_hints.clear()
    self._active_steps = list(steps)

    self._scenario_runner = ScenarioRunner(
      port=self._worker._port,
      steps=self._active_steps,
      unique_id=self._get_unique_id(),
      poll_addr=self._spin_addr.value(),
    )
    self._scenario_runner.step_started.connect(self._on_scenario_step_start)
    self._scenario_runner.step_done.connect(self._on_scenario_step_done)
    self._scenario_runner.step_result.connect(self._on_scenario_step_result)
    self._scenario_runner.step_error.connect(self._on_scenario_step_error)
    self._scenario_runner.uid_learned.connect(self._on_scenario_uid_learned)
    self._scenario_runner.manual_prompt.connect(self._on_scenario_manual_prompt)
    self._scenario_runner.scenario_done.connect(done_slot)

    self._scenario_progress.setMaximum(len(steps))
    self._scenario_progress.setValue(0)
    self._btn_scenario_run.setEnabled(False)
    self._btn_scenario_run_all.setEnabled(False)
    self._btn_scenario_abort.setEnabled(True)
    self._lbl_scenario_summary.setText(f"Running: {name}")
    self._scenario_runner.start()

  def _run_scenario(self):
    if not self._connected:
      self._log_info("Not connected - cannot run scenario")
      return
    if not self._worker._port or not self._worker._port.is_open:
      self._log_info("Port not open")
      return

    name = self._cb_scenario.currentText()
    sc = SCENARIOS.get(name, {})
    steps = sc.get("steps", [])
    if not steps:
      return

    self._log_info(f"\n\n═══ SCENARIO START: {name} ═══")
    self._start_runner(name, steps, self._on_scenario_done)

  def _run_all_scenarios(self):
    if not self._connected:
      self._log_info("Not connected - cannot run scenarios")
      return
    if not self._worker._port or not self._worker._port.is_open:
      self._log_info("Port not open")
      return

    import random
    names = [n for n in SCENARIOS.keys() if not SCENARIOS[n].get("manual_only")]
    skipped = [n for n in SCENARIOS.keys() if SCENARIOS[n].get("manual_only")]
    if self._chk_random_order.isChecked():
      seed = int(time.time() * 1000) & 0xFFFFFFFF
      rng = random.Random(seed)
      rng.shuffle(names)
      self._log_info(f"\n\n═══ RUN ALL: random order ═══")
      self._log_info("Order: " + " → ".join(f"#{list(SCENARIOS.keys()).index(n)+1}" for n in names))
    else:
      self._log_info("\n\n═══ RUN ALL: sequential order ═══")
    if skipped:
      self._log_info("Skipped (operator interaction needed, run individually): " + "; ".join(skipped))

    self._run_all_queue = names
    self._run_all_total = len(names)
    self._run_all_ok = 0
    self._run_all_fail = 0
    self._btn_scenario_run.setEnabled(False)
    self._btn_scenario_run_all.setEnabled(False)
    self._btn_scenario_abort.setEnabled(True)
    self._advance_run_all()

  def _advance_run_all(self):
    if not self._run_all_queue:
      done = self._run_all_total
      summary = (f"Run All done: {self._run_all_ok}/{done} scenarios fully OK, "
                 f"{self._run_all_fail} had failures")
      self._log_info(f"═══ {summary} ═══")
      self._lbl_run_all_progress.setText("")
      self._btn_scenario_run.setEnabled(True)
      self._btn_scenario_run_all.setEnabled(True)
      self._btn_scenario_abort.setEnabled(False)
      return

    name = self._run_all_queue.pop(0)
    remaining = len(self._run_all_queue)
    current = self._run_all_total - remaining
    self._lbl_run_all_progress.setText(f"Scenario {current}/{self._run_all_total}")

    # Select scenario in combo for visual feedback
    idx = self._cb_scenario.findText(name)
    if idx >= 0:
      self._cb_scenario.setCurrentIndex(idx)

    sc = SCENARIOS.get(name, {})
    steps = sc.get("steps", [])
    if not steps:
      self._advance_run_all()
      return

    self._log_info(f"\n\n═══ SCENARIO START [{current}/{self._run_all_total}]: {name} ═══")
    self._start_runner(name, steps, self._on_run_all_step_done)

  def _on_run_all_step_done(self, ok_count: int, fail_count: int):
    """Called after each scenario completes during Run All."""
    if fail_count == 0:
      self._run_all_ok += 1
    else:
      self._run_all_fail += 1
    self._on_scenario_done(ok_count, fail_count)
    # Continue only if queue is still active (not aborted)
    if self._run_all_queue is not None:
      QTimer.singleShot(800, self._advance_run_all)

  def _abort_scenario(self):
    if self._scenario_runner:
      self._scenario_runner.abort()
    self._btn_scenario_run.setEnabled(True)
    self._btn_scenario_run_all.setEnabled(True)
    self._btn_scenario_abort.setEnabled(False)
    self._lbl_scenario_summary.setText("Aborted")
    self._lbl_run_all_progress.setText("")
    self._run_all_queue = None
    self._log_info("═══ SCENARIO ABORTED ═══")

  def _on_scenario_step_start(self, idx: int, label: str, note: str):
    if idx > 0:
      self._append_log("")     # Add empty line before each new step
    self._log_info(f"  [{idx+1}] {label}")
    if note:
      self._scenario_hints.append(f"[{idx+1}] {label}\n    → {note}\n")
      cursor = self._scenario_hints.textCursor()
      cursor.movePosition(QTextCursor.End)
      self._scenario_hints.setTextCursor(cursor)
    item = self._scenario_table.item(idx, 2)
    if item:
      item.setText("⏳ running")
      item.setForeground(QColor("#fbbf24"))

  def _active_step(self, idx: int) -> dict:
    steps = getattr(self, "_active_steps", None) or []
    return steps[idx] if idx < len(steps) else {}

  def _on_scenario_step_done(self, idx: int, label: str,
                              tx: bytes, rx: bytes, latency_ms: float):
    """Log the exchange. Pass/fail is decided by the runner (step_result)."""
    self._scenario_progress.setValue(idx + 1)
    # Feed latency into Timing/Stats tab (same as _on_response does for manual commands)
    if rx:
      parsed_for_timing = parse_response(rx)
      if parsed_for_timing and parsed_for_timing.get("ok"):
        key = str(parsed_for_timing.get("cmd", "?"))
        if key not in self._timing_data:
          self._timing_data[key] = []
        self._timing_data[key].append(latency_ms)
        self._refresh_timing_table()
    step_def = self._active_step(idx)
    expect_no_resp = step_def.get("expect_no_response", False)
    step_addr = step_def.get("_addr", None)
    addr_mismatch = (step_addr is not None and step_addr != self._spin_addr.value()
                     and not step_is_strict(step_def))
    expected_silence = expect_no_resp or (addr_mismatch and not rx)

    self._log_frame("TX", label, tx, 0.0)
    self._log_frame("RX", label, rx, latency_ms, expected_silence=expected_silence)

    lat_item = self._scenario_table.item(idx, 3)
    if not lat_item:
      lat_item = QTableWidgetItem()
      self._scenario_table.setItem(idx, 3, lat_item)
    lat_item.setText(f"{latency_ms:.0f} ms" if rx else "timeout")
    lat_item.setForeground(QColor("#e2e8f0"))

    parsed = parse_response(rx)

    # Append status/latency summary lines matching manual command output
    if parsed and parsed.get("ok"):
      cmd_num = parsed.get("cmd", "?")
      cs_status = "✓ Checksum OK" if parsed.get("cs_ok") else "✗ Checksum FAIL"
      cs_color = LOG_RX_OK_COLOR if parsed.get("cs_ok") else LOG_RX_ERR_COLOR
      status_text = parsed["status_text"]
      if not parsed.get("cs_ok"):
        status_text += ", CheckSum FAIL"
      summary_lines = [
        (f"  └─ CMD={cmd_num}  st1=0x{parsed['st1']:02X}  st2=0x{parsed['st2']:02X}  "
         f"{cs_status}  latency={latency_ms:.0f}ms", cs_color),
        (f"     Status: {status_text}",
         LOG_DECODED_COLOR if parsed.get("cs_ok") else LOG_RX_ERR_COLOR),
      ]
      if parsed.get("dev_flags"):
        summary_lines.append(
          (f"     Device: {', '.join(parsed['dev_flags'])}", LOG_INFO_COLOR))
      decoded = decode_response(parsed.get("cmd", -1), parsed.get("payload", b""))
      if decoded and "error" not in decoded:
        fields = [f"{k}={v}" for k, v in decoded.items() if not k.startswith("_") and v is not None]
        if fields:
          summary_lines.append(("     Data: " + "; ".join(fields), LOG_DECODED_COLOR))
      for text, color in summary_lines:
        self._append_log(text, color)

  def _on_scenario_step_result(self, idx: int, ok: bool, reason: str):
    self._scenario_progress.setValue(idx + 1)
    step_def = self._active_step(idx)
    if ok:
      if step_def.get("expect_no_response") or reason.startswith("silent"):
        result_text = "✓ SILENT (expected)"
      else:
        result_text = "✓ OK"
      result_color = "#4ade80"
    else:
      result_text = f"✗ {reason}"
      result_color = "#f87171"
      hint = reason
      if reason == "no response":
        hint = "No response - check: baud/parity (1200 8O1), preamble count, address, wiring."
      elif reason.startswith("got a response") and step_def.get("_corrupt_cs"):
        hint = ("Slave responded to a frame with a bad checksum.\n"
                "    Conformant slave must silently discard corrupted frames.")
      elif reason.startswith("got a response"):
        hint = ("Slave responded to a frame that must be ignored (broadcast/silence expected).\n"
                "    This causes bus collisions in multi-drop. Check address filtering.")
      self._scenario_hints.append(f"  ⚠ Step {idx+1}: {hint}\n")
    if ok and INFO_SEP in reason:
      self._scenario_hints.append(f"  ℹ Step {idx+1}: {reason.split(INFO_SEP, 1)[1]}\n")
    self._log_info(f"     Result: {'PASS' if ok else 'FAIL'} - {reason}",
                   color=LOG_RX_OK_COLOR if ok else LOG_RX_ERR_COLOR)

    result_item = self._scenario_table.item(idx, 2)
    if not result_item:
      result_item = QTableWidgetItem()
      self._scenario_table.setItem(idx, 2, result_item)
    result_item.setText(result_text)
    result_item.setToolTip(reason)
    result_item.setForeground(QColor(result_color))

  def _on_scenario_uid_learned(self, uid: bytes):
    self._unique_id = uid
    uid_str = " ".join(f"{b:02X}" for b in uid)
    self._le_unique_id.setText(uid_str)
    self._lbl_unique_learned.setText(f"✓ Learned: {uid_str}")
    self._lbl_unique_learned.setStyleSheet("color: #4caf50;")

  def _on_scenario_manual_prompt(self, idx: int, text: str):
    self._log_info(f"     Waiting for operator: {text}")
    answer = QMessageBox.question(
      self, f"Scenario step {idx+1} - operator action", text + "\n\nPress OK when done.",
      QMessageBox.Ok | QMessageBox.Cancel, QMessageBox.Ok)
    if self._scenario_runner:
      self._scenario_runner.confirm_manual(answer == QMessageBox.Ok)

  def _on_scenario_step_error(self, idx: int, label: str, msg: str):
    self._scenario_progress.setValue(idx + 1)
    result_item = self._scenario_table.item(idx, 2)
    if result_item:
      result_item.setText(f"✗ ERR")
      result_item.setForeground(QColor("#f87171"))
    self._log_info(f"  Step {idx+1} IO error: {msg}", color=LOG_RX_ERR_COLOR)
    self._scenario_hints.append(f"  ⚠ Step {idx+1} IO error: {msg}\n")

  def _on_scenario_done(self, ok_count: int, fail_count: int):
    self._btn_scenario_run.setEnabled(True)
    self._btn_scenario_run_all.setEnabled(True)
    self._btn_scenario_abort.setEnabled(False)
    total = ok_count + fail_count
    summary = f"Done: {ok_count}/{total} OK, {fail_count} failed"
    self._lbl_scenario_summary.setText(summary)
    color = "#4ade80" if fail_count == 0 else ("#fbbf24" if ok_count > 0 else "#f87171")
    self._lbl_scenario_summary.setStyleSheet(f"color: {color}; font-weight: bold;")
    self._log_info(f"═══ SCENARIO DONE: {summary} ═══")

    if fail_count > 0:
      self._scenario_hints.append(
        "\n--- COMMON CAUSES FOR FAILED STEPS ---\n"
        "• Preamble count mismatch (slave needs ≥5, Rockwell sends 20)\n"
        "• HART baud must be exactly 1200, 8O1\n"
        "• Half-duplex: TX echoed back on RX - use RS-485 modem or HART modem\n"
        "• Primary/secondary master is address byte bit 7 (delimiter is 0x02/0x82 for both)\n"
        "• Slave only handles short-addr frames, rejects long-addr\n"
        "• Configuration Changed stays set until Cmd38 (per master); Cold Start clears\n"
        "  after the first response to each master\n"
        "• HART revision mismatch (slave HART5 vs master expects HART7)\n"
      )

  def closeEvent(self, event):
    self._stop_autopoll()
    self._worker.stop()
    self._worker.disconnect_port()
    event.accept()


# --- ENTRY POINT ------------------------------------------------------------

def main():
  app = QApplication(sys.argv)
#  with open("style.qss", "r", encoding="utf-8") as f:
#    app.setStyleSheet(f.read())
  with open("style.qss", "r", encoding="utf-8") as f:
    STYLE = f.read()
  app.setApplicationName("HART Master Simulator")
  win = HartMasterSim()
  win.show()
  sys.exit(app.exec_())


if __name__ == "__main__":
  main()
