#!/usr/bin/env python3
"""
HART Master Simulator
=====================
Simulates an industrial HART master (PLC-class) for debugging HART slave devices.
Targets compatibility with Rockwell 5094-IF4IHSXT and Emerson Trex behavior.

Requirements:
  pip install pyserial PyQt5
"""

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


# --- HART PROTOCOL CONSTANTS ------------------------------------------------

HART_BAUD = 1200
HART_PARITY = serial.PARITY_ODD
HART_STOPBITS = serial.STOPBITS_ONE
HART_BYTESIZE = serial.EIGHTBITS

# Delimiter bytes
DELIM_SHORT_MASTER  = 0x02  # Short frame, primary master → slave
DELIM_SHORT_SECOND  = 0x01  # Short frame, secondary master → slave
DELIM_LONG_MASTER   = 0x82  # Long frame, primary master → slave
DELIM_LONG_SECOND   = 0x81  # Long frame, secondary master → slave
DELIM_SHORT_SLAVE   = 0x06  # Short frame, slave → master
DELIM_LONG_SLAVE    = 0x86  # Long frame, slave → master

UNIT_CODES = {
  0: "Special", 1: "in H2O @ 68°F", 2: "in Hg @ 0°C", 3: "ft H2O @ 68°F",
  4: "mm H2O @ 68°F", 5: "mm Hg @ 0°C", 6: "psi", 7: "bar", 8: "mbar",
  9: "g/cm²", 10: "kg/cm²", 11: "Pa", 12: "kPa", 13: "torr",
  32: "°C", 33: "°F", 34: "°R", 35: "K",
  39: "mA", 40: "%", 41: "rpm", 42: "Hz", 43: "m/s", 44: "ft/s",
  57: "m³/s", 58: "m³/min", 59: "m³/h", 74: "l/s", 75: "l/min", 76: "l/h",
  250: "Not classified", 251: "Not used",
}

COMM_STATUS_BITS = {
  0x80: "Framing error",
  0x40: "Vertical parity error",
  0x20: "Overrun error",
  0x10: "Longitudinal parity error",
  0x08: "Reserved",
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
  64: "Command not implemented",
}

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


def build_short_frame(address: int, command: int, data: bytes = b"",
                      preamble_count: int = 5,
                      master_type: str = "primary") -> bytes:
  delim = DELIM_SHORT_MASTER if master_type == "primary" else DELIM_SHORT_SECOND
  addr_byte = address & 0x3F
  frame_body = bytes([delim, addr_byte, command, len(data)]) + data
  cs = calc_checksum(frame_body)
  return bytes([0xFF] * preamble_count) + frame_body + bytes([cs])


def build_long_frame(unique_id: bytes, command: int, data: bytes = b"",
                     preamble_count: int = 5,
                     master_type: str = "primary") -> bytes:
  """unique_id: 5 bytes (manufacturer[1] + devtype[1] + devid[3])"""
  delim = DELIM_LONG_MASTER if master_type == "primary" else DELIM_LONG_SECOND
  # Long address: bit7 of first byte = 1 marks slave address
  addr_bytes = bytes([unique_id[0] | 0x80]) + unique_id[1:5]
  frame_body = bytes([delim]) + addr_bytes + bytes([command, len(data)]) + data
  cs = calc_checksum(frame_body)
  return bytes([0xFF] * preamble_count) + frame_body + bytes([cs])


def parse_response(raw: bytes) -> dict | None:
  if not raw:
    return None

  # Skip preambles
  i = 0
  while i < len(raw) and raw[i] == 0xFF:
    i += 1

  preamble_count = i

  if i >= len(raw):
    return {"error": "Only preamble bytes received", "preamble_count": preamble_count}

  delimiter = raw[i]
  is_long = (delimiter & 0x80) != 0
  is_burst = (delimiter & 0x40) != 0

  try:
    if is_long:
      if i + 8 > len(raw):
        return {"error": "Frame too short (long addr)", "raw": raw}
      addr = raw[i+1:i+6]
      cmd = raw[i+6]
      byte_count = raw[i+7]
      frame_start = i
      data_start = i + 8
    else:
      if i + 4 > len(raw):
        return {"error": "Frame too short (short addr)", "raw": raw}
      addr = raw[i+1:i+2]
      cmd = raw[i+2]
      byte_count = raw[i+3]
      frame_start = i
      data_start = i + 4

    expected_len = data_start + byte_count + 1  # +1 for checksum
    actual_payload = raw[data_start:data_start + byte_count]
    checksum_byte = raw[data_start + byte_count] if data_start + byte_count < len(raw) else None

    # Verify checksum
    frame_for_cs = raw[frame_start:data_start + byte_count]
    expected_cs = calc_checksum(frame_for_cs)
    cs_ok = (checksum_byte == expected_cs) if checksum_byte is not None else False

    # Status bytes
    st1 = actual_payload[0] if len(actual_payload) > 0 else 0
    st2 = actual_payload[1] if len(actual_payload) > 1 else 0
    payload = actual_payload[2:] if len(actual_payload) > 2 else b""

    # Decode comm status vs response code
    if st1 & 0x80:
      comm_errors = [desc for bit, desc in COMM_STATUS_BITS.items() if st1 & bit and bit != 0x80]
      status_text = "COMM ERROR: " + (", ".join(comm_errors) if comm_errors else "unknown")
    else:
      status_text = RESPONSE_CODES.get(st1, f"Response code {st1}")

    dev_flags = [desc for bit, desc in DEVICE_STATUS_BITS.items() if st2 & bit]

    return {
      "ok": True,
      "delimiter": delimiter,
      "is_long": is_long,
      "is_burst": is_burst,
      "address": addr,
      "cmd": cmd,
      "byte_count": byte_count,
      "st1": st1,
      "st2": st2,
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


# --- COMMAND DECODERS -------------------------------------------------------

def decode_cmd0(payload: bytes) -> dict:
  if len(payload) < 12:
    return {"error": f"Too short: {len(payload)} bytes (need 12)"}
  mfr_id = payload[0]
  dev_type = payload[1]
  preambles = payload[2]
  hart_rev = payload[3]
  dev_rev = payload[4]
  sw_rev = payload[5]
  hw_rev = payload[6]
  flags = payload[7]
  dev_id = payload[8:11]
  num_resp_pre = payload[11] if len(payload) > 11 else None
  return {
    "Manufacturer ID": f"0x{mfr_id:02X} ({KNOWN_MANUFACTURERS.get(mfr_id, 'Unknown')})",
    "Device type code": f"0x{dev_type:02X}",
    "Min preambles req": preambles,
    "HART revision": hart_rev,
    "Device revision": dev_rev,
    "Software revision": sw_rev,
    "Hardware revision": hw_rev >> 3,
    "Physical signaling": hw_rev & 0x07,
    "Flags": f"0x{flags:02X}",
    "Device ID": f"{dev_id[0]:02X}:{dev_id[1]:02X}:{dev_id[2]:02X}",
    "Response preambles": num_resp_pre,
    "_unique_id": bytes([mfr_id, dev_type]) + bytes(dev_id),
  }


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
  if len(payload) < 6:
    return {"error": f"Too short: {len(payload)} bytes"}
  pv_code = payload[0]
  units = payload[1]
  upper = struct.unpack(">f", payload[2:6])[0]
  lower = struct.unpack(">f", payload[6:10])[0] if len(payload) >= 10 else float("nan")
  min_span = struct.unpack(">f", payload[10:14])[0] if len(payload) >= 14 else float("nan")
  return {
    "PV transducer S/N": f"{pv_code}",
    "PV units": f"{units} ({UNIT_CODES.get(units, 'unknown')})",
    "Upper sensor limit": f"{upper:.4f}",
    "Lower sensor limit": f"{lower:.4f}",
    "Minimum span": f"{min_span:.4f}",
  }


def decode_cmd48(payload: bytes) -> dict:
  if not payload:
    return {"error": "No payload"}
  lines = {}
  for i, b in enumerate(payload):
    lines[f"Extended status byte {i}"] = f"0x{b:02X} ({b:08b}b)"
  return lines


def _decode_packed_ascii(data: bytes) -> str:
  """Decode HART packed-ASCII (6-bit encoding, 3 bytes → 4 chars)."""
  result = []
  for i in range(0, len(data) - 2, 3):
    b0, b1, b2 = data[i], data[i+1], data[i+2]
    result.append(chr(((b0 >> 2) & 0x3F) + 0x20))
    result.append(chr(((b0 << 4 | b1 >> 4) & 0x3F) + 0x20))
    result.append(chr(((b1 << 2 | b2 >> 6) & 0x3F) + 0x20))
    result.append(chr((b2 & 0x3F) + 0x20))
  return "".join(result)


def decode_response(cmd: int, payload: bytes) -> dict | None:
  decoders = {
    0: decode_cmd0,
    1: decode_cmd1,
    2: decode_cmd2,
    3: decode_cmd3,
    12: decode_cmd12,
    13: decode_cmd13,
    14: decode_cmd14,
    48: decode_cmd48,
  }
  fn = decoders.get(cmd)
  return fn(payload) if fn else None


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

        self._port.reset_input_buffer()
        t_send = time.perf_counter()
        self._port.write(frame)
        self._port.flush()
        self.frame_logged.emit("TX", label, frame, 0.0)

        # Read response
        time.sleep(0.3)
        raw = b""
        deadline = time.perf_counter() + self._timeout
        while time.perf_counter() < deadline:
          waiting = self._port.in_waiting
          if waiting:
            raw += self._port.read(waiting)
            time.sleep(0.05)
          elif raw:
            time.sleep(0.15)
            if not self._port.in_waiting:
              break
          else:
            time.sleep(0.05)

        latency_ms = (time.perf_counter() - t_send) * 1000.0
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


  def stop(self):
    self._running = False
    self.wait()


# --- SNIFFER WORKER ---------------------------------------------------------

class SnifferWorker(QThread):
  """Passively drains the serial port and emits every complete HART frame found."""
  frame_sniffed = pyqtSignal(bytes, str)  # raw_bytes, info_str

  def __init__(self, port: serial.Serial):
    super().__init__()
    self._port = port
    self._running = False

  def stop(self):
    self._running = False
    self.wait()

  def run(self):
    self._running = True
    buf = b""
    last_activity = time.perf_counter()

    while self._running:
      try:
        waiting = self._port.in_waiting
      except Exception:
        time.sleep(0.05)
        continue

      if waiting:
        try:
          chunk = self._port.read(waiting)
        except Exception:
          time.sleep(0.05)
          continue
        buf += chunk
        last_activity = time.perf_counter()
      else:
        # Flush accumulated buffer if bus has been idle ≥20ms (≥24 bit-times @ 1200 baud)
        if buf and (time.perf_counter() - last_activity) > 0.020:
          self._process(buf)
          buf = b""
        time.sleep(0.005)
        continue

    # Flush remainder on exit
    if buf:
      self._process(buf)

  def _process(self, raw: bytes):
    """Strip preambles, emit frame bytes + decoded info string."""
    # Strip leading 0xFF preamble bytes
    i = 0
    while i < len(raw) and raw[i] == 0xFF:
      i += 1
    stripped = raw[i:]
    if not stripped:
      return

    info_parts = []

    # Delimiter byte
    delim = stripped[0] if stripped else None
    if delim is not None:
      addr_type = "long" if (delim & 0x80) else "short"
      frame_type_bits = delim & 0x06
      frame_type = {0x02: "STX(master→slave)", 0x06: "ACK(slave→master)",
                    0x01: "STX(2nd master)", 0x00: "BACK(burst)"}.get(frame_type_bits, f"delim=0x{delim:02X}")
      master_bit = "primary" if (delim & 0x01) == 0 else "secondary"
      info_parts.append(f"addr={addr_type}  type={frame_type}  master={master_bit}")

    # Try to extract command byte and byte count
    preamble_count = i
    info_parts.insert(0, f"preambles={preamble_count}")

    if addr_type == "short" and len(stripped) >= 3:
      addr_byte = stripped[1] & 0x0F
      cmd_byte  = stripped[2]
      info_parts.append(f"addr={addr_byte}  cmd={cmd_byte}")
      if len(stripped) >= 4:
        byte_count = stripped[3]
        info_parts.append(f"byte_count={byte_count}")
    elif addr_type == "long" and len(stripped) >= 7:
      cmd_byte  = stripped[6]
      info_parts.append(f"cmd={cmd_byte}")
      if len(stripped) >= 8:
        byte_count = stripped[7]
        info_parts.append(f"byte_count={byte_count}")

    # Checksum check (last byte = XOR of all bytes from delimiter onward)
    if len(stripped) >= 2:
      cs_calc = 0
      for b in stripped[:-1]:
        cs_calc ^= b
      cs_ok = cs_calc == stripped[-1]
      info_parts.append(f"CheckSum={'OK' if cs_ok else 'FAIL'}")

    info = "  |  ".join(info_parts)
    self.frame_sniffed.emit(raw, info)


# --- STYLE ------------------------------------------------------------------

STYLE = """
QMainWindow {
  background: #000000;
}
QWidget {
  background: #000000;
  color: #f1f5f9;
  font-family: 'Courier New', monospace;
  font-size: 12px;
}
QGroupBox {
  border: 1px solid #475569;
  border-radius: 4px;
  margin-top: 8px;
  padding-top: 4px;
  font-size: 12px;
  color: #ffffff;
  font-family: 'Courier New', monospace;
  font-weight: bold;
}
QGroupBox::title {
  subcontrol-origin: margin;
  left: 8px;
  padding: 0 4px;
}
QPushButton {
  background: #1e293b;
  border: 1px solid #475569;
  border-radius: 3px;
  padding: 5px 12px;
  color: #f1f5f9;
  font-family: 'Courier New', monospace;
  font-size: 12px;
}
QPushButton:hover {
  background: #334155;
  border-color: #3b82f6;
  color: #ffffff;
}
QPushButton:pressed {
  background: #0f172a;
}
QPushButton:disabled {
  color: #475569;
  border-color: #1e293b;
}
QPushButton#btn_connect {
  background: #064e3b;
  border-color: #059669;
  color: #34d399;
}
QPushButton#btn_connect:hover {
  background: #047857;
  border-color: #34d399;
}
QPushButton#btn_disconnect {
  background: #7f1d1d;
  border-color: #dc2626;
  color: #f87171;
}
QPushButton#btn_disconnect:hover {
  background: #991b1b;
  border-color: #f87171;
}
QPushButton#btn_send {
  background: #1e3a8a;
  border-color: #2563eb;
  color: #60a5fa;
  font-size: 12px;
  padding: 6px 16px;
}
QPushButton#btn_send:hover {
  background: #1d4ed8;
  border-color: #60a5fa;
  color: #93c5fd;
}
QPushButton#btn_poll_start {
  background: #064e3b;
  border-color: #059669;
  color: #34d399;
}
QPushButton#btn_poll_stop {
  background: #7f1d1d;
  border-color: #dc2626;
  color: #f87171;
}
QComboBox {
  background: #1e293b;
  border: 1px solid #475569;
  border-radius: 3px;
  padding: 4px 8px;
  color: #f1f5f9;
}
QComboBox::drop-down {
  border: none;
  width: 20px;
}
QComboBox QAbstractItemView {
  background: #1e293b;
  border: 1px solid #475569;
  selection-background-color: #2563eb;
}
QSpinBox, QDoubleSpinBox, QLineEdit {
  background: #1e293b;
  border: 1px solid #475569;
  border-radius: 3px;
  padding: 4px 8px;
  color: #f1f5f9;
}
QSpinBox:focus, QLineEdit:focus {
  border-color: #3b82f6;
}
QTextEdit {
  background: #000000;
  border: 1px solid #334155;
  border-radius: 3px;
  color: #f1f5f9;
  font-family: 'Courier New', monospace;
  font-size: 14px;
}
QTabWidget::pane {
  border: 1px solid #334155;
  background: #000000;
}
QTabBar {
  /* Fix for QTabBar (Tabs): Force transparent focus indicator */
  outline: 0;
}
QTabBar::tab:focus {
  /* Instead of a dotted box, indicate focus with a clean colored border */
  border-top: 3px solid #3b82f6;
}
/* Fix for QAbstractItemView (Tables, Lists, Trees) */
QAbstractItemView {
  outline: 0;
}

/* Fix for Buttons and Checkboxes */
QPushButton, QCheckBox, QRadioButton, QComboBox {
  /* Override focus visual representation by explicitly defining focus styles */
  border: 1px solid #475569;
}

QPushButton:focus, QCheckBox:focus, QComboBox:focus {
  /* Highlight the active control using a distinct border color instead of a dotted rect */
  border-color: #3b82f6;
}
QTabBar::tab {
  background: #1e293b;
  border: 1px solid #334155;
  border-bottom: none;
  padding: 8px 14px;             /* Reduced horizontal padding to prevent truncation */
  min-width: 120px;              /* Explicit minimum width to fit text layout */
  color: #cbd5e1;
  font-size: 13px;
  font-family: 'Segoe UI', Arial, sans-serif;
  font-weight: bold;
}
QTabBar::tab:selected {
  background: #000000;
  color: #ffffff;
  border-top: 3px solid #3b82f6;
}
QTabBar::tab:hover {
  color: #ffffff;
  background: #273549;
}
QLabel {
  color: #ffffff;
  font-size: 12px;
}
QLabel#val {
  color: #f1f5f9;
  font-size: 12px;
}
QLabel#bright {
  color: #3b82f6;
  font-size: 12px;
}
QLabel#ok {
  color: #34d399;
}
QLabel#warn {
  color: #fbbf24;
}
QLabel#err {
  color: #f87171;
}
QCheckBox {
  color: #f1f5f9;
}
QStatusBar {
  background: #000000;
  color: #94a3b8;
  border-top: 1px solid #334155;
  font-size: 11px;
}
QSplitter::handle {
  background: #334155;
}
QScrollBar:vertical {
  background: #000000;
  width: 8px;
  border: none;
}
QScrollBar::handle:vertical {
  background: #475569;
  border-radius: 4px;
  min-height: 20px;
}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
  height: 0;
}
QTableWidget {
  background: #000000;
  gridline-color: #334155;
  border: 1px solid #334155;
  /* Add explicit background for alternating rows to prevent bright white/grey mash */
  qproperty-alternatingRowColors: true;
}
QTableWidget::item {
  padding: 3px 6px;
}
/* Crucial fix for contrast: explicit background for alternating rows */
QTableWidget::item:alternate {
  background: #0f172a;
}
QHeaderView::section {
  background: #1e293b;
  border: 1px solid #334155;
  padding: 4px 6px;
  color: #e2e8f0;
  font-size: 11px;
  font-weight: bold;
}
/* Fix for Buttons and Checkboxes outline*/
QPushButton, QCheckBox, QRadioButton, QComboBox {
  /* Override focus visual representation by explicitly defining focus styles */
  border: 1px solid #475569;
  outline: 0;
}
"""

# Color tags for log text
LOG_TX_COLOR = "#60a5fa"
LOG_RX_OK_COLOR = "#34d399"
LOG_RX_ERR_COLOR = "#f87171"
LOG_INFO_COLOR = "#fbbf24"
LOG_DECODED_COLOR = "#93c5fd"
LOG_MUTED = "#94a3b8"


# --- SCENARIO ENGINE --------------------------------------------------------

# Each scenario is a list of steps executed sequentially by ScenarioRunner.
# Step dict keys:
#   label      str   – shown in log
#   cmd        int   – HART command number
#   data       bytes – command data payload
#   use_long   bool  – use long (unique-ID) address frame
#   preambles  int|None – override preamble count for this step
#   master     str|None – "primary" | "secondary"
#   delay_pre  float – sleep BEFORE sending (seconds)
#   delay_post float – sleep AFTER response received (seconds)
#   note       str   – diagnostic hint shown in log before step

SCENARIOS = {
  "1 - Cold-Start Enumeration (Rockwell style)": {
    "description": (
      "Mimics Rockwell 5094-IF4IHSXT startup sequence:\n"
      "• 20 preambles (Rockwell default)\n"
      "• Short-addr Cmd0 to discover unique ID\n"
      "• Switch to long-addr Cmd0 to confirm\n"
      "• Cmd1/Cmd2/Cmd3 with long addr\n"
      "Diagnoses: slave ignores long addr, wrong preamble count,\n"
      "           no response to addr=0 short frame."
    ),
    "steps": [
      {"label": "Cmd0 short addr (20 preambles)", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 20, "master": "primary",
       "delay_pre": 0.5, "delay_post": 0.3,
       "note": "Rockwell sends 20 preambles on init. If slave needs ≥5 this is fine."},
      {"label": "Cmd0 long addr (20 preambles)", "cmd": 0, "data": b"",
       "use_long": True,  "preambles": 20, "master": "primary",
       "delay_pre": 0.5, "delay_post": 0.3,
       "note": "Long addr uses unique ID from previous step. Slave must accept both."},
      {"label": "Cmd1 PV (long addr)", "cmd": 1, "data": b"",
       "use_long": True,  "preambles": 20, "master": "primary",
       "delay_pre": 0.3, "delay_post": 0.2,
       "note": "Normal PV read with long addr."},
      {"label": "Cmd2 Current+% (long addr)", "cmd": 2, "data": b"",
       "use_long": True,  "preambles": 20, "master": "primary",
       "delay_pre": 0.3, "delay_post": 0.2,
       "note": "Loop current + percent of range."},
      {"label": "Cmd3 Dynamic vars (long addr)", "cmd": 3, "data": b"",
       "use_long": True,  "preambles": 20, "master": "primary",
       "delay_pre": 0.3, "delay_post": 0.2,
       "note": "All dynamic variables."},
      {"label": "Cmd48 Ext Status (long addr)", "cmd": 48, "data": b"",
       "use_long": True,  "preambles": 20, "master": "primary",
       "delay_pre": 0.3, "delay_post": 0.2,
       "note": "Extended device status - PLCs poll this on startup."},
    ]
  },

  "2 - Preamble Count Sweep (3/5/8/16/20)": {
    "description": (
      "Sends Cmd0 with increasing preamble counts.\n"
      "Slave must respond to ≥5 preambles (HART spec minimum).\n"
      "3 preambles: slave MAY ignore (below spec).\n"
      "Rockwell uses 20, Emerson Trex uses 5.\n"
      "Diagnoses: slave preamble detector threshold."
    ),
    "steps": [
      {"label": "Cmd0 - 3 preambles", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 3, "master": "primary",
       "delay_pre": 0.5, "delay_post": 0.5,
       "note": "3 preambles - BELOW spec minimum. Slave may legitimately ignore."},
      {"label": "Cmd0 - 5 preambles", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.5, "delay_post": 0.5,
       "note": "5 preambles - HART spec minimum. Slave MUST respond."},
      {"label": "Cmd0 - 8 preambles", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 8, "master": "primary",
       "delay_pre": 0.5, "delay_post": 0.5,
       "note": "8 preambles - common default."},
      {"label": "Cmd0 - 16 preambles", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 16, "master": "primary",
       "delay_pre": 0.5, "delay_post": 0.5,
       "note": "16 preambles - conservative industrial default."},
      {"label": "Cmd0 - 20 preambles", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 20, "master": "primary",
       "delay_pre": 0.5, "delay_post": 0.5,
       "note": "20 preambles - Rockwell 5094 default. Slave must handle."},
    ]
  },

  "3 - Secondary Master Probe (Emerson Trex)": {
    "description": (
      "Emerson Trex communicates as secondary master (delimiter 0x01/0x81).\n"
      "Sends Cmd0 first as primary then as secondary.\n"
      "Slave must respond to both.\n"
      "Diagnoses: slave rejects secondary master delimiter,\n"
      "           or hardcodes primary-only check."
    ),
    "steps": [
      {"label": "Cmd0 - primary master", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.5, "delay_post": 0.5,
       "note": "Baseline: primary master delimiter 0x02."},
      {"label": "Cmd0 - secondary master", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "secondary",
       "delay_pre": 0.5, "delay_post": 0.5,
       "note": "Secondary master delimiter 0x01. Slave must also respond."},
      {"label": "Cmd1 PV - secondary master", "cmd": 1, "data": b"",
       "use_long": False, "preambles": 5, "master": "secondary",
       "delay_pre": 0.3, "delay_post": 0.3,
       "note": "Normal read via secondary master."},
      {"label": "Cmd3 Dynamic - secondary master", "cmd": 3, "data": b"",
       "use_long": False, "preambles": 5, "master": "secondary",
       "delay_pre": 0.3, "delay_post": 0.3,
       "note": "All vars via secondary master."},
    ]
  },

  "4 - Multi-Drop Address Sweep (addr 0–15)": {
    "description": (
      "Polls Cmd0 at every short address 0–15.\n"
      "Standard multi-drop enumeration - PLCs do this on bus startup.\n"
      "Diagnoses: slave responds on wrong address,\n"
      "           does not respond on its configured address,\n"
      "           or responds to broadcast address 0 only."
    ),
    "steps": [
      {"label": f"Cmd0 - addr {addr}", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "_addr": addr,
       "delay_pre": 0.2, "delay_post": 0.4,
       "note": f"Poll address {addr}. Only the device at this address should respond."}
      for addr in range(16)
    ]
  },

  "5 - Short/Long Address Interleave": {
    "description": (
      "Alternates between short-addr and long-addr frames.\n"
      "Some slaves mis-handle address mode switches mid-session.\n"
      "Diagnoses: slave confuses address mode, loses sync,\n"
      "           or fails to echo correct delimiter in response."
    ),
    "steps": [
      {"label": "Cmd0 short addr", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.4, "delay_post": 0.3,
       "note": "Short address frame. Response delimiter should be 0x06."},
      {"label": "Cmd0 long addr", "cmd": 0, "data": b"",
       "use_long": True,  "preambles": 5, "master": "primary",
       "delay_pre": 0.4, "delay_post": 0.3,
       "note": "Long address frame. Response delimiter should be 0x86."},
      {"label": "Cmd1 short addr", "cmd": 1, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.3, "delay_post": 0.3,
       "note": "Back to short addr."},
      {"label": "Cmd1 long addr", "cmd": 1, "data": b"",
       "use_long": True,  "preambles": 5, "master": "primary",
       "delay_pre": 0.3, "delay_post": 0.3,
       "note": "Long addr again."},
      {"label": "Cmd3 short addr", "cmd": 3, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.3, "delay_post": 0.3,
       "note": "Short addr, all variables."},
      {"label": "Cmd3 long addr", "cmd": 3, "data": b"",
       "use_long": True,  "preambles": 5, "master": "primary",
       "delay_pre": 0.3, "delay_post": 0.3,
       "note": "Long addr, all variables."},
    ]
  },

  "6 - Rapid-Fire Stress (gap/timing)": {
    "description": (
      "Sends Cmd1 repeatedly with short inter-frame gaps.\n"
      "Tests slave's ability to handle back-to-back requests.\n"
      "1200 baud = ~8.33ms/byte. HART spec requires ≥2 char times gap.\n"
      "Diagnoses: slave buffer overrun, missed frames,\n"
      "           wrong 'overrun error' bit in status byte."
    ),
    "steps": [
      {"label": f"Cmd1 rapid #{i+1}", "cmd": 1, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.05, "delay_post": 0.0,
       "note": "Minimal inter-frame gap (50ms). Tests slave receiver reset speed."}
      for i in range(10)
    ]
  },

  "7 - Bad Checksum Injection × 3 then Valid": {
    "description": (
      "Sends 3 frames with corrupted checksums, then a valid Cmd0.\n"
      "Slave must:\n"
      "  • Silently discard or NAK bad-CheckSum frames\n"
      "  • Recover and respond to the valid frame\n"
      "Diagnoses: slave locks up after bad frame, never recovers,\n"
      "           or responds to bad-CheckSum frame (wrong behavior)."
    ),
    "steps": [
      {"label": "BAD CheckSum frame #1", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.5, "delay_post": 0.5,
       "note": "Checksum XOR'd with 0xFF - slave must discard.",
       "_corrupt_cs": True},
      {"label": "BAD CheckSum frame #2", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.5, "delay_post": 0.5,
       "note": "Second bad-CS frame.",
       "_corrupt_cs": True},
      {"label": "BAD CheckSum frame #3", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.5, "delay_post": 0.5,
       "note": "Third bad-CS frame.",
       "_corrupt_cs": True},
      {"label": "VALID Cmd0 - recovery check", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.8, "delay_post": 0.5,
       "note": "Valid frame after 3 bad ones. Slave MUST respond - tests recovery."},
    ]
  },

  "8 - HART Revision Probe (Cmd0 → check revision byte)": {
    "description": (
      "Sends Cmd0 and inspects HART revision field.\n"
      "HART 5 = 0x05, HART 6 = 0x06, HART 7 = 0x07.\n"
      "Rockwell 5094 requires HART 6 or 7.\n"
      "Emerson Trex supports HART 5/6/7.\n"
      "Diagnoses: slave reports HART 5 to a HART 7 master\n"
      "           causing master to abort session."
    ),
    "steps": [
      {"label": "Cmd0 - check HART revision", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.5, "delay_post": 0.5,
       "note": "Check byte[3] of response payload: 5=HARTv5, 6=HARTv6, 7=HARTv7."},
      {"label": "Cmd0 long - cross-check", "cmd": 0, "data": b"",
       "use_long": True,  "preambles": 5, "master": "primary",
       "delay_pre": 0.5, "delay_post": 0.5,
       "note": "Long addr repeat - confirm same revision reported."},
      {"label": "Cmd48 - extended status", "cmd": 48, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.4, "delay_post": 0.4,
       "note": "HART 7 devices must implement Cmd48. HART 5 may return cmd-not-implemented (64)."},
    ]
  },

  "9 - Full PLC Session (Rockwell + Emerson combined)": {
    "description": (
      "Full realistic session combining both device patterns:\n"
      "Phase 1 (Rockwell): 20 preambles, primary master, long addr\n"
      "Phase 2 (Emerson):  5 preambles, secondary master, short+long addr\n"
      "Phase 3: rapid polling as primary\n"
      "Comprehensive end-to-end compatibility check."
    ),
    "steps": [
      # Phase 1: Rockwell
      {"label": "[Rockwell] Cmd0 short (20 pre)", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 20, "master": "primary",
       "delay_pre": 0.5, "delay_post": 0.4,
       "note": "Rockwell phase: discover device."},
      {"label": "[Rockwell] Cmd0 long (20 pre)", "cmd": 0, "data": b"",
       "use_long": True,  "preambles": 20, "master": "primary",
       "delay_pre": 0.4, "delay_post": 0.4,
       "note": "Rockwell phase: confirm long addr."},
      {"label": "[Rockwell] Cmd1 PV", "cmd": 1, "data": b"",
       "use_long": True,  "preambles": 20, "master": "primary",
       "delay_pre": 0.3, "delay_post": 0.3,
       "note": "Rockwell phase: read PV."},
      {"label": "[Rockwell] Cmd2 Current", "cmd": 2, "data": b"",
       "use_long": True,  "preambles": 20, "master": "primary",
       "delay_pre": 0.3, "delay_post": 0.3,
       "note": "Rockwell phase: read loop current."},
      {"label": "[Rockwell] Cmd48 Ext Status", "cmd": 48, "data": b"",
       "use_long": True,  "preambles": 20, "master": "primary",
       "delay_pre": 0.3, "delay_post": 0.5,
       "note": "Rockwell phase: extended status."},
      # Phase 2: Emerson Trex
      {"label": "[Emerson] Cmd0 short (5 pre, secondary)", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "secondary",
       "delay_pre": 0.6, "delay_post": 0.4,
       "note": "Emerson Trex phase: secondary master, 5 preambles."},
      {"label": "[Emerson] Cmd13 Tag/Descriptor", "cmd": 13, "data": b"",
       "use_long": False, "preambles": 5, "master": "secondary",
       "delay_pre": 0.3, "delay_post": 0.3,
       "note": "Emerson reads tag/descriptor."},
      {"label": "[Emerson] Cmd14 PV Info", "cmd": 14, "data": b"",
       "use_long": False, "preambles": 5, "master": "secondary",
       "delay_pre": 0.3, "delay_post": 0.3,
       "note": "Emerson reads PV transducer info."},
      # Phase 3: rapid primary poll
      {"label": "[Poll] Cmd1 #1", "cmd": 1, "data": b"",
       "use_long": True,  "preambles": 5, "master": "primary",
       "delay_pre": 0.2, "delay_post": 0.1,
       "note": "Rapid poll phase."},
      {"label": "[Poll] Cmd1 #2", "cmd": 1, "data": b"",
       "use_long": True,  "preambles": 5, "master": "primary",
       "delay_pre": 0.2, "delay_post": 0.1,
       "note": "Rapid poll phase."},
      {"label": "[Poll] Cmd1 #3", "cmd": 1, "data": b"",
       "use_long": True,  "preambles": 5, "master": "primary",
       "delay_pre": 0.2, "delay_post": 0.1,
       "note": "Rapid poll phase."},
    ]
  },

  "10 - Identity Read (Cmd12/13/14)": {
    "description": (
      "Reads device identity information in the order PLCs use during commissioning:\n"
      "• Cmd0  – identify device, learn unique ID\n"
      "• Cmd12 – read message string (32-char packed ASCII)\n"
      "• Cmd13 – read tag, descriptor, date\n"
      "• Cmd14 – read PV transducer info (units, upper/lower limits, min span)\n"
      "Diagnoses: slave omits optional identity commands, returns wrong packed-ASCII,\n"
      "           or rejects Cmd14 on non-pressure devices."
    ),
    "steps": [
      {"label": "Cmd0 - identify device", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.5, "delay_post": 0.3,
       "note": "Baseline identification. Unique ID learned for long-addr steps."},
      {"label": "Cmd12 - read message", "cmd": 12, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.3, "delay_post": 0.3,
       "note": "Read 32-char packed-ASCII message field. Must return 24 bytes payload."},
      {"label": "Cmd13 - tag/descriptor/date", "cmd": 13, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.3, "delay_post": 0.3,
       "note": "Read tag (8 char), descriptor (16 char), date (3 bytes). Must return 21 bytes."},
      {"label": "Cmd14 - PV transducer info", "cmd": 14, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.3, "delay_post": 0.3,
       "note": "PV units, upper/lower sensor limits, min span. Must return ≥14 bytes."},
      {"label": "Cmd14 - long addr repeat", "cmd": 14, "data": b"",
       "use_long": True,  "preambles": 5, "master": "primary",
       "delay_pre": 0.3, "delay_post": 0.3,
       "note": "Same read via long addr – confirm consistent response."},
    ]
  },

  "11 - Write-Protect Probe (Cmd17/18/19)": {
    "description": (
      "Attempts configuration writes to verify write-protect behavior:\n"
      "• Cmd17 – write message (should return 0x07 if write-protected)\n"
      "• Cmd18 – write tag/descriptor/date\n"
      "• Cmd19 – write final assembly number\n"
      "In write-protect mode, slave MUST return response code 0x07.\n"
      "Diagnoses: slave silently ignores writes, corrupts config,\n"
      "           or fails to set write-protect bit in status byte."
    ),
    "steps": [
      {"label": "Cmd0 - baseline status", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.5, "delay_post": 0.3,
       "note": "Read status before write attempts. Note 'Analog output fixed' bit."},
      {"label": "Cmd17 - write message (24 bytes)", "cmd": 17,
       "data": bytes([0x20] * 24),  # packed-ASCII spaces
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.3, "delay_post": 0.4,
       "note": "Write blank message. If write-protected: expect RC=0x07. If not: message overwritten."},
      {"label": "Cmd18 - write tag/descriptor/date", "cmd": 18,
       "data": bytes([0x20] * 21),  # spaces tag + descriptor + date
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.3, "delay_post": 0.4,
       "note": "Write blank tag and descriptor. Expect RC=0x07 if write-protected."},
      {"label": "Cmd19 - write final assembly number", "cmd": 19,
       "data": bytes([0x00, 0x00, 0x00]),
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.3, "delay_post": 0.4,
       "note": "Write assembly number 0. Expect RC=0x07 if write-protected."},
      {"label": "Cmd0 - verify no config change", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.4, "delay_post": 0.3,
       "note": "'Configuration changed' bit should NOT be set if write-protect worked."},
    ]
  },

  "12 - Configuration Changed Recovery": {
    "description": (
      "Simulates the PLC cold-start / config-changed handling sequence:\n"
      "• Cmd0 – check for 'Cold start' (0x20) and 'Config changed' (0x40) bits\n"
      "• Cmd48 – read extended status to capture full snapshot\n"
      "• Cmd0 × 3 – re-poll until status bits clear (PLCs retry up to N times)\n"
      "Diagnoses: slave never clears Cold start bit, Config changed sticky after Cmd0,\n"
      "           or 'More status available' bit loops indefinitely."
    ),
    "steps": [
      {"label": "Cmd0 - initial (check cold-start/config bits)", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.5, "delay_post": 0.4,
       "note": "ST2 byte: 0x20=ColdStart, 0x40=ConfigChanged, 0x10=MoreStatusAvail."},
      {"label": "Cmd48 - extended status snapshot", "cmd": 48, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.3, "delay_post": 0.3,
       "note": "Capture full extended status. Must send after 'More status available' bit set."},
      {"label": "Cmd0 - retry #1 (expect bits clearing)", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.5, "delay_post": 0.4,
       "note": "Cold-start and config-changed bits should clear after first valid Cmd0 exchange."},
      {"label": "Cmd0 - retry #2", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.5, "delay_post": 0.4,
       "note": "Second retry - bits must be clear by now on a conformant device."},
      {"label": "Cmd1 - PV after recovery", "cmd": 1, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.3, "delay_post": 0.3,
       "note": "Confirm normal operation restored after status-bit recovery sequence."},
      {"label": "Cmd3 - all vars after recovery", "cmd": 3, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.3, "delay_post": 0.3,
       "note": "Confirm all dynamic variables readable post-recovery."},
    ]
  },

  "13 - Device Variable Commands (Cmd9/33)": {
    "description": (
      "HART 7 device variable commands:\n"
      "• Cmd9  – read device variables with status (up to 4 variable codes)\n"
      "• Cmd33 – read device variables (HART 5/6 compatible subset)\n"
      "• Cmd3  – fallback all-vars read for comparison\n"
      "Diagnoses: HART 7 slave does not implement Cmd9 (returns RC=64),\n"
      "           variable status byte always 0x00 (no status support),\n"
      "           or returns wrong variable count."
    ),
    "steps": [
      {"label": "Cmd0 - check HART revision", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.5, "delay_post": 0.3,
       "note": "Byte[3] of payload: 7 = HART7 required for Cmd9. 5/6 = expect RC=64 on Cmd9."},
      # Cmd9: request 4 device variable codes (0=PV,1=SV,2=TV,3=QV)
      {"label": "Cmd9 - device vars with status (PV/SV/TV/QV)", "cmd": 9,
       "data": bytes([0x00, 0x01, 0x02, 0x03]),
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.4, "delay_post": 0.3,
       "note": "HART7 Cmd9: request variable codes 0,1,2,3. Each var returns 8 bytes + status."},
      {"label": "Cmd9 - just PV (code 0)", "cmd": 9,
       "data": bytes([0x00, 0xFF, 0xFF, 0xFF]),
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.4, "delay_post": 0.3,
       "note": "Cmd9 with only PV requested (0xFF = not used). Minimal variable read."},
      {"label": "Cmd33 - read device variables", "cmd": 33,
       "data": bytes([0x00]),  # slot 0
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.4, "delay_post": 0.3,
       "note": "Cmd33: HART 5/6/7 compatible device variable read."},
      {"label": "Cmd3 - all dynamic vars (baseline)", "cmd": 3, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.3, "delay_post": 0.3,
       "note": "Cmd3 as baseline - compare values against Cmd9/33 results."},
    ]
  },

  "14 - Broadcast / Global Address (addr 63)": {
    "description": (
      "Tests slave behavior on broadcast address (short addr 63, 0x3F):\n"
      "• HART spec: address 63 is global broadcast - slaves must NOT respond\n"
      "• Slave responding to addr 63 causes bus collisions in multi-drop\n"
      "• Followed by valid addressed Cmd0 to confirm slave is still alive\n"
      "Diagnoses: slave incorrectly responds to broadcast,\n"
      "           slave locks up after seeing broadcast address."
    ),
    "steps": [
      {"label": "Cmd0 - normal addr (baseline)", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.5, "delay_post": 0.4,
       "note": "Baseline: normal addressed Cmd0. Slave must respond."},
      {"label": "Cmd0 - broadcast addr 63 (expect NO response)", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "_addr": 63,
       "delay_pre": 0.5, "delay_post": 0.6,
       "note": "Broadcast addr 0x3F. Conformant slave MUST NOT respond. Timeout = pass."},
      {"label": "Cmd0 - normal addr (recovery check)", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.5, "delay_post": 0.4,
       "note": "Slave must still respond normally after seeing broadcast."},
      {"label": "Cmd1 - PV after broadcast", "cmd": 1, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.3, "delay_post": 0.3,
       "note": "Confirm normal data flow after broadcast test."},
    ]
  },
}


class ScenarioRunner(QThread):
  """Executes a scenario step list sequentially on the serial port."""
  step_started  = pyqtSignal(int, str, str)       # step_idx, label, note
  step_done     = pyqtSignal(int, str, bytes, bytes, float)  # step_idx, label, tx, rx, latency_ms
  step_error    = pyqtSignal(int, str, str)        # step_idx, label, error_msg
  scenario_done = pyqtSignal(int, int)             # steps_ok, steps_fail

  def __init__(self, port: serial.Serial, steps: list, unique_id: bytes | None,
               poll_addr: int):
    super().__init__()
    self._port = port
    self._steps = steps
    self._unique_id = unique_id
    self._poll_addr = poll_addr
    self._abort = False

  def abort(self):
    self._abort = True

  def run(self):
    ok_count = 0
    fail_count = 0

    for idx, step in enumerate(self._steps):
      if self._abort:
        break

      label    = step.get("label", f"Step {idx}")
      note     = step.get("note", "")
      cmd      = step["cmd"]
      data     = step.get("data", b"")
      use_long = step.get("use_long", False)
      preambles = step.get("preambles", 5)
      master   = step.get("master", "primary")
      delay_pre  = step.get("delay_pre", 0.3)
      delay_post = step.get("delay_post", 0.2)
      corrupt_cs = step.get("_corrupt_cs", False)

      self.step_started.emit(idx, label, note)

      time.sleep(delay_pre)

      try:
        # Build frame
        if use_long and self._unique_id:
          frame = build_long_frame(self._unique_id, cmd, data, preambles, master)
        else:
          # Support per-step address override (multi-drop sweep)
          step_addr = step.get("_addr", self._poll_addr)
          frame = build_short_frame(step_addr, cmd, data, preambles, master)

        if corrupt_cs:
          frame = frame[:-1] + bytes([frame[-1] ^ 0xFF])

        self._port.reset_input_buffer()
        t0 = time.perf_counter()
        self._port.write(frame)
        self._port.flush()

        # Collect response
        time.sleep(0.3)
        raw = b""
        deadline = time.perf_counter() + 2.0
        while time.perf_counter() < deadline:
          waiting = self._port.in_waiting
          if waiting:
            raw += self._port.read(waiting)
            time.sleep(0.05)
          elif raw:
            time.sleep(0.15)
            if not self._port.in_waiting:
              break
          else:
            time.sleep(0.05)

        latency_ms = (time.perf_counter() - t0) * 1000.0
        self.step_done.emit(idx, label, frame, raw, latency_ms)

        parsed = parse_response(raw)
        if parsed and parsed.get("ok"):
          ok_count += 1
          # Update unique ID if we learned it
          if cmd == 0 and not corrupt_cs:
            decoded = decode_response(0, parsed.get("payload", b""))
            if decoded and "_unique_id" in decoded:
              self._unique_id = decoded["_unique_id"]
        else:
          fail_count += 1

        time.sleep(delay_post)

      except Exception as e:
        self.step_error.emit(idx, label, str(e))
        fail_count += 1

    self.scenario_done.emit(ok_count, fail_count)


# --- MAIN WINDOW ------------------------------------------------------------

class HartMasterSim(QMainWindow):
  def __init__(self):
    super().__init__()
    self.setWindowTitle("HART Master Simulator  v1.0")
    self.setMinimumSize(1400, 750)
    self.resize(1400, 860)

    self._worker = HartWorker()
    self._worker.frame_logged.connect(self._on_frame_logged)
    self._worker.response_ready.connect(self._on_response)
    self._worker.error_occurred.connect(self._on_error)
    self._worker.start()

    self._connected = False
    self._unique_id = None  # 5-byte long address learned from Cmd0
    self._auto_poller = None
    self._scenario_runner = None
    self._sniffer_worker = None
    self._run_all_queue = []
    self._run_all_total = 0
    self._run_all_ok = 0
    self._run_all_fail = 0
    self._log_lines = 0
    self._max_log_lines = 2000
    self._session_log = []

    self._build_ui()
    self.setStyleSheet(STYLE)
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
    self._tabs.addTab(self._build_decode_tab(), "Decoded Response")
    self._tabs.addTab(self._build_custom_tab(), "Custom Frame")
    self._tabs.addTab(self._build_timing_tab(), "Timing / Stats")
    self._tabs.addTab(self._build_scenarios_tab(), "Scenarios")
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

    sep = QFrame()
    sep.setFrameShape(QFrame.HLine)
    sep.setFrameShadow(QFrame.Sunken)
    layout.addWidget(sep, 6, 0, 1, 3)

    self._btn_sniff = QPushButton("\u25b6 Start sniffer")
    self._btn_sniff.setToolTip(
      "Passively read bus traffic without sending frames.\n"
      "Requires port to be connected (open) first.\n"
      "Use a hardware tap if the port is owned by another process.")
    self._btn_sniff.clicked.connect(self._toggle_sniffer)
    layout.addWidget(self._btn_sniff, 7, 0, 1, 3)

    self._sniffing = False

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

    layout.addWidget(QLabel("Poll addr (0-15)"), 1, 0)
    self._spin_addr = QSpinBox()
    self._spin_addr.setRange(0, 15)
    layout.addWidget(self._spin_addr, 1, 1)

    layout.addWidget(QLabel("Unique ID"), 2, 0)
    self._le_unique_id = QLineEdit()
    self._le_unique_id.setPlaceholderText("e.g. E8 01 01 23 45")
    self._le_unique_id.setEnabled(False)
    self._le_unique_id.setToolTip(
      "5 bytes hex: ManufID DevType DevID[3]\n"
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
      (0,  "Cmd 0  - Read Unique Identifier"),
      (1,  "Cmd 1  - Read Primary Variable"),
      (2,  "Cmd 2  - Read Loop Current + %"),
      (3,  "Cmd 3  - Read Dynamic Variables"),
      (6,  "Cmd 6  - Write Polling Address"),
      (11, "Cmd 11 - Read Unique ID by Tag"),
      (12, "Cmd 12 - Read Message"),
      (13, "Cmd 13 - Read Tag/Descriptor/Date"),
      (14, "Cmd 14 - Read PV Info"),
      (15, "Cmd 15 - Read Output Info"),
      (16, "Cmd 16 - Read Final Assembly"),
      (48, "Cmd 48 - Read Additional Status"),
    ]

    for cmd_num, label in commands:
      btn = QPushButton(label)
      btn.setFixedHeight(22)
      btn.clicked.connect(lambda checked, c=cmd_num, l=label: self._send_command(c, b"", l))
      layout.addWidget(btn)

    return grp

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

  def _toggle_sniffer(self):
    if not self._sniffing:
      if not self._connected or not self._worker._port or not self._worker._port.is_open:
        self._log_info("Cannot start sniffer - connect to a port first")
        return
      self._sniffing = True
      self._btn_sniff.setText("\u25a0 Stop sniffer")
      self._sniffer_worker = SnifferWorker(self._worker._port)
      self._sniffer_worker.frame_sniffed.connect(self._on_sniffed_frame)
      self._sniffer_worker.start()
      self._log_info("== SNIFFER ON -- passive RX, waiting for bus traffic... ==",
                     color="#fbbf24")
    else:
      self._sniffing = False
      self._btn_sniff.setText("\u25b6 Start sniffer")
      if self._sniffer_worker:
        self._sniffer_worker.stop()
        self._sniffer_worker = None
      self._log_info("== SNIFFER OFF ==", color="#fbbf24")

  def _on_sniffed_frame(self, raw: bytes, info: str):
    hex_str = " ".join(f"{b:02X}" for b in raw)
    self._log_info(f"[SNIFF] {info}", color="#fbbf24")
    self._log_info(f"        {hex_str}", color="#94a3b8")

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
    if cmd == 0 and decoded and "_unique_id" in decoded:
      uid = decoded["_unique_id"]
      self._unique_id = uid
      uid_str = " ".join(f"{b:02X}" for b in uid)
      self._le_unique_id.setText(uid_str)
      self._lbl_unique_learned.setText(f"✓ Learned: {uid_str}")
      self._lbl_unique_learned.setStyleSheet("color: #4caf50;")

    # Append parsed info to log
    cs_status = "✓ CheskSum OK" if parsed.get("cs_ok") else "✗ CheskSum FAIL"
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
  def _log_frame(self, direction: str, label: str, data: bytes, latency_ms: float):
    if not data:
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
    self._append_log(f"  > {msg}", color)
    self._session_log.append(f"  > {msg}")

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
      elif "FAIL" in v or "ERROR" in v:
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

    # Step table
    self._scenario_table = QTableWidget(0, 4)
    self._scenario_table.setHorizontalHeaderLabels(["#", "Label", "Result", "Latency"])
    self._scenario_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
    self._scenario_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
    self._scenario_table.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeToContents)
    self._scenario_table.horizontalHeader().setSectionResizeMode(3, QHeaderView.ResizeToContents)
    self._scenario_table.verticalHeader().setVisible(False)
    self._scenario_table.setEditTriggers(QTableWidget.NoEditTriggers)
    self._scenario_table.setAlternatingRowColors(True)
    layout.addWidget(self._scenario_table)

    # Progress bar
    self._scenario_progress = QProgressBar()
    self._scenario_progress.setValue(0)
    self._scenario_progress.setTextVisible(True)
    self._scenario_progress.setStyleSheet(
      "QProgressBar { border: 1px solid #334155; background: #0f172a; color: #f1f5f9; height: 16px; }"
      "QProgressBar::chunk { background: #2563eb; }"
    )
    layout.addWidget(self._scenario_progress)

    # Summary label
    self._lbl_scenario_summary = QLabel("Ready")
    self._lbl_scenario_summary.setAlignment(Qt.AlignCenter)
    layout.addWidget(self._lbl_scenario_summary)

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

    layout.addLayout(btn_row)

    # Random order option + run-all progress label
    rand_row = QHBoxLayout()
    self._chk_random_order = QCheckBox("Random order (seed from time)")
    self._chk_random_order.setToolTip(
      "Shuffle scenario order using a time-based seed.\n"
      "Each Run All session uses a different shuffle.")
    rand_row.addWidget(self._chk_random_order)
    rand_row.addStretch()
    self._lbl_run_all_progress = QLabel("")
    self._lbl_run_all_progress.setObjectName("val")
    rand_row.addWidget(self._lbl_run_all_progress)
    layout.addLayout(rand_row)

    # Diagnostic hints box
    self._scenario_hints = QTextEdit()
    self._scenario_hints.setReadOnly(True)
    self._scenario_hints.setMaximumHeight(120)
    self._scenario_hints.setStyleSheet(
      "QTextEdit { background: #0f172a; color: #ffff00; font-size: 11px; border: 1px solid #334155; }")
    palette = self._scenario_hints.palette()
    palette.setColor(QPalette.PlaceholderText, QColor("#d0d064"))
    self._scenario_hints.setPalette(palette)
    self._scenario_hints.setPlaceholderText("Diagnostic hints will appear here as steps run…")
    layout.addWidget(self._scenario_hints)

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
      self._scenario_table.setItem(i, 2, QTableWidgetItem("—"))
      self._scenario_table.setItem(i, 3, QTableWidgetItem("—"))
    self._scenario_progress.setValue(0)
    self._scenario_progress.setMaximum(max(len(steps), 1))
    self._lbl_scenario_summary.setText(f"{len(steps)} steps - press RUN to start")
    self._scenario_hints.clear()

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

    # Reset table
    self._on_scenario_changed(self._cb_scenario.currentIndex())
    self._scenario_hints.clear()

    # For multi-drop sweep, patch address from UI spin
    patched = []
    base_addr = self._spin_addr.value()
    for step in steps:
      s = dict(step)
      # Address sweep scenario: use addr encoded in label "addr N" → handled by runner via poll_addr per step
      patched.append(s)

    self._scenario_runner = ScenarioRunner(
      port=self._worker._port,
      steps=patched,
      unique_id=self._unique_id,
      poll_addr=base_addr,
    )
    self._scenario_runner.step_started.connect(self._on_scenario_step_start)
    self._scenario_runner.step_done.connect(self._on_scenario_step_done)
    self._scenario_runner.step_error.connect(self._on_scenario_step_error)
    self._scenario_runner.scenario_done.connect(self._on_scenario_done)

    self._scenario_progress.setMaximum(len(steps))
    self._scenario_progress.setValue(0)
    self._btn_scenario_run.setEnabled(False)
    self._btn_scenario_run_all.setEnabled(False)
    self._btn_scenario_abort.setEnabled(True)
    self._lbl_scenario_summary.setText(f"Running: {name}")

    self._log_info(f"═══ SCENARIO START: {name} ═══")
    self._scenario_runner.start()

  def _run_all_scenarios(self):
    if not self._connected:
      self._log_info("Not connected - cannot run scenarios")
      return
    if not self._worker._port or not self._worker._port.is_open:
      self._log_info("Port not open")
      return

    import random
    names = list(SCENARIOS.keys())
    if self._chk_random_order.isChecked():
      seed = int(time.time() * 1000) & 0xFFFFFFFF
      rng = random.Random(seed)
      rng.shuffle(names)
      self._log_info(f"═══ RUN ALL: random order (seed={seed}) ═══")
      self._log_info("Order: " + " → ".join(f"#{list(SCENARIOS.keys()).index(n)+1}" for n in names))
    else:
      self._log_info("═══ RUN ALL: sequential order ═══")

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

    self._on_scenario_changed(self._cb_scenario.currentIndex())
    self._scenario_hints.clear()

    base_addr = self._spin_addr.value()
    self._scenario_runner = ScenarioRunner(
      port=self._worker._port,
      steps=list(steps),
      unique_id=self._unique_id,
      poll_addr=base_addr,
    )
    self._scenario_runner.step_started.connect(self._on_scenario_step_start)
    self._scenario_runner.step_done.connect(self._on_scenario_step_done)
    self._scenario_runner.step_error.connect(self._on_scenario_step_error)
    self._scenario_runner.scenario_done.connect(self._on_run_all_step_done)

    self._scenario_progress.setMaximum(len(steps))
    self._scenario_progress.setValue(0)
    # Lock buttons during run-all execution
    self._btn_scenario_run.setEnabled(False)
    self._btn_scenario_run_all.setEnabled(False)
    self._btn_scenario_abort.setEnabled(True)
    self._lbl_scenario_summary.setText(f"Running: {name}")
    self._log_info(f"═══ SCENARIO START [{current}/{self._run_all_total}]: {name} ═══")
    self._scenario_runner.start()

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

  def _on_scenario_step_done(self, idx: int, label: str,
                              tx: bytes, rx: bytes, latency_ms: float):
    self._scenario_progress.setValue(idx + 1)
    self._log_frame("TX", label, tx, 0.0)
    self._log_frame("RX", label, rx, latency_ms)

    parsed = parse_response(rx)
    ok = parsed is not None and parsed.get("ok", False)

    # Learn unique ID from scenario steps too
    if ok and parsed.get("cmd") == 0:
      decoded = decode_response(0, parsed.get("payload", b""))
      if decoded and "_unique_id" in decoded:
        uid = decoded["_unique_id"]
        self._unique_id = uid
        uid_str = " ".join(f"{b:02X}" for b in uid)
        self._le_unique_id.setText(uid_str)
        self._lbl_unique_learned.setText(f"✓ Learned: {uid_str}")
        self._lbl_unique_learned.setStyleSheet("color: #4caf50;")

    result_text = "✓ OK" if ok else ("✗ NO RESPONSE" if not rx else "✗ PARSE ERR")
    result_color = "#4ade80" if ok else "#f87171"

    result_item = self._scenario_table.item(idx, 2)
    if not result_item:
      result_item = QTableWidgetItem()
      self._scenario_table.setItem(idx, 2, result_item)
    result_item.setText(result_text)
    result_item.setForeground(QColor(result_color))

    lat_item = self._scenario_table.item(idx, 3)
    if not lat_item:
      lat_item = QTableWidgetItem()
      self._scenario_table.setItem(idx, 3, lat_item)
    lat_item.setText(f"{latency_ms:.0f} ms")
    lat_item.setForeground(QColor("#e2e8f0"))

    if not ok:
      hint = ""
      if not rx:
        hint = "No response - check: baud/parity (1200 8O1), preamble count, address, wiring."
      elif parsed and not parsed.get("cs_ok"):
        hint = "Response received but checksum failed - possible line noise or half-duplex echo."
      elif parsed and "error" in parsed:
        hint = f"Parse error: {parsed['error']}"
      if hint:
        self._scenario_hints.append(f"  ⚠ Step {idx+1}: {hint}\n")

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
    summary = f"Done: {ok_count}/{total} OK  {fail_count} failed"
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
        "• Slave only responds to primary master (0x02), rejects secondary (0x01)\n"
        "• Slave only handles short-addr frames, rejects long-addr\n"
        "• Cold-start: 'Configuration changed' bit set - send Cmd0 again\n"
        "• HART revision mismatch (slave HART5 vs master expects HART7)\n"
      )

  def closeEvent(self, event):
    self._stop_autopoll()
    if self._sniffer_worker:
      self._sniffer_worker.stop()
    self._worker.stop()
    self._worker.disconnect_port()
    event.accept()


# --- ENTRY POINT ------------------------------------------------------------

def main():
  app = QApplication(sys.argv)
  app.setApplicationName("HART Master Simulator")
  win = HartMasterSim()
  win.show()
  sys.exit(app.exec_())


if __name__ == "__main__":
  main()