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


# ─── HART PROTOCOL CONSTANTS ────────────────────────────────────────────────

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


# ─── HART FRAME ENGINE ──────────────────────────────────────────────────────

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


# ─── COMMAND DECODERS ───────────────────────────────────────────────────────

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


# ─── SERIAL WORKER THREAD ───────────────────────────────────────────────────

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


# ─── AUTO-POLL WORKER ───────────────────────────────────────────────────────

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


# ─── STYLE ──────────────────────────────────────────────────────────────────

STYLE = """
QMainWindow {
  background: #1a1c1e;
}
QWidget {
  background: #1a1c1e;
  color: #c8cdd3;
  font-family: 'Courier New', monospace;
  font-size: 12px;
}
QGroupBox {
  border: 1px solid #2e3338;
  border-radius: 4px;
  margin-top: 8px;
  padding-top: 4px;
  font-size: 11px;
  color: #6b7280;
  font-family: 'Courier New', monospace;
}
QGroupBox::title {
  subcontrol-origin: margin;
  left: 8px;
  padding: 0 4px;
}
QPushButton {
  background: #252830;
  border: 1px solid #3a3f47;
  border-radius: 3px;
  padding: 5px 12px;
  color: #c8cdd3;
  font-family: 'Courier New', monospace;
  font-size: 12px;
}
QPushButton:hover {
  background: #2e3338;
  border-color: #4a8fff;
  color: #e0e5ec;
}
QPushButton:pressed {
  background: #1e2228;
}
QPushButton:disabled {
  color: #444;
  border-color: #2a2e34;
}
QPushButton#btn_connect {
  background: #1a2e1a;
  border-color: #2a6e2a;
  color: #4caf50;
}
QPushButton#btn_connect:hover {
  background: #1e3a1e;
  border-color: #4caf50;
}
QPushButton#btn_disconnect {
  background: #2e1a1a;
  border-color: #6e2a2a;
  color: #f44336;
}
QPushButton#btn_disconnect:hover {
  background: #3a1e1e;
  border-color: #f44336;
}
QPushButton#btn_send {
  background: #1a2540;
  border-color: #2a4a8e;
  color: #4a8fff;
  font-size: 12px;
  padding: 6px 16px;
}
QPushButton#btn_send:hover {
  background: #1e2e50;
  border-color: #4a8fff;
  color: #80b0ff;
}
QPushButton#btn_poll_start {
  background: #1a2a20;
  border-color: #2a6a40;
  color: #4caf80;
}
QPushButton#btn_poll_stop {
  background: #2a1a1a;
  border-color: #6a2a2a;
  color: #f07060;
}
QComboBox {
  background: #252830;
  border: 1px solid #3a3f47;
  border-radius: 3px;
  padding: 4px 8px;
  color: #c8cdd3;
}
QComboBox::drop-down {
  border: none;
  width: 20px;
}
QComboBox QAbstractItemView {
  background: #252830;
  border: 1px solid #3a3f47;
  selection-background-color: #2e3d5a;
}
QSpinBox, QDoubleSpinBox, QLineEdit {
  background: #252830;
  border: 1px solid #3a3f47;
  border-radius: 3px;
  padding: 4px 8px;
  color: #c8cdd3;
}
QSpinBox:focus, QLineEdit:focus {
  border-color: #4a8fff;
}
QTextEdit {
  background: #0f1114;
  border: 1px solid #2e3338;
  border-radius: 3px;
  color: #c8cdd3;
  font-family: 'Courier New', monospace;
  font-size: 11px;
}
QTabWidget::pane {
  border: 1px solid #2e3338;
  background: #1a1c1e;
}
QTabBar::tab {
  background: #252830;
  border: 1px solid #2e3338;
  border-bottom: none;
  padding: 5px 14px;
  color: #6b7280;
  font-size: 11px;
  font-family: 'Courier New', monospace;
}
QTabBar::tab:selected {
  background: #1a1c1e;
  color: #c8cdd3;
  border-top: 2px solid #4a8fff;
}
QTabBar::tab:hover {
  color: #a0a8b4;
}
QLabel {
  color: #6b7280;
  font-size: 11px;
}
QLabel#val {
  color: #c8cdd3;
  font-size: 12px;
}
QLabel#bright {
  color: #4a8fff;
  font-size: 12px;
}
QLabel#ok {
  color: #4caf50;
}
QLabel#warn {
  color: #ff9800;
}
QLabel#err {
  color: #f44336;
}
QCheckBox {
  color: #c8cdd3;
}
QStatusBar {
  background: #0f1114;
  color: #6b7280;
  border-top: 1px solid #2e3338;
  font-size: 11px;
}
QSplitter::handle {
  background: #2e3338;
}
QScrollBar:vertical {
  background: #1a1c1e;
  width: 8px;
  border: none;
}
QScrollBar::handle:vertical {
  background: #3a3f47;
  border-radius: 4px;
  min-height: 20px;
}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
  height: 0;
}
QTableWidget {
  background: #0f1114;
  gridline-color: #2e3338;
  border: 1px solid #2e3338;
}
QTableWidget::item {
  padding: 3px 6px;
}
QHeaderView::section {
  background: #252830;
  border: 1px solid #2e3338;
  padding: 4px 6px;
  color: #6b7280;
  font-size: 11px;
}
"""

# Color tags for log text
LOG_TX_COLOR = "#4a8fff"
LOG_RX_OK_COLOR = "#4caf50"
LOG_RX_ERR_COLOR = "#f44336"
LOG_INFO_COLOR = "#ff9800"
LOG_DECODED_COLOR = "#b0d0ff"
LOG_MUTED = "#4a5060"


# ─── MAIN WINDOW ────────────────────────────────────────────────────────────

class HartMasterSim(QMainWindow):
  def __init__(self):
    super().__init__()
    self.setWindowTitle("HART Master Simulator  v1.0")
    self.setMinimumSize(1200, 750)
    self.resize(1400, 860)

    self._worker = HartWorker()
    self._worker.frame_logged.connect(self._on_frame_logged)
    self._worker.response_ready.connect(self._on_response)
    self._worker.error_occurred.connect(self._on_error)
    self._worker.start()

    self._connected = False
    self._unique_id = None  # 5-byte long address learned from Cmd0
    self._auto_poller = None
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

  # ── UI construction ────────────────────────────────────────────────────

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
      (0,  "Cmd 0  — Read Unique Identifier"),
      (1,  "Cmd 1  — Read Primary Variable"),
      (2,  "Cmd 2  — Read Loop Current + %"),
      (3,  "Cmd 3  — Read Dynamic Variables"),
      (6,  "Cmd 6  — Write Polling Address"),
      (11, "Cmd 11 — Read Unique ID by Tag"),
      (12, "Cmd 12 — Read Message"),
      (13, "Cmd 13 — Read Tag/Descriptor/Date"),
      (14, "Cmd 14 — Read PV Info"),
      (15, "Cmd 15 — Read Output Info"),
      (16, "Cmd 16 — Read Final Assembly"),
      (48, "Cmd 48 — Read Additional Status"),
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
    self._cb_poll_cmd.addItems(["0 — Identify", "1 — PV", "2 — Current+%", "3 — All vars"])
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
    self._chk_timestamps.setChecked(True)
    tb.addWidget(self._chk_timestamps)

    self._chk_annotate = QCheckBox("Annotate bytes")
    self._chk_annotate.setChecked(True)
    tb.addWidget(self._chk_annotate)

    tb.addStretch()
    layout.addLayout(tb)

    self._log = QTextEdit()
    self._log.setReadOnly(True)
    self._log.setFont(QFont("Courier New", 11))
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

    # Sniffer mode
    sep = QFrame()
    sep.setFrameShape(QFrame.HLine)
    sep.setFrameShadow(QFrame.Sunken)
    layout.addWidget(sep)

    layout.addWidget(QLabel("Passive sniffer mode — parse whatever arrives on the bus:"))
    self._btn_sniff = QPushButton("▶ Start sniffer")
    self._btn_sniff.clicked.connect(self._toggle_sniffer)
    layout.addWidget(self._btn_sniff)
    self._sniffing = False

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

  # ── Slots ──────────────────────────────────────────────────────────────

  def _refresh_ports(self):
    current = self._cb_port.currentText()
    ports = [p.device for p in serial.tools.list_ports.comports()]
    self._cb_port.clear()
    if ports:
      self._cb_port.addItems(ports)
      if current in ports:
        self._cb_port.setCurrentText(current)
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
      self._log_info("Not connected — cannot send")
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
      self._log_info("Invalid hex data — check your input")
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
    self._log_info("Injecting BAD CHECKSUM frame — testing slave error response")
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
    self._sniffing = not self._sniffing
    self._btn_sniff.setText("■ Stop sniffer" if self._sniffing else "▶ Start sniffer")
    self._log_info("Sniffer mode ON — parsing incoming bytes" if self._sniffing
                   else "Sniffer mode OFF")

  def _clear_log(self):
    self._log.clear()
    self._log_lines = 0
    self._session_log.clear()

  def _save_log(self):
    path, _ = QFileDialog.getSaveFileName(
      self, "Save HART log", f"hart_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt",
      "Text files (*.txt)"
    )
    if path:
      with open(path, "w") as f:
        f.write("\n".join(self._session_log))
      self._log_info(f"Log saved: {path}")

  # ── Signal handlers ────────────────────────────────────────────────────

  def _on_frame_logged(self, direction: str, label: str, data: bytes, latency_ms: float):
    self._log_frame(direction, label, data, latency_ms)

  def _on_response(self, parsed: dict):
    if "error" in parsed:
      self._log_info(f"⚠  {parsed.get('_label', '')} — {parsed['error']}", color=LOG_RX_ERR_COLOR)
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
    cs_status = "✓ CS OK" if parsed.get("cs_ok") else "✗ CS FAIL"
    cs_color = LOG_RX_OK_COLOR if parsed.get("cs_ok") else LOG_RX_ERR_COLOR

    lines = [
      (f"  └─ CMD={cmd}  st1=0x{parsed['st1']:02X}  st2=0x{parsed['st2']:02X}  "
       f"{cs_status}  latency={latency:.0f}ms", cs_color),
      (f"     Status: {parsed['status_text']}", LOG_DECODED_COLOR),
    ]
    if parsed["dev_flags"]:
      lines.append((f"     Device: {', '.join(parsed['dev_flags'])}", LOG_INFO_COLOR))

    for text, color in lines:
      self._append_log(text, color)

    # Fill decode table
    self._populate_decode_table(parsed)

  def _on_error(self, msg: str):
    self._log_info(f"ERROR: {msg}", color=LOG_RX_ERR_COLOR)

  # ── Log helpers ────────────────────────────────────────────────────────

  def _log_frame(self, direction: str, label: str, data: bytes, latency_ms: float):
    if not data:
      self._append_log(f"  [{direction}] {label} — (no data)", LOG_RX_ERR_COLOR)
      return

    color = LOG_TX_COLOR if direction == "TX" else LOG_RX_OK_COLOR
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3] if self._chk_timestamps.isChecked() else ""
    ts_part = f"[{ts}] " if ts else ""

    # Optionally hide preamble bytes
    display_data = data
    if not self._chk_show_preamble.isChecked() and direction == "TX":
      display_data = bytes(b for b in data if b != 0xFF) if all(b == 0xFF for b in data[:5]) else data
      preamble_stripped = data[:5] if data[:5] == b"\xff\xff\xff\xff\xff" else b""
      if preamble_stripped:
        display_data = data[5:]

    hex_str = " ".join(f"{b:02X}" for b in data)
    dir_arrow = "──▶" if direction == "TX" else "◀──"

    header = f"{ts_part}{dir_arrow} {direction}  {label}  ({len(data)}B)"
    self._append_log(header, color)

    if self._chk_annotate.isChecked():
      annotation = self._annotate_frame(data, direction)
      self._append_log(f"  {hex_str}", color)
      self._append_log(f"  {annotation}", LOG_MUTED)
    else:
      self._append_log(f"  {hex_str}", color)

    self._session_log.append(f"{header}\n  {hex_str}")

  def _annotate_frame(self, data: bytes, direction: str) -> str:
    """Produce a byte-by-byte annotation string."""
    parts = []
    i = 0
    # Preambles
    while i < len(data) and data[i] == 0xFF:
      parts.append("PRE")
      i += 1

    if i >= len(data):
      return " ".join(parts)

    delimiter = data[i]
    is_long = (delimiter & 0x80) != 0
    parts.append(f"DLM({delimiter:02X})")
    i += 1

    if is_long:
      for j in range(5):
        if i < len(data):
          parts.append(f"A{j}({data[i]:02X})")
          i += 1
    else:
      if i < len(data):
        parts.append(f"ADR({data[i]:02X})")
        i += 1

    if i < len(data):
      parts.append(f"CMD({data[i]:02X})")
      i += 1
    if i < len(data):
      parts.append(f"CNT({data[i]:02X})")
      cnt = data[i]
      i += 1
    else:
      cnt = 0

    # For slave responses: first 2 data bytes are status
    if direction == "RX" and cnt >= 2:
      if i < len(data):
        parts.append(f"ST1({data[i]:02X})")
        i += 1
      if i < len(data):
        parts.append(f"ST2({data[i]:02X})")
        i += 1
      cnt -= 2

    for j in range(cnt):
      if i < len(data):
        parts.append(f"D{j}({data[i]:02X})")
        i += 1

    if i < len(data):
      parts.append(f"CS({data[i]:02X})")

    return " ".join(parts)

  def _log_info(self, msg: str, color: str = LOG_INFO_COLOR):
    self._append_log(f"  ⟫ {msg}", color)
    self._session_log.append(f"  ⟫ {msg}")

  def _append_log(self, text: str, color: str = "#c8cdd3"):
    cursor = self._log.textCursor()
    cursor.movePosition(QTextCursor.End)
    fmt = cursor.charFormat()
    fmt.setForeground(QColor(color))
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
    rows.append(("Status byte 1 (comm)", f"0x{parsed['st1']:02X} — {parsed['status_text']}"))
    rows.append(("Status byte 2 (device)", f"0x{parsed['st2']:02X}"))

    dev_flags = parsed.get("dev_flags", [])
    rows.append(("Device flags", ", ".join(dev_flags) if dev_flags else "(none)"))

    cs_label = "✓ OK" if parsed.get("cs_ok") else f"✗ FAIL (got 0x{parsed.get('checksum_byte', 0):02X}, expected 0x{parsed.get('expected_cs', 0):02X})"
    rows.append(("Checksum", cs_label))
    rows.append(("Preambles received", str(parsed.get("preamble_count", "?"))))
    rows.append(("Latency", f"{parsed.get('_latency_ms', 0):.1f} ms"))
    rows.append(("Raw payload (hex)", parsed.get("payload", b"").hex(" ")))

    decoded = parsed.get("_decoded")
    if decoded:
      rows.append(("", ""))  # separator
      rows.append(("── DECODED FIELDS ──", ""))
      for k, v in decoded.items():
        if not k.startswith("_"):
          rows.append((k, str(v)))

    self._decode_table.setRowCount(len(rows))
    for i, (k, v) in enumerate(rows):
      item_k = QTableWidgetItem(k)
      item_v = QTableWidgetItem(v)
      item_k.setForeground(QColor("#6b7280"))
      if k.startswith("──") or k == "":
        item_k.setForeground(QColor("#4a8fff"))
        item_v.setForeground(QColor("#4a8fff"))
      elif "FAIL" in v or "ERROR" in v:
        item_v.setForeground(QColor("#f44336"))
      elif v.startswith("✓"):
        item_v.setForeground(QColor("#4caf50"))
      else:
        item_v.setForeground(QColor("#c8cdd3"))
      self._decode_table.setItem(i, 0, item_k)
      self._decode_table.setItem(i, 1, item_v)

    self._tabs.setCurrentIndex(1)  # Switch to decoded tab

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

  def closeEvent(self, event):
    self._stop_autopoll()
    self._worker.stop()
    self._worker.disconnect_port()
    event.accept()


# ─── ENTRY POINT ────────────────────────────────────────────────────────────

def main():
  app = QApplication(sys.argv)
  app.setApplicationName("HART Master Simulator")
  win = HartMasterSim()
  win.show()
  sys.exit(app.exec_())


if __name__ == "__main__":
  main()