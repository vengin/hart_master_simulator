"""In-memory HART 7 field device used to test the simulator offline.

FakeSlave models the Ectron BUOS firmware behaviour that the scenarios check
(HART 7.3.3 per-master status bits, NV persistence, broadcast rules, C6, C11,
C12-C22, C38, C42). It also acts as the serial port given to ScenarioRunner.

`faults` injects known defects so tests can prove a scenario detects them:
  no_persistence        configuration is lost on reset / power cycle
  reset_drops_last_save C42 discards a change acknowledged just before it
  shared_config_flag    one Config Changed bit for both masters
  c38_clears_both       C38 clears both masters' Config Changed bits
  shared_cold_start     one Cold Start bit, cleared by the first response
  no_cold_start         Cold Start never reported
  wrong_master_bit      response master bit not echoed (always primary)
  broadcast_any_cmd     answers every command on the Broadcast Address
  c38_no_counter_check  C38 ignores the counter (never RC 9)
  counter_on_rc5        a rejected (RC 5) write still increments the counter
  no_c6_persistence     polling address not persisted
  missing_cmd48         Cmd48 not implemented (RC 64)
  wp_not_enforced       reports write protect but still accepts writes
  cmd9_no_timestamp_pad Cmd9 slots one byte short (bad layout)

`write_protect=True` makes the device write protected (Cmd15 byte 15 = 1,
writes return RC 7).

Process data commands (Cmd1/2/3/8/9/14/15/48) return fixed, layout-correct
values so the Universal Command Conformance scenario can run offline.
"""
import copy
import struct

PRIMARY = 0x80
SECONDARY = 0x40
ST_CONFIG_CHANGED = 0x40
ST_COLD_START = 0x20

EXP_DEV_TYPE = 0xE78B
DEVICE_ID = bytes([0x01, 0x23, 0x45])
UNIQUE_ADDR = bytes([(EXP_DEV_TYPE >> 8) & 0x3F, EXP_DEV_TYPE & 0xFF]) + DEVICE_ID

PV_UNITS = 38            # Hz (Common Table 2)
PV_VALUE = 1234.5
LOOP_MA = 12.0
PERCENT = 50.0


def _f(v):
  return struct.pack(">f", v)


def _xor(data):
  cs = 0
  for b in data:
    cs ^= b
  return cs


class FakeSlave:
  def __init__(self, faults=(), poll_addr=0, echo=False, write_protect=False):
    """echo: emulate a half-duplex modem that echoes the request on RX
    (True = exact copy, "short" = copy with only 2 preambles)."""
    self.faults = set(faults)
    self.echo = echo
    self.write_protect = write_protect
    self.default_poll = poll_addr
    self.nv = self._defaults()          # what is in EEPROM
    self.ram = copy.deepcopy(self.nv)
    self.cold = PRIMARY | SECONDARY
    self.pending_save = False
    self.rx_buf = b""
    self.log = []                       # (request cmd, responded) for debugging
    self.resets = 0

  # ---------------------------------------------------------------- config
  def _defaults(self):
    return {
      "poll": self.default_poll if hasattr(self, "default_poll") else 0,
      "loop": 1,
      "ccc": 0,
      "ccflag": 0,
      "tag": bytes([0x08, 0x55, 0x53, 0x7C, 0xD3, 0x71]),   # packed "BUOS_481"
      "desc": bytes(range(1, 13)),
      "date": bytes([9, 11, 126]),
      "msg": bytes(range(40, 64)),
      "fan": bytes([0, 0, 7]),
      "long_tag": b"Ectron Back Up Overspeed Monitor",
    }

  def _save(self):
    self.pending_save = True
    self._commit()

  def _commit(self):
    if "no_persistence" in self.faults:
      return
    keep_poll = self.nv["poll"], self.nv["loop"]
    self.nv = copy.deepcopy(self.ram)
    if "no_c6_persistence" in self.faults:
      self.nv["poll"], self.nv["loop"] = keep_poll
    self.pending_save = False

  def _config_change(self):
    self.ram["ccflag"] = PRIMARY | SECONDARY
    self.ram["ccc"] = (self.ram["ccc"] + 1) & 0xFFFF
    self._save()

  def reset(self, power_cycle=False):
    """Device Reset / power-up: RAM reloaded from NV, Cold Start for both."""
    self.resets += 1
    if "no_persistence" in self.faults:
      self.nv = self._defaults()
    self.ram = copy.deepcopy(self.nv)
    self.cold = PRIMARY | SECONDARY
    self.rx_buf = b""

  # ------------------------------------------------------------ serial API
  @property
  def in_waiting(self):
    return len(self.rx_buf)

  def read(self, n):
    data, self.rx_buf = self.rx_buf[:n], self.rx_buf[n:]
    return data

  def reset_input_buffer(self):
    self.rx_buf = b""

  def flush(self):
    pass

  def write(self, frame):
    frame = bytes(frame)
    if self.echo:
      self.rx_buf += frame if self.echo is True else b"\xff\xff" + frame.lstrip(b"\xff")
    resp = self._handle(frame)
    if resp:
      self.rx_buf += resp
    return len(frame)

  # -------------------------------------------------------------- protocol
  def _handle(self, frame):
    i = 0
    while i < len(frame) and frame[i] == 0xFF:
      i += 1
    if i < 2 or i >= len(frame):
      return None
    delim = frame[i]
    if (delim & 0x07) != 0x02:               # only STX requests
      return None
    is_long = bool(delim & 0x80)
    alen = 5 if is_long else 1
    addr = frame[i + 1:i + 1 + alen]
    cmd = frame[i + 1 + alen]
    n = frame[i + 2 + alen]
    data = frame[i + 3 + alen:i + 3 + alen + n]
    cs = frame[i + 3 + alen + n] if i + 3 + alen + n < len(frame) else None
    if cs != _xor(frame[i:i + 3 + alen + n]):
      return None                            # bad checksum: silently discard
    master = PRIMARY if addr[0] & 0x80 else SECONDARY

    broadcast = False
    if is_long:
      a = bytes([addr[0] & 0x3F]) + addr[1:]
      if a == bytes(5):
        broadcast = True
        if cmd not in (11, 21) and "broadcast_any_cmd" not in self.faults:
          return None
      elif a != UNIQUE_ADDR:
        return None
    elif (addr[0] & 0x3F) != self.ram["poll"]:
      return None

    if "wrong_master_bit" in self.faults:
      mbit = 0x80
    else:
      mbit = 0x80 if master == PRIMARY else 0x00
    if is_long:
      raddr = bytes([mbit]) + bytes(4) if broadcast else bytes([mbit | UNIQUE_ADDR[0]]) + UNIQUE_ADDR[1:]
    else:
      raddr = bytes([mbit | self.ram["poll"]])   # address of the request

    status_after = cmd in (11, 21, 38)
    status = None if status_after else self._status(master)
    body = self._command(cmd, data, master)
    self.log.append((cmd, body is not None))
    if body is None:
      return None
    rc, payload = body
    if status_after:
      status = self._status(master)
    out = bytes([0x86 if is_long else 0x06]) + raddr + bytes([cmd, len(payload) + 2, rc, status]) + payload
    resp = bytes([0xFF] * 5) + out + bytes([_xor(out)])
    if cmd == 42 and rc == 0:
      self._after_c42()
    return resp

  def _after_c42(self):
    if "reset_drops_last_save" in self.faults and hasattr(self, "_last_nv_before_save"):
      self.nv = self._last_nv_before_save
    self.reset()

  def _status(self, master):
    st = 0
    if "shared_cold_start" in self.faults:
      if self.cold:
        st |= ST_COLD_START
      self.cold = 0
    elif "no_cold_start" not in self.faults:
      if self.cold & master:
        st |= ST_COLD_START
      self.cold &= ~master
    flag = self.ram["ccflag"]
    if "shared_config_flag" in self.faults:
      if flag:
        st |= ST_CONFIG_CHANGED
    elif flag & master:
      st |= ST_CONFIG_CHANGED
    if self.ram["loop"] == 0:
      st |= 0x08
    return st

  def _c0_data(self):
    r = self.ram
    return bytes([254, EXP_DEV_TYPE >> 8, EXP_DEV_TYPE & 0xFF, 5, 7, 7, 24, (5 << 3), 0]) + DEVICE_ID + \
      bytes([5, 0, r["ccc"] >> 8, r["ccc"] & 0xFF, 0, 0x61, 0xF6, 0x61, 0xF6, 1])

  def _write(self, data, need, apply):
    if self.write_protect and "wp_not_enforced" not in self.faults:
      return 7, b""
    if len(data) < need:
      if "counter_on_rc5" in self.faults:
        self._config_change()
      return 5, b""
    self._last_nv_before_save = copy.deepcopy(self.nv)
    apply(data[:need])
    self._config_change()
    return 0, bytes(data[:need])

  def _command(self, cmd, data, master):
    r = self.ram
    if cmd == 0:
      return 0, self._c0_data()
    if cmd == 1:
      return 0, bytes([PV_UNITS]) + _f(PV_VALUE)
    if cmd == 2:
      return 0, _f(LOOP_MA) + _f(PERCENT)
    if cmd == 3:
      return 0, _f(LOOP_MA) + bytes([PV_UNITS]) + _f(PV_VALUE)
    if cmd == 8:
      return 0, bytes([0, 250, 250, 250])
    if cmd == 9:
      if not 1 <= len(data) <= 8:
        return 5, b""
      slot_len = 7 if "cmd9_no_timestamp_pad" in self.faults else 8
      out = bytes([0])
      for code in data:
        slot = bytes([code, 0, PV_UNITS]) + _f(PV_VALUE) + bytes([0xC0])
        out += slot[:slot_len]
      # Like the BUOS firmware: Device Variable codes are answered with the
      # Dynamic Variables -> RC 14 warning (data still complete)
      rc = 0 if all(c >= 246 for c in data) else 14
      return rc, out + bytes(4)                # + timestamp
    if cmd == 14:
      return 0, bytes([0, 0, 0, PV_UNITS]) + _f(30000.0) + _f(0.0) + _f(1.0)
    if cmd == 15:
      wp = 1 if self.write_protect else 251    # 251 = None, as the BUOS firmware reports
      return 0, bytes([0, 0, PV_UNITS]) + _f(30000.0) + _f(0.0) + _f(0.0) + bytes([wp, 250, 0])
    if cmd == 48:
      if "missing_cmd48" in self.faults:
        return 64, b""
      return 0, bytes(9)
    if cmd == 6:
      if self.write_protect and "wp_not_enforced" not in self.faults:
        return 7, b""
      if len(data) < 1:
        return 5, b""
      mode = data[1] if len(data) >= 2 else (1 if data[0] == 0 else 0)
      if data[0] > 63:
        return 2, b""
      if mode > 1:
        return 12, b""
      self._last_nv_before_save = copy.deepcopy(self.nv)
      r["poll"], r["loop"] = data[0], mode
      self._config_change()
      return 0, bytes([data[0], mode])
    if cmd == 7:
      return 0, bytes([r["poll"], r["loop"]])
    if cmd in (11, 21):
      want = r["tag"] if cmd == 11 else r["long_tag"]
      if len(data) < len(want) or bytes(data[:len(want)]) != want:
        return None
      return 0, self._c0_data()
    if cmd == 12:
      return 0, r["msg"]
    if cmd == 13:
      return 0, r["tag"] + r["desc"] + r["date"]
    if cmd == 16:
      return 0, r["fan"]
    if cmd == 17:
      return self._write(data, 24, lambda d: r.__setitem__("msg", bytes(d)))
    if cmd == 18:
      def apply18(d):
        r["tag"], r["desc"], r["date"] = bytes(d[:6]), bytes(d[6:18]), bytes(d[18:21])
      return self._write(data, 21, apply18)
    if cmd == 19:
      return self._write(data, 3, lambda d: r.__setitem__("fan", bytes(d)))
    if cmd == 20:
      return 0, r["long_tag"]
    if cmd == 22:
      return self._write(data, 32, lambda d: r.__setitem__("long_tag", bytes(d)))
    if cmd == 38:
      ccc = bytes([r["ccc"] >> 8, r["ccc"] & 0xFF])
      if len(data) >= 2 and "c38_no_counter_check" not in self.faults:
        if ((data[0] << 8) | data[1]) != r["ccc"]:
          return 9, ccc
      if r["ccflag"] & (PRIMARY if master == PRIMARY else SECONDARY):
        if "c38_clears_both" in self.faults:
          r["ccflag"] = 0
        else:
          r["ccflag"] &= ~master
        self._save()
      return 0, ccc
    if cmd == 42:
      return 0, b""
    return 64, b""
