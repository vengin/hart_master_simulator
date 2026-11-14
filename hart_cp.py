"""HART 7 Common Practice commands: request encoders and response decoders.

Follows the Common Practice Command Specification (sections 7.1-7.72) as
provided in buos_src/hart/hart_spec/common practise commands. Pure functions,
no GUI: used by the "Common Practice" tab, the scenarios and the tests.
"""
import struct

NAN_BYTES = b"\x7f\xa0\x00\x00"   # "not applicable / not used" float

# Common Table 3 (Transfer Function Codes), subset
TRANSFER_FUNCTIONS = {0: "Linear", 1: "Square Root", 2: "Square Root Third Power",
                      3: "Square Root Fifth Power", 4: "Special Curve", 5: "Square",
                      250: "Not Used", 251: "None"}
# Common Table 9 (Burst Mode Control Codes)
BURST_CONTROL = {0: "Off", 1: "Enable on Token-Passing Data Link", 2: "Enable on TDMA",
                 3: "Enable on TDMA and Token-Passing", 4: "Enable on HART-IP"}

# (cmd, label, input kind, default input text, tooltip)
#   none   - no request data
#   u8     - one byte (decimal)
#   u8list - 1..n bytes (decimal, space separated)
#   f32    - one float
#   u24    - Unsigned-24 (decimal)
#   u16    - Unsigned-16 (decimal)
#   range  - "<units> <upper> <lower>"
#   c108   - "<command number> [<burst message>]"
#   c109   - "<control code> [<burst message>]"
CP_COMMANDS = [
  (33, "Cmd33 - Read Device Variables", "u8list", "0 1 2 3", "1-4 Device Variable codes"),
  (34, "Cmd34 - Write PV Damping", "f32", "0.0", "seconds"),
  (35, "Cmd35 - Write PV Range Values", "range", "57 120.0 0.0", "units upper lower"),
  (36, "Cmd36 - Set PV Upper Range Value", "none", "", ""),
  (37, "Cmd37 - Set PV Lower Range Value", "none", "", ""),
  (38, "Cmd38 - Reset Configuration Changed", "u16", "", "Config Change Counter (blank = HART 6 form)"),
  (40, "Cmd40 - Enter/Exit Fixed Current", "f32", "0.0", "mA (0 = exit)"),
  (41, "Cmd41 - Perform Self-Test", "none", "", ""),
  (43, "Cmd43 - Set PV Zero", "none", "", ""),
  (44, "Cmd44 - Write PV Units", "u8", "57", "Units Code (Common Table 2)"),
  (45, "Cmd45 - Trim Loop Current Zero", "f32", "4.0", "measured mA"),
  (46, "Cmd46 - Trim Loop Current Gain", "f32", "20.0", "measured mA"),
  (47, "Cmd47 - Write PV Transfer Function", "u8", "0", "Common Table 3"),
  (48, "Cmd48 - Read Additional Device Status", "none", "", ""),
  (49, "Cmd49 - Write PV Transducer Serial No", "u24", "0", "Unsigned-24"),
  (50, "Cmd50 - Read Dynamic Variable Assignments", "none", "", ""),
  (51, "Cmd51 - Write Dynamic Variable Assignments", "u8list", "0 1 2 3", "PV SV TV QV"),
  (54, "Cmd54 - Read Device Variable Information", "u8", "0", "Device Variable code"),
  (59, "Cmd59 - Write Number of Response Preambles", "u8", "5", "5-20"),
  (108, "Cmd108 - Write Burst Mode Command Number", "c108", "1 0", "command [burst message]"),
  (109, "Cmd109 - Burst Mode Control", "c109", "0 0", "code [burst message]"),
]
CP_BY_CMD = {c[0]: c for c in CP_COMMANDS}


def _ints(text):
  return [int(t, 0) for t in text.replace(",", " ").split()]


def encode_cp_request(cmd: int, text: str) -> bytes:
  """Request data bytes for a Common Practice command from the GUI text.
  Raises ValueError on bad input."""
  kind = CP_BY_CMD[cmd][2]
  text = (text or "").strip()
  if kind == "none":
    return b""
  if kind == "u8":
    (v,) = _ints(text)
    return bytes([v])
  if kind == "u8list":
    vals = _ints(text)
    if not vals:
      raise ValueError("need at least one code")
    return bytes(vals)
  if kind == "f32":
    return struct.pack(">f", float(text))
  if kind == "u24":
    v = int(text, 0)
    if not 0 <= v <= 0xFFFFFF:
      raise ValueError("Unsigned-24 out of range")
    return v.to_bytes(3, "big")
  if kind == "u16":
    if not text:
      return b""
    return int(text, 0).to_bytes(2, "big")
  if kind == "range":
    parts = text.split()
    if len(parts) != 3:
      raise ValueError("need: units upper lower")
    return bytes([int(parts[0], 0)]) + struct.pack(">ff", float(parts[1]), float(parts[2]))
  if kind == "c108":
    vals = _ints(text)
    if len(vals) == 1:
      return bytes([vals[0]])                      # HART 5/6 form
    return vals[0].to_bytes(2, "big") + bytes([vals[1]])
  if kind == "c109":
    return bytes(_ints(text))
  raise ValueError(kind)


# ----------------------------------------------------------------- decoders
def fmt_float(b: bytes) -> str:
  if b == NAN_BYTES:
    return "NaN (not applicable)"
  return f"{struct.unpack('>f', b)[0]:.4f}"


def _unit(code, units):
  return f"{code} ({units.get(code, 'unknown')})"


def _need(payload, n):
  if len(payload) < n:
    return {"error": f"Too short: {len(payload)} bytes (need {n})"}
  return None


def decode_cmd33(payload, units):
  if not payload or len(payload) % 6:
    return {"error": f"Length {len(payload)} is not a multiple of 6"}
  out = {}
  for i in range(0, len(payload), 6):
    s = payload[i:i + 6]
    out[f"Slot {i // 6}"] = f"DV {s[0]}: {fmt_float(s[2:6])} {_unit(s[1], units)}"
  return out


def decode_float_echo(name):
  def dec(payload, units):
    return _need(payload, 4) or {name: fmt_float(payload[:4])}
  return dec


def decode_cmd35(payload, units):
  return _need(payload, 9) or {
    "Range units": _unit(payload[0], units),
    "Upper Range Value": fmt_float(payload[1:5]),
    "Lower Range Value": fmt_float(payload[5:9]),
  }


def decode_cmd44(payload, units):
  return _need(payload, 1) or {"PV units": _unit(payload[0], units)}


def decode_cmd47(payload, units):
  return _need(payload, 1) or {
    "Transfer function": f"{payload[0]} ({TRANSFER_FUNCTIONS.get(payload[0], 'unknown')})"}


def decode_cmd49(payload, units):
  return _need(payload, 3) or {"PV transducer S/N": str(int.from_bytes(payload[:3], "big"))}


def decode_cmd50(payload, units):
  return _need(payload, 4) or {
    name: str(payload[i]) for i, name in enumerate(["PV", "SV", "TV", "QV"])}


def decode_cmd54(payload, units):
  err = _need(payload, 23)
  if err:
    return err
  out = {
    "Device Variable Code": str(payload[0]),
    "Transducer Serial Number": str(int.from_bytes(payload[1:4], "big")),
    "Limits/Min Span Units": _unit(payload[4], units),
    "Upper Transducer Limit": fmt_float(payload[5:9]),
    "Lower Transducer Limit": fmt_float(payload[9:13]),
    "Damping Value": fmt_float(payload[13:17]) + " s",
    "Minimum Span": fmt_float(payload[17:21]),
    "Classification": str(payload[21]),
    "Family": str(payload[22]),
  }
  if len(payload) >= 28:
    acq = int.from_bytes(payload[23:27], "big")
    out["Acquisition Period"] = ("not calculated (0xFFFFFFFF)" if acq == 0xFFFFFFFF
                                 else f"{acq / 32.0:.3f} ms")
    out["Properties"] = f"0x{payload[27]:02X}"
  else:
    out["warning"] = "HART 7 response is 28 bytes (Acquisition Period, Properties missing)"
  return out


def decode_cmd108(payload, units):
  if len(payload) == 1:
    return {"Burst command (HART 5/6 form)": str(payload[0])}
  return _need(payload, 2) or {
    "Burst command": str(int.from_bytes(payload[:2], "big")),
    **({"Burst message": str(payload[2])} if len(payload) > 2 else {})}


def decode_cmd109(payload, units):
  err = _need(payload, 1)
  if err:
    return err
  out = {"Burst mode": f"{payload[0]} ({BURST_CONTROL.get(payload[0], 'unknown')})"}
  if len(payload) > 1:
    out["Burst message"] = str(payload[1])
  return out


CP_DECODERS = {
  33: decode_cmd33,
  34: decode_float_echo("PV damping (s)"),
  35: decode_cmd35,
  40: decode_float_echo("Actual fixed current (mA)"),
  44: decode_cmd44,
  45: decode_float_echo("Actual measured current (mA)"),
  46: decode_float_echo("Actual measured current (mA)"),
  47: decode_cmd47,
  49: decode_cmd49,
  50: decode_cmd50,
  51: decode_cmd50,     # response = the assignments actually used
  54: decode_cmd54,
  108: decode_cmd108,
  109: decode_cmd109,
}
