"""Common Practice request encoders / response decoders (hart_cp.py)."""
import struct

import pytest

import hart_cp as cp
from hart_master_simulator import UNIT_CODES, decode_response


@pytest.mark.parametrize("cmd, text, data", [
  (33, "0 1 2 3", bytes([0, 1, 2, 3])),
  (33, "246", bytes([246])),
  (34, "1.5", struct.pack(">f", 1.5)),
  (35, "57 120 0", bytes([57]) + struct.pack(">ff", 120.0, 0.0)),
  (36, "", b""),
  (38, "", b""),
  (38, "300", bytes([1, 44])),
  (40, "4", struct.pack(">f", 4.0)),
  (44, "57", bytes([57])),
  (49, "1193046", bytes([0x12, 0x34, 0x56])),
  (49, "0x123456", bytes([0x12, 0x34, 0x56])),
  (51, "0 1 2 3", bytes([0, 1, 2, 3])),
  (54, "0", bytes([0])),
  (59, "8", bytes([8])),
  (108, "1", bytes([1])),                    # HART 5/6 form
  (108, "1 0", bytes([0, 1, 0])),            # HART 7: Unsigned-16 + burst message
  (109, "0", bytes([0])),
  (109, "1 0", bytes([1, 0])),
])
def test_encode(cmd, text, data):
  assert cp.encode_cp_request(cmd, text) == data


@pytest.mark.parametrize("cmd, text", [(33, ""), (35, "57 1"), (49, "16777216"), (44, "x"), (34, "")])
def test_encode_rejects_bad_input(cmd, text):
  with pytest.raises((ValueError, OverflowError)):
    cp.encode_cp_request(cmd, text)


def test_every_cp_default_encodes():
  for cmd, label, kind, default, tip in cp.CP_COMMANDS:
    cp.encode_cp_request(cmd, default)


def test_decode_cmd33_and_nan():
  p = bytes([0, 57]) + struct.pack(">f", 50.5) + bytes([100, 250]) + cp.NAN_BYTES
  d = decode_response(33, p)
  assert d["Slot 0"] == "DV 0: 50.5000 57 (%)"
  assert "NaN" in d["Slot 1"]
  assert "error" in decode_response(33, p[:7])


def test_decode_cmd54_hart7_and_short():
  p = bytes([0, 0x12, 0x34, 0x56, 57]) + cp.NAN_BYTES * 2 + struct.pack(">f", 0) + cp.NAN_BYTES + \
      bytes([0, 250]) + (7).to_bytes(4, "big") + b"\x00"
  d = decode_response(54, p)
  assert d["Transducer Serial Number"] == str(0x123456)
  assert d["Upper Transducer Limit"].startswith("NaN")
  assert d["Acquisition Period"] == "0.219 ms"
  assert "warning" in decode_response(54, p[:23])
  p2 = p[:23] + (0xFFFFFFFF).to_bytes(4, "big") + b"\x00"
  assert "not calculated" in decode_response(54, p2)["Acquisition Period"]


def test_decode_simple():
  assert decode_response(50, bytes([0, 1, 2, 3]))["QV"] == "3"
  assert decode_response(49, bytes([0, 0, 9]))["PV transducer S/N"] == "9"
  assert decode_response(44, bytes([57]))["PV units"] == "57 (%)"
  assert decode_response(108, bytes([0, 9, 0]))["Burst command"] == "9"
  assert decode_response(109, bytes([1]))["Burst mode"].startswith("1 ")
  assert decode_response(35, bytes([57]) + struct.pack(">ff", 120, 0))["Upper Range Value"] == "120.0000"
