# Each scenario is a list of steps executed sequentially by ScenarioRunner.
# Step dict keys:
#   label      str   – shown in log
#   cmd        int   – HART command number
#   data       bytes – command data payload
#   use_long   bool  – use long (unique-ID) address frame
#   preambles  int|None – override preamble count for this step
#   master     str|None – "primary" | "secondary" (address byte bit 7)
#   delay_pre  float – sleep BEFORE sending (seconds)
#   delay_post float – sleep AFTER response received (seconds)
#   note       str   – diagnostic hint shown in log before step
#
# Optional checks (see ScenarioRunner / evaluate_step in hart_master_simulator.py):
#   expect_rc, expect_status_set, expect_status_clear, expect_payload,
#   expect_payload_one_of, payload_slice, expect_u16, save_as, data_from,
#   broadcast, retries, retry_delay, manual. A scenario with "manual_only"
#   needs an operator and is skipped by RUN ALL.

from hart_codec import encode_packed_ascii, encode_date, encode_latin1

# Device Status bits (Field Device Status byte)
ST_CONFIG_CHANGED = 0x40
ST_COLD_START     = 0x20

# C0 response data: bytes 14-15 = Configuration Change Counter
CCC_OFFSET = 14

# Test values written by the configuration / NV scenarios (originals are read
# first and restored at the end of each scenario)
NV_TAG       = encode_packed_ascii("NVTEST01", 6)
NV_DESC      = encode_packed_ascii("NV PERSIST TEST", 12)
NV_DATE      = encode_date(13, 11, 2026)
NV_C18       = NV_TAG + NV_DESC + NV_DATE
NV_MSG       = encode_packed_ascii("HART NV PERSISTENCE TEST MESSAGE", 24)
NV_FAN       = bytes([0x12, 0x34, 0x56])
NV_LONG_TAG  = encode_latin1("Ectron NV persistence test tag", 32)
TORN_C18     = encode_packed_ascii("TORNTEST", 6) + encode_packed_ascii("RESET IN SAVE", 12) + encode_date(14, 11, 2026)
WRONG_TAG    = encode_packed_ascii("NOSUCHTG", 6)

# Time for the device to store a change in EEPROM (HART NV writer: <= 1 byte per
# main-loop pass, up to 5 ms each) and to boot after a Device Reset.
NV_SAVE_WAIT_S = 1.5
RESET_BOOT_WAIT_S = 3.0


def _s(label, cmd, data=b"", **kw):
  """Step with HART 7 defaults: long frame, primary master, 5 preambles."""
  step = {"label": label, "cmd": cmd, "data": data, "use_long": True,
          "preambles": 5, "master": "primary", "delay_pre": 0.3, "delay_post": 0.3}
  step.update(kw)
  return step


def _discover(note="Short-frame Cmd0 at the configured polling address: learns the long address."):
  return {"label": "Cmd0 short addr - discover device", "cmd": 0, "data": b"",
          "use_long": False, "preambles": 5, "master": "primary",
          "delay_pre": 0.5, "delay_post": 0.3, "expect_rc": 0, "note": note}


def _after_reset(label, master="primary", **kw):
  """First Cmd0 after a Device Reset / power cycle: retried until the device
  has booted; Cold Start must be set for this master (proves the reset)."""
  kw.setdefault("expect_status_set", 0)
  kw["expect_status_set"] |= ST_COLD_START
  return _s(label, 0, master=master, expect_rc=0, retries=20, retry_delay=0.5, **kw)


# ---------------------------------------------------------------------------
# Helpers for scenario 17 (Universal Command Conformance, test defs 3.1-3.17)
# ---------------------------------------------------------------------------
def _pack_ascii(text: str, nchars: int) -> bytes:
  """HART Packed-ASCII of nchars characters (4 chars -> 3 bytes)."""
  return encode_packed_ascii(text, nchars * 3 // 4)


def _u(label, cmd, data=b"", long=False, master="primary",
       pre=0.3, post=0.3, note="", **extra):
  """Compact step builder for scenario 17 (short frame by default; the HART 7
  scenarios use _s above, long frame by default)."""
  step = {"label": label, "cmd": cmd, "data": data, "use_long": long,
          "preambles": 5, "master": master, "delay_pre": pre, "delay_post": post,
          "note": note}
  step.update(extra)
  return step


# Test values - edit here if you want different write patterns.
UAL_TAG        = "UALTAG01"                       # 8 chars  -> 6 bytes packed
UAL_DESC       = "UAL CONFORMANCE"                # 16 chars -> 12 bytes packed
UAL_DATE       = bytes([6, 11, 126])              # day, month, year-1900 (06 Nov 2026)
UAL_MESSAGE    = "UAL CONFORMANCE TEST MESSAGE"   # 32 chars -> 24 bytes packed
UAL_ASSEMBLY   = bytes([0x01, 0x23, 0x45])        # final assembly number (3 bytes)
UAL_LONG_TAG   = "UAL-CONFORMANCE-LONG-TAG-0001".ljust(32).encode("latin-1")
UAL_POLL_TEST  = 1                                # temporary polling address for UAL003

_TAG_PACKED  = _pack_ascii(UAL_TAG, 8)
_WR_TAG_DATA = _TAG_PACKED + _pack_ascii(UAL_DESC, 16) + UAL_DATE   # 6+12+3 = 21 bytes
_WR_MSG_DATA = _pack_ascii(UAL_MESSAGE, 32)                         # 24 bytes


def _wp_expect(show_info=False):
  """expect_rc callable for UAL009 write attempts: decided from the write-protect
  code saved from Cmd15 (data byte 15): 1 -> writes must return RC 7,
  0 / 250 / 251 -> writes must succeed (RC 0). Common Table 7: 0 = No,
  1 = Yes, 250 = Not Used, 251 = None (device has no write protection)."""
  def fn(saved):
    p = saved.get("wp")
    if p is None or len(p) < 16:
      return None, "Write-protect code unavailable (Cmd15 read failed) - RC not checked"
    code = p[15]
    if code == 1:
      return [7], ""
    if code in (0, 250, 251):
      return [0], ("Write protect is OFF (code %d) - writes succeeded; RC 7 behaviour "
                   "NOT tested. Enable write protect and rerun." % code) if show_info else ""
    return None, "Unexpected write-protect code %d - RC not checked" % code
  return fn


def _all(*fns):
  """Combine content checks: first failure wins, infos are concatenated."""
  def fn(parsed, saved):
    infos = []
    for f in fns:
      ok, msg = f(parsed, saved)
      if not ok:
        return False, msg
      if msg:
        infos.append(msg)
    return True, " ".join(infos)
  return fn


def _payload(parsed):
  return parsed.get("payload", b"")


def _chk_cmd0_len(parsed, saved):
  """Cmd0-format response (Cmd0/11/21): HART 7 needs 22 data bytes, HART 6 needs 17."""
  p = _payload(parsed)
  if not p or p[0] != 0xFE:
    return True, ""                      # legacy HART 5 format, not checked here
  rev = p[4] if len(p) > 4 else 0
  need = 22 if rev >= 7 else 17
  if len(p) < need:
    miss = (" (manufacturer ID, private-label distributor code and device profile missing)"
            if rev >= 7 and len(p) >= 17 else "")
    return False, "HART %d identity response has %d data bytes, expected %d%s" % (rev, len(p), need, miss)
  return True, ""


def _identity(p):
  return p[:14] + p[17:]                 # skip config counter (14-15) + ext. status (16)


def _chk_same_id(key):
  """Identity data must equal the response saved under `key` (counter/ext status ignored)."""
  def fn(parsed, saved):
    ref = saved.get(key)
    if ref is None:
      return True, "Reference identity not saved - comparison skipped"
    p = _payload(parsed)
    if _identity(p) != _identity(ref):
      diff = [i for i in range(max(len(p), len(ref)))
              if i not in (14, 15, 16) and p[i:i + 1] != ref[i:i + 1]]
      return False, "Identity differs from the reference response at data byte(s) %s" % diff
    return True, ""
  return fn


def _chk_cmd9(codes):
  """Cmd9: ext. device status (1) + 8 bytes/slot (code, class, units, value, status) [+ 4-byte timestamp]."""
  n = len(codes)
  def fn(parsed, saved):
    p = _payload(parsed)
    base = 1 + 8 * n
    if len(p) not in (base, base + 4):
      return False, ("Cmd9 for %d variable(s) returned %d data bytes, expected %d (or %d with timestamp): "
                     "ext. device status + 8 bytes per slot (code, classification, units, value, status)"
                     % (n, len(p), base, base + 4))
    for i, c in enumerate(codes):
      got = p[1 + 8 * i]
      if got != c:
        return False, "Slot %d: device variable code %d, requested %d" % (i, got, c)
    return True, ""
  return fn


def _chk_cmd8(parsed, saved):
  """Cmd8: four classification codes; valid = 0, 64..127 (Common Table 21 area) or 250 (not used)."""
  p = _payload(parsed)
  if len(p) < 4:
    return False, "Cmd8 returned %d data bytes, expected 4" % len(p)
  bad = [c for c in p[:4] if not (c == 0 or 64 <= c <= 127 or c == 250)]
  if bad:
    return False, "Classification code(s) %s are not valid Device Variable Classification codes" % bad
  return True, ""


def _chk_range_units(parsed, saved):
  """Cmd15 range units code must equal the PV units code from Cmd1."""
  pv, p = saved.get("pv"), _payload(parsed)
  if pv is None or not pv or len(p) < 3:
    return True, "PV units not saved - range units comparison skipped"
  if p[2] != pv[0]:
    return False, "Cmd15 range units code %d differs from Cmd1 PV units code %d" % (p[2], pv[0])
  return True, ""


def _chk_cmd38_counter(parsed, saved):
  """Cmd38 response = 2-byte configuration change counter, same as in Cmd0."""
  c0, p = saved.get("c0"), _payload(parsed)
  if len(p) < 2:
    return False, "Cmd38 returned %d data bytes, expected the 2-byte configuration change counter" % len(p)
  if c0 is None or len(c0) < 16:
    return True, "Cmd0 counter not saved - comparison skipped"
  if p[:2] != c0[14:16]:
    return False, "Cmd38 counter 0x%s differs from Cmd0 counter 0x%s" % (p[:2].hex(), c0[14:16].hex())
  return True, ""


def _chk_cmd6_counter(parsed, saved):
  """WARNING only (spec expectation not verified): Cmd6 should change the configuration counter."""
  ref, p = saved.get("cnt6"), _payload(parsed)
  if ref is None or len(ref) < 16 or len(p) < 16:
    return True, "Counter before Cmd6 not saved - check skipped"
  if p[14:16] == ref[14:16]:
    return True, ("Warning: Cmd6 changed the polling address but the configuration change counter "
                  "stayed at 0x%s - verify against the HART spec" % p[14:16].hex())
  return True, ""


def _attach(S, label, **kw):
  """Add keys to the step whose label starts with `label`."""
  for st in S:
    if st["label"].startswith(label):
      st.update(kw)
      return
  raise KeyError(label)


def _ual_steps():
  S = []
  # ---- Setup (needed so long-address, Cmd11 and Cmd21 steps can match) ----
  S += [
    _u("[Setup] Cmd0 - learn unique ID", 0, pre=0.5,
       note="Learns the unique ID used by all long-address steps below."),
    _u("[Setup] Cmd7 - save original polling address", 7, long=True, _save="poll",
       note="Saved so it can be restored at the end of the scenario."),
    _u("[Setup] Cmd12 - save original message", 12, long=True, _save="msg",
       note="Saved for restore."),
    _u("[Setup] Cmd13 - save original tag/descriptor/date", 13, long=True, _save="tag",
       note="Saved for restore."),
    _u("[Setup] Cmd16 - save original final assembly no.", 16, long=True, _save="asm",
       note="Saved for restore."),
    _u("[Setup] Cmd20 - save original long tag", 20, long=True, _save="ltag",
       note="Saved for restore."),
    _u("[Setup] Cmd18 - write test tag/descriptor/date", 18, _WR_TAG_DATA, long=True,
       note="Writes UAL_TAG so Cmd11 can match it. Skipped effect if device is write-protected (RC=7)."),
    _u("[Setup] Cmd22 - write test long tag", 22, UAL_LONG_TAG, long=True,
       note="Writes UAL_LONG_TAG so Cmd21 can match it."),
  ]

  # ---- 3.1 UAL000: every universal command must be implemented (RC != 64) ----
  n_ual000 = len(S)
  S += [
    _u("[3.1 UAL000] Cmd0",  0,  long=True, note="UAL000: RC must NOT be 64 (command not implemented)."),
    _u("[3.1 UAL000] Cmd1",  1,  long=True, note="UAL000: RC != 64."),
    _u("[3.1 UAL000] Cmd2",  2,  long=True, note="UAL000: RC != 64."),
    _u("[3.1 UAL000] Cmd3",  3,  long=True, note="UAL000: RC != 64."),
    _u("[3.1 UAL000] Cmd6", 6, long=True, _data_from="poll",
       note="UAL000: RC != 64. Writes back the original polling address / loop current mode (no change)."),
    _u("[3.1 UAL000] Cmd7",  7,  long=True, note="UAL000: RC != 64."),
    _u("[3.1 UAL000] Cmd8",  8,  long=True, note="UAL000: RC != 64."),
    _u("[3.1 UAL000] Cmd9",  9,  bytes([0, 1, 2, 3]), long=True, note="UAL000: RC != 64."),
    _u("[3.1 UAL000] Cmd11", 11, _TAG_PACKED, long=True, note="UAL000: RC != 64."),
    _u("[3.1 UAL000] Cmd12", 12, long=True, note="UAL000: RC != 64."),
    _u("[3.1 UAL000] Cmd13", 13, long=True, note="UAL000: RC != 64."),
    _u("[3.1 UAL000] Cmd14", 14, long=True, note="UAL000: RC != 64."),
    _u("[3.1 UAL000] Cmd15", 15, long=True, note="UAL000: RC != 64."),
    _u("[3.1 UAL000] Cmd16", 16, long=True, note="UAL000: RC != 64."),
    _u("[3.1 UAL000] Cmd17", 17, _WR_MSG_DATA, long=True, note="UAL000: RC != 64 (RC 7 is fine if write-protected)."),
    _u("[3.1 UAL000] Cmd18", 18, _WR_TAG_DATA, long=True, note="UAL000: RC != 64 (RC 7 is fine if write-protected)."),
    _u("[3.1 UAL000] Cmd19", 19, UAL_ASSEMBLY, long=True, note="UAL000: RC != 64 (RC 7 is fine if write-protected)."),
    _u("[3.1 UAL000] Cmd20", 20, long=True, note="UAL000: RC != 64."),
    _u("[3.1 UAL000] Cmd21", 21, UAL_LONG_TAG, long=True, note="UAL000: RC != 64."),
    _u("[3.1 UAL000] Cmd22", 22, UAL_LONG_TAG, long=True, note="UAL000: RC != 64 (RC 7 is fine if write-protected)."),
    _u("[3.1 UAL000] Cmd38", 38, long=True, note="UAL000: RC != 64."),
    _u("[3.1 UAL000] Cmd48", 48, long=True, note="UAL000: RC != 64."),
  ]

  for st in S[n_ual000:]:
    st["reject_rc"] = [64]   # UAL000 fails if any command is "not implemented"

  # ---- 3.2 UAL001: Commands 1, 2, 3 ----
  S += [
    _u("[3.2 UAL001] Cmd1 - PV (short)",     1, note="UAL001: PV unit code + float. Expect 5 data bytes."),
    _u("[3.2 UAL001] Cmd1 - PV (long)",      1, long=True, note="Same via long address."),
    _u("[3.2 UAL001] Cmd2 - current/%range", 2, note="UAL001: loop current + percent of range (8 data bytes)."),
    _u("[3.2 UAL001] Cmd2 (long)",           2, long=True),
    _u("[3.2 UAL001] Cmd3 - dynamic vars",   3, note="UAL001: loop current + up to 4 (unit,float) pairs. Values consistent with Cmd1/Cmd2."),
    _u("[3.2 UAL001] Cmd3 (long)",           3, long=True),
  ]

  # ---- 3.3 UAL003: Commands 6 and 7 (polling address) ----
  S += [
    _u("[3.3 UAL003] Cmd7 - read loop config (before)", 7, long=True,
       note="Records current polling address + loop current mode."),
    _u("[3.3 UAL003] Cmd6 - write poll addr = %d" % UAL_POLL_TEST, 6, bytes([UAL_POLL_TEST, 1]), long=True,
       note="Long-addr write so it works regardless of current polling address."),
    _u("[3.3 UAL003] Cmd7 - confirm new address", 7, long=True, pre=0.5,
       note="Response must report the new polling address."),
    _u("[3.3 UAL003] Cmd0 short @ new addr %d" % UAL_POLL_TEST, 0, pre=0.5, _addr=UAL_POLL_TEST,
       note="Device must now answer short-frame polling at the new address."),
    _u("[3.3 UAL003] Cmd6 - restore original poll addr", 6, long=True, _data_from="poll",
       note="Restores the address saved in setup so remaining steps use the UI poll address."),
    _u("[3.3 UAL003] Cmd7 - confirm restored", 7, long=True, pre=0.5),
    _u("[3.3 UAL003] Cmd0 short @ restored addr", 0, pre=0.5,
       note="Uses the UI 'Poll addr' - must equal the device's original polling address."),
  ]

  # ---- 3.4 UAL004: Command 11 ----
  S += [
    _u("[3.4 UAL004] Cmd11 - matching tag (long)", 11, _TAG_PACKED, long=True,
       note="Response data must be identical to Cmd0 (manufacturer, type, revs, device ID)."),
    _u("[3.4 UAL004] Cmd0 - compare with Cmd11", 0, long=True,
       note="Compare identity fields with the Cmd11 result above."),
  ]

  # ---- 3.5 UAL005: Commands 12 and 17 ----
  S += [
    _u("[3.5 UAL005] Cmd12 - read message (before)", 12, note="24 data bytes (32 packed chars)."),
    _u("[3.5 UAL005] Cmd17 - write message", 17, _WR_MSG_DATA, post=0.4,
       note="Writes UAL_MESSAGE. RC=7 means device is write-protected (see 3.9)."),
    _u("[3.5 UAL005] Cmd12 - read back", 12,
       note="Must return the message just written."),
  ]

  # ---- 3.6 UAL006: Commands 13 and 18 ----
  S += [
    _u("[3.6 UAL006] Cmd13 - read tag/desc/date (before)", 13, note="21 data bytes."),
    _u("[3.6 UAL006] Cmd18 - write tag/desc/date", 18, _WR_TAG_DATA, post=0.4,
       note="Writes UAL_TAG / UAL_DESC / UAL_DATE."),
    _u("[3.6 UAL006] Cmd13 - read back", 13,
       note="Tag, descriptor and date must match the values written."),
  ]

  # ---- 3.7 UAL007: Commands 14 and 15 ----
  S += [
    _u("[3.7 UAL007] Cmd14 - PV transducer info", 14,
       note="Sensor S/N, limits units, upper/lower sensor limits, min span."),
    _u("[3.7 UAL007] Cmd14 (long)", 14, long=True),
    _u("[3.7 UAL007] Cmd15 - output info", 15,
       note="Alarm select, transfer function, range units, upper/lower range, damping, write-protect code, private-label distributor."),
    _u("[3.7 UAL007] Cmd15 (long)", 15, long=True),
  ]

  # ---- 3.8 UAL008: Commands 16 and 19 ----
  S += [
    _u("[3.8 UAL008] Cmd16 - read final assembly (before)", 16, note="3 data bytes."),
    _u("[3.8 UAL008] Cmd19 - write final assembly", 19, UAL_ASSEMBLY, post=0.4,
       note="Writes UAL_ASSEMBLY."),
    _u("[3.8 UAL008] Cmd16 - read back", 16,
       note="Must return the value just written."),
  ]

  # ---- 3.9 UAL009: Write protect ----
  S += [
    _u("[3.9 UAL009] Cmd15 - read write-protect code", 15, _save="wp",
       note="Write-protect code is Cmd15 data byte 15: 0 = no, 1 = yes, 250 = not used. If 1, every write below must return RC=7."),
    _u("[3.9 UAL009] Cmd6 write attempt",  6,  long=True, _data_from="poll", expect_rc=_wp_expect(True), note="RC=7 expected if write-protected."),
    _u("[3.9 UAL009] Cmd17 write attempt", 17, _WR_MSG_DATA, expect_rc=_wp_expect(), note="RC=7 expected if write-protected."),
    _u("[3.9 UAL009] Cmd18 write attempt", 18, _WR_TAG_DATA, expect_rc=_wp_expect(), note="RC=7 expected if write-protected."),
    _u("[3.9 UAL009] Cmd19 write attempt", 19, UAL_ASSEMBLY, expect_rc=_wp_expect(), note="RC=7 expected if write-protected."),
    _u("[3.9 UAL009] Cmd22 write attempt", 22, UAL_LONG_TAG, long=True, expect_rc=_wp_expect(), note="RC=7 expected if write-protected."),
    _u("[3.9 UAL009] Cmd0 - verify still alive", 0),
  ]

  # ---- 3.10 UAL010: Cold start bit ----
  S += [
    _u("[3.10 UAL010] Cmd0 - check cold start bit", 0, pre=0.5,
       note="Cold start (ST2 0x20) is only set after a power cycle. Power-cycle DUT and run this scenario right away for a meaningful result."),
    _u("[3.10 UAL010] Cmd0 - bit must be clear", 0,
       note="Cold start bit must be cleared after the first exchange."),
  ]

  # ---- 3.11 UAL011: Command 9 ----
  S += [
    _u("[3.11 UAL011] Cmd9 - slots 0..3", 9, bytes([0, 1, 2, 3]), note="Each variable: code, class, units, float, status. Response must echo requested codes."),
    _u("[3.11 UAL011] Cmd9 - dynamic vars PV/SV/TV/QV (246..249)", 9, bytes([246, 247, 248, 249])),
    _u("[3.11 UAL011] Cmd9 - single var", 9, bytes([0]), note="Minimal request (1 variable)."),
    _u("[3.11 UAL011] Cmd9 (long)", 9, bytes([0, 1, 2, 3]), long=True),
  ]

  # ---- 3.12 UAL012: Command 8 ----
  S += [
    _u("[3.12 UAL012] Cmd8 - dynamic variable classification", 8,
       note="4 data bytes: classification codes for PV/SV/TV/QV."),
    _u("[3.12 UAL012] Cmd8 (long)", 8, long=True),
  ]

  # ---- 3.13 UAL013: Commands 20 and 22 ----
  S += [
    _u("[3.13 UAL013] Cmd20 - read long tag (before)", 20, note="32 data bytes."),
    _u("[3.13 UAL013] Cmd22 - write long tag", 22, UAL_LONG_TAG, long=True, post=0.4,
       note="Writes UAL_LONG_TAG (32 bytes, space padded)."),
    _u("[3.13 UAL013] Cmd20 - read back", 20, note="Must match the long tag written."),
  ]

  # ---- 3.14 UAL014: Command 0 ----
  S += [
    _u("[3.14 UAL014] Cmd0 (short)", 0, pre=0.5, note="Expect 0xFE expansion code, HART revision, device IDs."),
    _u("[3.14 UAL014] Cmd0 (long)",  0, long=True, note="Must return same identity as short frame."),
    _u("[3.14 UAL014] Cmd0 (secondary master)", 0, master="secondary",
       note="Secondary-master frame must also be answered."),
  ]

  # ---- 3.15 UAL015: Command 21 ----
  S += [
    _u("[3.15 UAL015] Cmd21 - matching long tag", 21, UAL_LONG_TAG, long=True,
       note="Response data identical to Cmd0."),
    _u("[3.15 UAL015] Cmd0 - compare with Cmd21", 0, long=True),
  ]

  # ---- 3.16 UAL038: Command 38 ----
  S += [
    _u("[3.16 UAL038] Cmd0 - config-changed bit (before)", 0,
       note="After the earlier writes, ST2 'Configuration changed' (0x40) is expected to be set."),
    _u("[3.16 UAL038] Cmd38 - reset configuration changed flag", 38, long=True,
       note="HART 7 devices may expect the 2-byte config change counter; if RC=5 add it from Cmd0 bytes 14-15."),
    _u("[3.16 UAL038] Cmd0 - config-changed bit must be clear", 0, pre=0.5,
       note="'Configuration changed' bit must now be cleared."),
  ]

  # ---- 3.17 UAL048: Command 48 ----
  S += [
    _u("[3.17 UAL048] Cmd48 - additional device status", 48,
       note="Expect >= 9 status bytes (HART 6/7). Check byte 0-5 specific status, byte 6 ext device status, byte 7 op mode, byte 8 standardized status 0."),
    _u("[3.17 UAL048] Cmd48 (long)", 48, long=True),
    _u("[3.17 UAL048] Cmd0 - 'More status available' bit", 0,
       note="If ST2 0x10 is set, Cmd48 must return non-zero status."),
  ]
  # ---- Teardown: restore the original device values saved in setup ----
  S += [
    _u("[Restore] Cmd6 - original polling address", 6, long=True, _data_from="poll", post=0.4,
       note="Writes back the Cmd7 value captured in setup."),
    _u("[Restore] Cmd17 - original message", 17, long=True, _data_from="msg", post=0.4),
    _u("[Restore] Cmd18 - original tag/descriptor/date", 18, long=True, _data_from="tag", post=0.4),
    _u("[Restore] Cmd19 - original final assembly no.", 19, long=True, _data_from="asm", post=0.4),
    _u("[Restore] Cmd22 - original long tag", 22, long=True, _data_from="ltag", post=0.4),
    _u("[Restore] Cmd38 - clear config-changed flag", 38, long=True,
       note="The restore writes set 'Configuration changed' again; clear it. Add the 2-byte counter if RC=5."),
    _u("[Restore] Cmd0 - final check", 0, pre=0.5,
       note="Device should answer at its original polling address."),
  ]
  # ---- Response content checks (pass/fail) ----
  i = next(k for k, st in enumerate(S) if st["label"].startswith("[3.3 UAL003] Cmd6 - write poll addr"))
  S.insert(i, _u("[3.3 UAL003] Cmd0 - config counter (before Cmd6)", 0, long=True, _save="cnt6",
                 note="Saves the configuration change counter to see whether Cmd6 changes it."))
  _attach(S, "[3.2 UAL001] Cmd1 - PV (short)", _save="pv")
  _attach(S, "[3.3 UAL003] Cmd0 short @ new addr", check=_chk_cmd6_counter)
  _attach(S, "[3.4 UAL004] Cmd11 - matching tag", _save="id11", check=_chk_cmd0_len)
  _attach(S, "[3.4 UAL004] Cmd0 - compare with Cmd11", check=_chk_same_id("id11"))
  _attach(S, "[3.7 UAL007] Cmd15 - output info", check=_chk_range_units)
  # Cmd9 RC 14 = Warning "Dynamic Variables Returned for Device Variables":
  # the full response is still returned (device maps codes 0-3 to PV..QV)
  CMD9_RC = [0, 14]
  _attach(S, "[3.11 UAL011] Cmd9 - slots 0..3", expect_rc=CMD9_RC, check=_chk_cmd9([0, 1, 2, 3]))
  _attach(S, "[3.11 UAL011] Cmd9 - dynamic vars", expect_rc=CMD9_RC, check=_chk_cmd9([246, 247, 248, 249]))
  _attach(S, "[3.11 UAL011] Cmd9 - single var", expect_rc=CMD9_RC, check=_chk_cmd9([0]))
  _attach(S, "[3.11 UAL011] Cmd9 (long)", expect_rc=CMD9_RC, check=_chk_cmd9([0, 1, 2, 3]))
  _attach(S, "[3.12 UAL012] Cmd8 - dynamic variable classification", check=_chk_cmd8)
  _attach(S, "[3.12 UAL012] Cmd8 (long)", check=_chk_cmd8)
  _attach(S, "[3.14 UAL014] Cmd0 (short)", _save="id0", check=_chk_cmd0_len)
  _attach(S, "[3.14 UAL014] Cmd0 (long)", check=_all(_chk_cmd0_len, _chk_same_id("id0")))
  _attach(S, "[3.14 UAL014] Cmd0 (secondary master)", check=_all(_chk_cmd0_len, _chk_same_id("id0")))
  _attach(S, "[3.15 UAL015] Cmd21 - matching long tag", _save="id21", check=_chk_cmd0_len)
  _attach(S, "[3.15 UAL015] Cmd0 - compare with Cmd21", check=_chk_same_id("id21"))
  _attach(S, "[3.16 UAL038] Cmd0 - config-changed bit (before)", _save="c0")
  _attach(S, "[3.16 UAL038] Cmd38 - reset configuration changed flag", check=_chk_cmd38_counter)
  return S


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
      "Emerson Trex communicates as secondary master: address byte bit 7 = 0\n"
      "(primary master: bit 7 = 1). The delimiter is STX 0x02/0x82 for both.\n"
      "Sends Cmd0 first as primary then as secondary.\n"
      "Slave must respond to both, echoing the master bit in its response.\n"
      "Diagnoses: slave rejects secondary master requests,\n"
      "           or answers the wrong master."
    ),
    "steps": [
      {"label": "Cmd0 - primary master", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.5, "delay_post": 0.5,
       "note": "Baseline: primary master (address bit 7 = 1)."},
      {"label": "Cmd0 - secondary master", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "secondary",
       "delay_pre": 0.5, "delay_post": 0.5,
       "note": "Secondary master (address bit 7 = 0). Slave must also respond."},
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
       "_corrupt_cs": True, "expect_no_response": True},
      {"label": "BAD CheckSum frame #2", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.5, "delay_post": 0.5,
       "note": "Second bad-CS frame.",
       "_corrupt_cs": True, "expect_no_response": True},
      {"label": "BAD CheckSum frame #3", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary",
       "delay_pre": 0.5, "delay_post": 0.5,
       "note": "Third bad-CS frame.",
       "_corrupt_cs": True, "expect_no_response": True},
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

  "11 - Write Commands (Cmd17/18/19) - echo, counter, restore": {
    "description": (
      "Writes Message (Cmd17), Tag/Descriptor/Date (Cmd18) and Final Assembly\n"
      "Number (Cmd19) with valid packed-ASCII / date values, then:\n"
      "• response must echo the written data (RC 0)\n"
      "• too-short requests must return RC 5 and change nothing\n"
      "• Config Change Counter +1 per accepted write, Config Changed set\n"
      "• Cmd12/13/16 read back the new values; originals restored at the end\n"
      "Write protect is not implemented in this device (Cmd15 reports 251), so\n"
      "RC 7 (In Write Protect Mode) cannot be provoked here."
    ),
    "steps": [
      _discover(),
      _s("Cmd0 - counter before", 0, expect_rc=0, save_as="c0_before"),
      _s("Cmd12 - read original message", 12, expect_rc=0, save_as="orig12"),
      _s("Cmd13 - read original tag/descriptor/date", 13, expect_rc=0, save_as="orig13"),
      _s("Cmd16 - read original final assembly number", 16, expect_rc=0, save_as="orig16"),
      _s("Cmd17 - write message", 17, NV_MSG, expect_rc=0, expect_payload=NV_MSG,
         note="Response must echo the 24 data bytes actually stored."),
      _s("Cmd18 - write tag/descriptor/date", 18, NV_C18, expect_rc=0, expect_payload=NV_C18),
      _s("Cmd19 - write final assembly number", 19, NV_FAN, expect_rc=0, expect_payload=NV_FAN),
      _s("Cmd17 - 10 bytes only (expect RC 5)", 17, NV_MSG[:10], expect_rc=5),
      _s("Cmd18 - 5 bytes only (expect RC 5)", 18, NV_C18[:5], expect_rc=5),
      _s("Cmd19 - 2 bytes only (expect RC 5)", 19, NV_FAN[:2], expect_rc=5),
      _s("Cmd0 - counter +3, Config Changed set", 0, expect_rc=0,
         expect_status_set=ST_CONFIG_CHANGED,
         expect_u16={"offset": CCC_OFFSET, "ref": "c0_before", "delta": 3},
         note="Only the 3 accepted writes count; RC 5 requests must not change the counter."),
      _s("Cmd12 - read back message", 12, expect_rc=0, expect_payload=NV_MSG),
      _s("Cmd13 - read back tag/descriptor/date", 13, expect_rc=0, expect_payload=NV_C18),
      _s("Cmd16 - read back final assembly number", 16, expect_rc=0, expect_payload=NV_FAN),
      _s("Cmd17 - restore message", 17, data_from="orig12", expect_rc=0, expect_payload="orig12"),
      _s("Cmd18 - restore tag/descriptor/date", 18, data_from="orig13", expect_rc=0, expect_payload="orig13"),
      _s("Cmd19 - restore final assembly number", 19, data_from="orig16", expect_rc=0,
         expect_payload="orig16"),
      _s("Cmd38 primary - clear Config Changed", 38, expect_rc=0),
      _s("Cmd38 secondary - clear Config Changed", 38, master="secondary", expect_rc=0,
         delay_post=NV_SAVE_WAIT_S),
    ]
  },

  "12 - Configuration Changed per Master + Cmd38": {
    "description": (
      "HART 7.3.3: one Configuration Changed bit per master. A configuration\n"
      "write sets it for BOTH masters; Cmd38 clears it only for the master that\n"
      "sends it, and only if the (optional) counter matches (else RC 9).\n"
      "Cmd0 does NOT clear it. The write here re-writes the current Cmd18\n"
      "values, so the device configuration is unchanged.\n"
      "Diagnoses: single shared bit, Cmd38 clearing both masters,\n"
      "           counter check missing, bit cleared by other commands."
    ),
    "steps": [
      _discover(),
      _s("Cmd13 - read tag/descriptor/date", 13, expect_rc=0, save_as="orig13"),
      _s("Cmd18 - re-write the same values", 18, data_from="orig13", expect_rc=0,
         expect_payload="orig13", note="Any accepted write is a configuration change."),
      _s("Cmd0 primary - Config Changed set", 0, expect_rc=0,
         expect_status_set=ST_CONFIG_CHANGED, save_as="c0_a"),
      _s("Cmd0 primary again - still set (Cmd0 must not clear it)", 0, expect_rc=0,
         expect_status_set=ST_CONFIG_CHANGED),
      _s("Cmd0 secondary - Config Changed set", 0, master="secondary", expect_rc=0,
         expect_status_set=ST_CONFIG_CHANGED),
      _s("Cmd38 primary - wrong counter (expect RC 9)", 38,
         data_u16={"ref": "c0_a", "offset": CCC_OFFSET, "delta": 1}, expect_rc=9,
         expect_status_set=ST_CONFIG_CHANGED,
         expect_payload_one_of=[b"", ("c0_a", CCC_OFFSET, CCC_OFFSET + 2)],
         note="Device counter + 1 never matches. Bit must stay set. The response may carry the counter."),
      _s("Cmd38 primary - matching counter", 38, data_from=("c0_a", CCC_OFFSET, CCC_OFFSET + 2),
         expect_rc=0, expect_status_clear=ST_CONFIG_CHANGED,
         expect_payload=("c0_a", CCC_OFFSET, CCC_OFFSET + 2),
         note="Bit is reset immediately, i.e. already clear in this response."),
      _s("Cmd0 primary - Config Changed clear", 0, expect_rc=0,
         expect_status_clear=ST_CONFIG_CHANGED,
         expect_u16={"offset": CCC_OFFSET, "ref": "c0_a", "delta": 0},
         note="Cmd38 must not change the Config Change Counter."),
      _s("Cmd0 secondary - still set", 0, master="secondary", expect_rc=0,
         expect_status_set=ST_CONFIG_CHANGED,
         note="Cmd38 from the primary master must not affect the secondary master's bit."),
      _s("Cmd38 secondary - no counter (HART 6 style)", 38, master="secondary", expect_rc=0,
         expect_status_clear=ST_CONFIG_CHANGED),
      _s("Cmd0 secondary - Config Changed clear", 0, master="secondary", expect_rc=0,
         expect_status_clear=ST_CONFIG_CHANGED, delay_post=NV_SAVE_WAIT_S),
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

  "14 - Broadcast Address (long frame 00 00 00 00 00)": {
    "description": (
      "HART 7 Broadcast Address = the all-zero 38-bit long frame address.\n"
      "Only identity commands Cmd11/Cmd21 may use it (and answer only if the\n"
      "tag matches); the response must carry the same (broadcast) address.\n"
      "• Cmd0/Cmd1/Cmd6 to the broadcast address: device must stay silent\n"
      "• Cmd11 with the device's tag: response, Cmd0 data\n"
      "• Cmd11 with a wrong tag: no response\n"
      "Note: short polling address 63 is a valid HART 7 polling address (0-63)\n"
      "and is not a broadcast address; it is not tested here."
    ),
    "steps": [
      _discover(),
      _s("Cmd13 - read own tag", 13, expect_rc=0, save_as="orig13"),
      _s("Cmd0 - to broadcast addr (expect NO response)", 0, broadcast=True,
         expect_no_response=True, delay_post=0.5),
      _s("Cmd1 - to broadcast addr (expect NO response)", 1, broadcast=True,
         expect_no_response=True, delay_post=0.5),
      _s("Cmd6 - to broadcast addr (expect NO response)", 6, b"\x00\x01", broadcast=True,
         expect_no_response=True, delay_post=0.5,
         note="A broadcast write would re-address every device on the loop."),
      _s("Cmd11 - broadcast, matching tag", 11, data_from=("orig13", 0, 6), broadcast=True,
         expect_rc=0, save_as="c11_bcast",
         note="Response address must be the broadcast address (same as request)."),
      _s("Cmd11 - broadcast, wrong tag (expect NO response)", 11, WRONG_TAG, broadcast=True,
         expect_no_response=True, delay_post=0.5),
      _s("Cmd11 - own long address, matching tag", 11, data_from=("orig13", 0, 6), expect_rc=0,
         expect_payload=("c11_bcast", 0, 14), payload_slice=[0, 14],
         note="Same identity data as the broadcast answer (bytes 0-13)."),
      _s("Cmd0 - recovery check", 0, expect_rc=0),
    ]
  },

  "15 - Cold Start per Master after Device Reset (Cmd42)": {
    "description": (
      "HART 7.3.3: one Cold Start bit per master, both set at power-up/reset;\n"
      "the first response to each master clears that master's bit only.\n"
      "• Cmd42 Device Reset, wait for reboot\n"
      "• primary: 1st Cmd0 Cold Start set, 2nd clear\n"
      "• secondary: 1st Cmd0 Cold Start still set, 2nd clear\n"
      "Diagnoses: single shared Cold Start bit, bit never set, bit never cleared."
    ),
    "steps": [
      _discover(),
      _s("Cmd42 - Device Reset", 42, expect_rc=0, delay_post=RESET_BOOT_WAIT_S,
         note="Device must answer first, then reset."),
      _after_reset("Cmd0 primary - Cold Start set"),
      _s("Cmd0 primary - Cold Start clear", 0, expect_rc=0, expect_status_clear=ST_COLD_START),
      _s("Cmd0 secondary - Cold Start still set", 0, master="secondary", expect_rc=0,
         expect_status_set=ST_COLD_START,
         note="Serving the primary master must not clear the secondary master's bit."),
      _s("Cmd0 secondary - Cold Start clear", 0, master="secondary", expect_rc=0,
         expect_status_clear=ST_COLD_START),
    ]
  },

  "16 - Invalid Command Exception Handling": {
    "description": (
      "Injects unimplemented command code to monitor slave's response logic:\n"
      "• Sends invalid Cmd 99\n"
      "• Expects response code 64 ('Command not implemented')\n"
      "• Verifies slave doesn't freeze up under invalid inputs."
    ),
    "steps": [
      _discover(),
      _s("Send Unimplemented Cmd 99 (long frame)", 99, b"", expect_rc=64,
         note="Slave must respond with error code 64, not time out. Long frame: in HART 7 only "
              "Cmd0 may be answered on a short (polling) address."),
      _s("Cmd0 - still alive", 0, expect_rc=0),
    ]
  },

  "17 - Universal Command Conformance (UAL 3.1-3.17)": {
    "description": (
      "HART universal command conformance tests, sections 3.1-3.17:\n"
      "UAL000 all commands supported | UAL001 Cmd1/2/3 | UAL003 Cmd6/7 | UAL004 Cmd11\n"
      "UAL005 Cmd12/17 | UAL006 Cmd13/18 | UAL007 Cmd14/15 | UAL008 Cmd16/19\n"
      "UAL009 write protect | UAL010 cold start | UAL011 Cmd9 | UAL012 Cmd8\n"
      "UAL013 Cmd20/22 | UAL014 Cmd0 | UAL015 Cmd21 | UAL038 Cmd38 | UAL048 Cmd48\n"
      "Checked: UAL000 RC!=64, UAL009 RC vs write-protect, Cmd0/11/21 length+identity,\n"
      "Cmd9 layout, Cmd8 codes, Cmd15 vs Cmd1 units, Cmd38 counter (Cmd6 counter = warning).\n"
      "WRITES to device, then restores the original tag, descriptor, date, message,\n"
      "assembly no., long tag and polling addr (saved in setup). Power-cycle DUT\n"
      "before the run for UAL010. Poll addr in UI must be the device's current addr."
    ),
    "steps": _ual_steps()
  },

  "18 - NV Persistence across Device Reset (Cmd42)": {
    "description": (
      "HART 7 configuration must survive a Device Reset (and power loss):\n"
      "• read originals (Cmd12/13/16/20), write new Message, Tag/Descriptor/Date,\n"
      "  Final Assembly Number and Long Tag (Cmd17/18/19/22)\n"
      "• Config Change Counter +4, Config Changed set for both masters\n"
      "• primary clears its Config Changed bit with Cmd38\n"
      "• Cmd42 Device Reset; Cold Start set after reboot proves the reset\n"
      "• after reboot: all values, the counter and the per-master Config Changed\n"
      "  bits (primary clear, secondary set) must be as before the reset\n"
      "• originals restored at the end (the counter keeps counting)"
    ),
    "steps": [
      _discover(),
      _s("Cmd0 secondary - serve secondary once", 0, master="secondary", expect_rc=0),
      _s("Cmd12 - read original message", 12, expect_rc=0, save_as="orig12"),
      _s("Cmd13 - read original tag/descriptor/date", 13, expect_rc=0, save_as="orig13"),
      _s("Cmd16 - read original final assembly number", 16, expect_rc=0, save_as="orig16"),
      _s("Cmd20 - read original long tag", 20, expect_rc=0, save_as="orig20"),
      _s("Cmd0 - counter before", 0, expect_rc=0, save_as="c0_before"),
      _s("Cmd17 - write message", 17, NV_MSG, expect_rc=0, expect_payload=NV_MSG),
      _s("Cmd18 - write tag/descriptor/date", 18, NV_C18, expect_rc=0, expect_payload=NV_C18),
      _s("Cmd19 - write final assembly number", 19, NV_FAN, expect_rc=0, expect_payload=NV_FAN),
      _s("Cmd22 - write long tag", 22, NV_LONG_TAG, expect_rc=0, expect_payload=NV_LONG_TAG),
      _s("Cmd0 - counter +4, Config Changed set", 0, expect_rc=0,
         expect_status_set=ST_CONFIG_CHANGED, save_as="c0_written",
         expect_u16={"offset": CCC_OFFSET, "ref": "c0_before", "delta": 4}),
      _s("Cmd38 primary - reset Config Changed (counter check)", 38,
         data_from=("c0_written", CCC_OFFSET, CCC_OFFSET + 2), expect_rc=0,
         expect_status_clear=ST_CONFIG_CHANGED),
      _s("Cmd0 secondary - Config Changed still set", 0, master="secondary", expect_rc=0,
         expect_status_set=ST_CONFIG_CHANGED, delay_post=NV_SAVE_WAIT_S,
         note="Then wait for the EEPROM save to finish."),
      _s("Cmd42 - Device Reset", 42, expect_rc=0, delay_post=RESET_BOOT_WAIT_S),
      _after_reset("Cmd0 primary after reset - counter, Config Changed clear",
                   expect_status_clear=ST_CONFIG_CHANGED,
                   expect_u16={"offset": CCC_OFFSET, "ref": "c0_written", "delta": 0},
                   note="Cold Start set = the device really rebooted (RAM was cleared)."),
      _after_reset("Cmd0 secondary after reset - Config Changed still set", master="secondary",
                   expect_status_set=ST_CONFIG_CHANGED,
                   expect_u16={"offset": CCC_OFFSET, "ref": "c0_written", "delta": 0}),
      _s("Cmd12 - message persisted", 12, expect_rc=0, expect_payload=NV_MSG),
      _s("Cmd13 - tag/descriptor/date persisted", 13, expect_rc=0, expect_payload=NV_C18),
      _s("Cmd16 - final assembly number persisted", 16, expect_rc=0, expect_payload=NV_FAN),
      _s("Cmd20 - long tag persisted", 20, expect_rc=0, expect_payload=NV_LONG_TAG),
      _s("Cmd17 - restore message", 17, data_from="orig12", expect_rc=0, expect_payload="orig12"),
      _s("Cmd18 - restore tag/descriptor/date", 18, data_from="orig13", expect_rc=0, expect_payload="orig13"),
      _s("Cmd19 - restore final assembly number", 19, data_from="orig16", expect_rc=0, expect_payload="orig16"),
      _s("Cmd22 - restore long tag", 22, data_from="orig20", expect_rc=0, expect_payload="orig20"),
      _s("Cmd38 primary - clear Config Changed", 38, expect_rc=0),
      _s("Cmd38 secondary - clear Config Changed", 38, master="secondary", expect_rc=0,
         delay_post=NV_SAVE_WAIT_S),
    ]
  },

  "19 - NV Persistence of Polling Address (Cmd6)": {
    "description": (
      "Cmd6 polling address and loop current mode must survive a Device Reset.\n"
      "Cmd6/Cmd7/Cmd42 use the long address, so the scenario can always restore\n"
      "the original address even if a step fails.\n"
      "• Cmd6 -> polling address 5 (loop current mode kept enabled)\n"
      "• device answers at 5, not at the configured address\n"
      "• Cmd42 reset: still at 5 after reboot; Cmd7 reads 5\n"
      "• original address restored"
    ),
    "steps": [
      _discover("Device must be at the polling address set in the GUI (not 5)."),
      _s("Cmd7 - read original loop configuration", 7, expect_rc=0, save_as="orig7"),
      _s("Cmd6 - polling address 5", 6, b"\x05\x01", expect_rc=0, expect_payload=b"\x05\x01"),
      {"label": "Cmd0 short addr 5 - answers", "cmd": 0, "data": b"", "use_long": False,
       "preambles": 5, "master": "primary", "_addr": 5, "delay_pre": 0.3, "delay_post": 0.3,
       "expect_rc": 0},
      {"label": "Cmd0 short configured addr - silent", "cmd": 0, "data": b"", "use_long": False,
       "preambles": 5, "master": "primary", "delay_pre": 0.3, "delay_post": NV_SAVE_WAIT_S,
       "expect_no_response": True},
      _s("Cmd42 - Device Reset", 42, expect_rc=0, delay_post=RESET_BOOT_WAIT_S),
      {"label": "Cmd0 short addr 5 after reset - answers, Cold Start", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary", "_addr": 5,
       "delay_pre": 0.3, "delay_post": 0.3, "expect_rc": 0,
       "expect_status_set": ST_COLD_START, "retries": 20, "retry_delay": 0.5},
      {"label": "Cmd0 short configured addr after reset - silent", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary", "delay_pre": 0.3,
       "delay_post": 0.3, "expect_no_response": True},
      _s("Cmd7 - polling address 5 persisted", 7, expect_rc=0, expect_payload=b"\x05\x01"),
      _s("Cmd6 - restore original loop configuration", 6, data_from="orig7", expect_rc=0,
         expect_payload="orig7"),
      {"label": "Cmd0 short configured addr - answers again", "cmd": 0, "data": b"",
       "use_long": False, "preambles": 5, "master": "primary", "delay_pre": 0.3,
       "delay_post": 0.3, "expect_rc": 0},
      _s("Cmd38 primary - clear Config Changed", 38, expect_rc=0),
      _s("Cmd38 secondary - clear Config Changed", 38, master="secondary", expect_rc=0,
         delay_post=NV_SAVE_WAIT_S),
    ]
  },

  "20 - NV: Device Reset right after a write": {
    "description": (
      "Cmd42 sent immediately after an acknowledged Cmd18, while the EEPROM\n"
      "save may still be in progress. HART 7.3.3: the Config Change Counter\n"
      "(and the change) must be maintained across a Device Reset, so after the\n"
      "reboot the NEW tag and counter+1 must be read back.\n"
      "Diagnoses: Device Reset discarding a pending/partial NV save."
    ),
    "steps": [
      _discover(),
      _s("Cmd13 - read original tag/descriptor/date", 13, expect_rc=0, save_as="orig13"),
      _s("Cmd0 - counter before", 0, expect_rc=0, save_as="c0_before", delay_post=NV_SAVE_WAIT_S),
      _s("Cmd18 - write tag/descriptor/date", 18, TORN_C18, expect_rc=0, expect_payload=TORN_C18,
         delay_post=0.0),
      _s("Cmd42 - Device Reset immediately", 42, expect_rc=0, delay_pre=0.0,
         delay_post=RESET_BOOT_WAIT_S),
      _after_reset("Cmd0 after reset - counter +1",
                   expect_u16={"offset": CCC_OFFSET, "ref": "c0_before", "delta": 1}),
      _s("Cmd13 - new tag/descriptor/date persisted", 13, expect_rc=0, expect_payload=TORN_C18),
      _s("Cmd18 - restore tag/descriptor/date", 18, data_from="orig13", expect_rc=0,
         expect_payload="orig13"),
      _s("Cmd38 primary - clear Config Changed", 38, expect_rc=0),
      _s("Cmd38 secondary - clear Config Changed", 38, master="secondary", expect_rc=0,
         delay_post=NV_SAVE_WAIT_S),
    ]
  },

  "21 - NV Persistence across Power Cycle (manual)": {
    "manual_only": True,
    "description": (
      "Same checks as scenario 18, but across a real power loss.\n"
      "The operator is asked to remove power for at least 5 s and restore it.\n"
      "Cold Start set after power-up proves the device was really off.\n"
      "Not part of RUN ALL (needs the operator)."
    ),
    "steps": [
      _discover(),
      _s("Cmd12 - read original message", 12, expect_rc=0, save_as="orig12"),
      _s("Cmd13 - read original tag/descriptor/date", 13, expect_rc=0, save_as="orig13"),
      _s("Cmd20 - read original long tag", 20, expect_rc=0, save_as="orig20"),
      _s("Cmd17 - write message", 17, NV_MSG, expect_rc=0, expect_payload=NV_MSG),
      _s("Cmd18 - write tag/descriptor/date", 18, NV_C18, expect_rc=0, expect_payload=NV_C18),
      _s("Cmd22 - write long tag", 22, NV_LONG_TAG, expect_rc=0, expect_payload=NV_LONG_TAG),
      _s("Cmd0 - counter after writes", 0, expect_rc=0, save_as="c0_written",
         expect_status_set=ST_CONFIG_CHANGED, delay_post=NV_SAVE_WAIT_S),
      {"label": "Operator: power-cycle the device",
       "manual": ("Remove power from the device for at least 5 seconds, restore power and "
                  "wait until it has booted (about 3 s)."),
       "delay_post": 1.0},
      _after_reset("Cmd0 after power-up - counter, Config Changed kept",
                   expect_status_set=ST_CONFIG_CHANGED,
                   expect_u16={"offset": CCC_OFFSET, "ref": "c0_written", "delta": 0}),
      _s("Cmd12 - message persisted", 12, expect_rc=0, expect_payload=NV_MSG),
      _s("Cmd13 - tag/descriptor/date persisted", 13, expect_rc=0, expect_payload=NV_C18),
      _s("Cmd20 - long tag persisted", 20, expect_rc=0, expect_payload=NV_LONG_TAG),
      _s("Cmd17 - restore message", 17, data_from="orig12", expect_rc=0, expect_payload="orig12"),
      _s("Cmd18 - restore tag/descriptor/date", 18, data_from="orig13", expect_rc=0, expect_payload="orig13"),
      _s("Cmd22 - restore long tag", 22, data_from="orig20", expect_rc=0, expect_payload="orig20"),
      _s("Cmd38 primary - clear Config Changed", 38, expect_rc=0),
      _s("Cmd38 secondary - clear Config Changed", 38, master="secondary", expect_rc=0,
         delay_post=NV_SAVE_WAIT_S),
    ]
  },
}
