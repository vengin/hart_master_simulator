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

# ---------------------------------------------------------------------------
# Helpers for scenario 17 (Universal Command Conformance, test defs 3.1-3.17)
# ---------------------------------------------------------------------------
def _pack_ascii(text: str, nchars: int) -> bytes:
  """HART Packed-ASCII (spec): pad/truncate to nchars, keep low 6 bits of each
  char, 4 chars -> 3 bytes. Space (0x20) packs to 0x82 0x08 0x20 per 4 chars."""
  s = text.upper().ljust(nchars)[:nchars]
  v = [ord(c) & 0x3F for c in s]
  out = bytearray()
  for i in range(0, nchars, 4):
    a, b, c, d = v[i:i + 4]
    out += bytes([(a << 2) | (b >> 4), ((b & 0x0F) << 4) | (c >> 2), ((c & 0x03) << 6) | d])
  return bytes(out)


def _s(label, cmd, data=b"", long=False, master="primary",
       pre=0.3, post=0.3, note="", **extra):
  """Compact step builder (same keys the ScenarioRunner understands)."""
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
  0 / 250 -> writes must succeed (RC 0)."""
  def fn(saved):
    p = saved.get("wp")
    if p is None or len(p) < 16:
      return None, "Write-protect code unavailable (Cmd15 read failed) - RC not checked"
    code = p[15]
    if code == 1:
      return [7], ""
    if code in (0, 250):
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
    _s("[Setup] Cmd0 - learn unique ID", 0, pre=0.5,
       note="Learns the unique ID used by all long-address steps below."),
    _s("[Setup] Cmd7 - save original polling address", 7, long=True, _save="poll",
       note="Saved so it can be restored at the end of the scenario."),
    _s("[Setup] Cmd12 - save original message", 12, long=True, _save="msg",
       note="Saved for restore."),
    _s("[Setup] Cmd13 - save original tag/descriptor/date", 13, long=True, _save="tag",
       note="Saved for restore."),
    _s("[Setup] Cmd16 - save original final assembly no.", 16, long=True, _save="asm",
       note="Saved for restore."),
    _s("[Setup] Cmd20 - save original long tag", 20, long=True, _save="ltag",
       note="Saved for restore."),
    _s("[Setup] Cmd18 - write test tag/descriptor/date", 18, _WR_TAG_DATA, long=True,
       note="Writes UAL_TAG so Cmd11 can match it. Skipped effect if device is write-protected (RC=7)."),
    _s("[Setup] Cmd22 - write test long tag", 22, UAL_LONG_TAG, long=True,
       note="Writes UAL_LONG_TAG so Cmd21 can match it."),
  ]

  # ---- 3.1 UAL000: every universal command must be implemented (RC != 64) ----
  n_ual000 = len(S)
  S += [
    _s("[3.1 UAL000] Cmd0",  0,  long=True, note="UAL000: RC must NOT be 64 (command not implemented)."),
    _s("[3.1 UAL000] Cmd1",  1,  long=True, note="UAL000: RC != 64."),
    _s("[3.1 UAL000] Cmd2",  2,  long=True, note="UAL000: RC != 64."),
    _s("[3.1 UAL000] Cmd3",  3,  long=True, note="UAL000: RC != 64."),
    _s("[3.1 UAL000] Cmd6", 6, long=True, _data_from="poll",
       note="UAL000: RC != 64. Writes back the original polling address / loop current mode (no change)."),
    _s("[3.1 UAL000] Cmd7",  7,  long=True, note="UAL000: RC != 64."),
    _s("[3.1 UAL000] Cmd8",  8,  long=True, note="UAL000: RC != 64."),
    _s("[3.1 UAL000] Cmd9",  9,  bytes([0, 1, 2, 3]), long=True, note="UAL000: RC != 64."),
    _s("[3.1 UAL000] Cmd11", 11, _TAG_PACKED, long=True, note="UAL000: RC != 64."),
    _s("[3.1 UAL000] Cmd12", 12, long=True, note="UAL000: RC != 64."),
    _s("[3.1 UAL000] Cmd13", 13, long=True, note="UAL000: RC != 64."),
    _s("[3.1 UAL000] Cmd14", 14, long=True, note="UAL000: RC != 64."),
    _s("[3.1 UAL000] Cmd15", 15, long=True, note="UAL000: RC != 64."),
    _s("[3.1 UAL000] Cmd16", 16, long=True, note="UAL000: RC != 64."),
    _s("[3.1 UAL000] Cmd17", 17, _WR_MSG_DATA, long=True, note="UAL000: RC != 64 (RC 7 is fine if write-protected)."),
    _s("[3.1 UAL000] Cmd18", 18, _WR_TAG_DATA, long=True, note="UAL000: RC != 64 (RC 7 is fine if write-protected)."),
    _s("[3.1 UAL000] Cmd19", 19, UAL_ASSEMBLY, long=True, note="UAL000: RC != 64 (RC 7 is fine if write-protected)."),
    _s("[3.1 UAL000] Cmd20", 20, long=True, note="UAL000: RC != 64."),
    _s("[3.1 UAL000] Cmd21", 21, UAL_LONG_TAG, long=True, note="UAL000: RC != 64."),
    _s("[3.1 UAL000] Cmd22", 22, UAL_LONG_TAG, long=True, note="UAL000: RC != 64 (RC 7 is fine if write-protected)."),
    _s("[3.1 UAL000] Cmd38", 38, long=True, note="UAL000: RC != 64."),
    _s("[3.1 UAL000] Cmd48", 48, long=True, note="UAL000: RC != 64."),
  ]

  for st in S[n_ual000:]:
    st["reject_rc"] = [64]   # UAL000 fails if any command is "not implemented"

  # ---- 3.2 UAL001: Commands 1, 2, 3 ----
  S += [
    _s("[3.2 UAL001] Cmd1 - PV (short)",     1, note="UAL001: PV unit code + float. Expect 5 data bytes."),
    _s("[3.2 UAL001] Cmd1 - PV (long)",      1, long=True, note="Same via long address."),
    _s("[3.2 UAL001] Cmd2 - current/%range", 2, note="UAL001: loop current + percent of range (8 data bytes)."),
    _s("[3.2 UAL001] Cmd2 (long)",           2, long=True),
    _s("[3.2 UAL001] Cmd3 - dynamic vars",   3, note="UAL001: loop current + up to 4 (unit,float) pairs. Values consistent with Cmd1/Cmd2."),
    _s("[3.2 UAL001] Cmd3 (long)",           3, long=True),
  ]

  # ---- 3.3 UAL003: Commands 6 and 7 (polling address) ----
  S += [
    _s("[3.3 UAL003] Cmd7 - read loop config (before)", 7, long=True,
       note="Records current polling address + loop current mode."),
    _s("[3.3 UAL003] Cmd6 - write poll addr = %d" % UAL_POLL_TEST, 6, bytes([UAL_POLL_TEST, 1]), long=True,
       note="Long-addr write so it works regardless of current polling address."),
    _s("[3.3 UAL003] Cmd7 - confirm new address", 7, long=True, pre=0.5,
       note="Response must report the new polling address."),
    _s("[3.3 UAL003] Cmd0 short @ new addr %d" % UAL_POLL_TEST, 0, pre=0.5, _addr=UAL_POLL_TEST,
       note="Device must now answer short-frame polling at the new address."),
    _s("[3.3 UAL003] Cmd6 - restore original poll addr", 6, long=True, _data_from="poll",
       note="Restores the address saved in setup so remaining steps use the UI poll address."),
    _s("[3.3 UAL003] Cmd7 - confirm restored", 7, long=True, pre=0.5),
    _s("[3.3 UAL003] Cmd0 short @ restored addr", 0, pre=0.5,
       note="Uses the UI 'Poll addr' - must equal the device's original polling address."),
  ]

  # ---- 3.4 UAL004: Command 11 ----
  S += [
    _s("[3.4 UAL004] Cmd11 - matching tag (long)", 11, _TAG_PACKED, long=True,
       note="Response data must be identical to Cmd0 (manufacturer, type, revs, device ID)."),
    _s("[3.4 UAL004] Cmd0 - compare with Cmd11", 0, long=True,
       note="Compare identity fields with the Cmd11 result above."),
  ]

  # ---- 3.5 UAL005: Commands 12 and 17 ----
  S += [
    _s("[3.5 UAL005] Cmd12 - read message (before)", 12, note="24 data bytes (32 packed chars)."),
    _s("[3.5 UAL005] Cmd17 - write message", 17, _WR_MSG_DATA, post=0.4,
       note="Writes UAL_MESSAGE. RC=7 means device is write-protected (see 3.9)."),
    _s("[3.5 UAL005] Cmd12 - read back", 12,
       note="Must return the message just written."),
  ]

  # ---- 3.6 UAL006: Commands 13 and 18 ----
  S += [
    _s("[3.6 UAL006] Cmd13 - read tag/desc/date (before)", 13, note="21 data bytes."),
    _s("[3.6 UAL006] Cmd18 - write tag/desc/date", 18, _WR_TAG_DATA, post=0.4,
       note="Writes UAL_TAG / UAL_DESC / UAL_DATE."),
    _s("[3.6 UAL006] Cmd13 - read back", 13,
       note="Tag, descriptor and date must match the values written."),
  ]

  # ---- 3.7 UAL007: Commands 14 and 15 ----
  S += [
    _s("[3.7 UAL007] Cmd14 - PV transducer info", 14,
       note="Sensor S/N, limits units, upper/lower sensor limits, min span."),
    _s("[3.7 UAL007] Cmd14 (long)", 14, long=True),
    _s("[3.7 UAL007] Cmd15 - output info", 15,
       note="Alarm select, transfer function, range units, upper/lower range, damping, write-protect code, private-label distributor."),
    _s("[3.7 UAL007] Cmd15 (long)", 15, long=True),
  ]

  # ---- 3.8 UAL008: Commands 16 and 19 ----
  S += [
    _s("[3.8 UAL008] Cmd16 - read final assembly (before)", 16, note="3 data bytes."),
    _s("[3.8 UAL008] Cmd19 - write final assembly", 19, UAL_ASSEMBLY, post=0.4,
       note="Writes UAL_ASSEMBLY."),
    _s("[3.8 UAL008] Cmd16 - read back", 16,
       note="Must return the value just written."),
  ]

  # ---- 3.9 UAL009: Write protect ----
  S += [
    _s("[3.9 UAL009] Cmd15 - read write-protect code", 15, _save="wp",
       note="Write-protect code is Cmd15 data byte 15: 0 = no, 1 = yes, 250 = not used. If 1, every write below must return RC=7."),
    _s("[3.9 UAL009] Cmd6 write attempt",  6,  long=True, _data_from="poll", expect_rc=_wp_expect(True), note="RC=7 expected if write-protected."),
    _s("[3.9 UAL009] Cmd17 write attempt", 17, _WR_MSG_DATA, expect_rc=_wp_expect(), note="RC=7 expected if write-protected."),
    _s("[3.9 UAL009] Cmd18 write attempt", 18, _WR_TAG_DATA, expect_rc=_wp_expect(), note="RC=7 expected if write-protected."),
    _s("[3.9 UAL009] Cmd19 write attempt", 19, UAL_ASSEMBLY, expect_rc=_wp_expect(), note="RC=7 expected if write-protected."),
    _s("[3.9 UAL009] Cmd22 write attempt", 22, UAL_LONG_TAG, long=True, expect_rc=_wp_expect(), note="RC=7 expected if write-protected."),
    _s("[3.9 UAL009] Cmd0 - verify still alive", 0),
  ]

  # ---- 3.10 UAL010: Cold start bit ----
  S += [
    _s("[3.10 UAL010] Cmd0 - check cold start bit", 0, pre=0.5,
       note="Cold start (ST2 0x20) is only set after a power cycle. Power-cycle DUT and run this scenario right away for a meaningful result."),
    _s("[3.10 UAL010] Cmd0 - bit must be clear", 0,
       note="Cold start bit must be cleared after the first exchange."),
  ]

  # ---- 3.11 UAL011: Command 9 ----
  S += [
    _s("[3.11 UAL011] Cmd9 - slots 0..3", 9, bytes([0, 1, 2, 3]), note="Each variable: code, class, units, float, status. Response must echo requested codes."),
    _s("[3.11 UAL011] Cmd9 - dynamic vars PV/SV/TV/QV (246..249)", 9, bytes([246, 247, 248, 249])),
    _s("[3.11 UAL011] Cmd9 - single var", 9, bytes([0]), note="Minimal request (1 variable)."),
    _s("[3.11 UAL011] Cmd9 (long)", 9, bytes([0, 1, 2, 3]), long=True),
  ]

  # ---- 3.12 UAL012: Command 8 ----
  S += [
    _s("[3.12 UAL012] Cmd8 - dynamic variable classification", 8,
       note="4 data bytes: classification codes for PV/SV/TV/QV."),
    _s("[3.12 UAL012] Cmd8 (long)", 8, long=True),
  ]

  # ---- 3.13 UAL013: Commands 20 and 22 ----
  S += [
    _s("[3.13 UAL013] Cmd20 - read long tag (before)", 20, note="32 data bytes."),
    _s("[3.13 UAL013] Cmd22 - write long tag", 22, UAL_LONG_TAG, long=True, post=0.4,
       note="Writes UAL_LONG_TAG (32 bytes, space padded)."),
    _s("[3.13 UAL013] Cmd20 - read back", 20, note="Must match the long tag written."),
  ]

  # ---- 3.14 UAL014: Command 0 ----
  S += [
    _s("[3.14 UAL014] Cmd0 (short)", 0, pre=0.5, note="Expect 0xFE expansion code, HART revision, device IDs."),
    _s("[3.14 UAL014] Cmd0 (long)",  0, long=True, note="Must return same identity as short frame."),
    _s("[3.14 UAL014] Cmd0 (secondary master)", 0, master="secondary",
       note="Secondary-master frame must also be answered."),
  ]

  # ---- 3.15 UAL015: Command 21 ----
  S += [
    _s("[3.15 UAL015] Cmd21 - matching long tag", 21, UAL_LONG_TAG, long=True,
       note="Response data identical to Cmd0."),
    _s("[3.15 UAL015] Cmd0 - compare with Cmd21", 0, long=True),
  ]

  # ---- 3.16 UAL038: Command 38 ----
  S += [
    _s("[3.16 UAL038] Cmd0 - config-changed bit (before)", 0,
       note="After the earlier writes, ST2 'Configuration changed' (0x40) is expected to be set."),
    _s("[3.16 UAL038] Cmd38 - reset configuration changed flag", 38, long=True,
       note="HART 7 devices may expect the 2-byte config change counter; if RC=5 add it from Cmd0 bytes 14-15."),
    _s("[3.16 UAL038] Cmd0 - config-changed bit must be clear", 0, pre=0.5,
       note="'Configuration changed' bit must now be cleared."),
  ]

  # ---- 3.17 UAL048: Command 48 ----
  S += [
    _s("[3.17 UAL048] Cmd48 - additional device status", 48,
       note="Expect >= 9 status bytes (HART 6/7). Check byte 0-5 specific status, byte 6 ext device status, byte 7 op mode, byte 8 standardized status 0."),
    _s("[3.17 UAL048] Cmd48 (long)", 48, long=True),
    _s("[3.17 UAL048] Cmd0 - 'More status available' bit", 0,
       note="If ST2 0x10 is set, Cmd48 must return non-zero status."),
  ]
  # ---- Teardown: restore the original device values saved in setup ----
  S += [
    _s("[Restore] Cmd6 - original polling address", 6, long=True, _data_from="poll", post=0.4,
       note="Writes back the Cmd7 value captured in setup."),
    _s("[Restore] Cmd17 - original message", 17, long=True, _data_from="msg", post=0.4),
    _s("[Restore] Cmd18 - original tag/descriptor/date", 18, long=True, _data_from="tag", post=0.4),
    _s("[Restore] Cmd19 - original final assembly no.", 19, long=True, _data_from="asm", post=0.4),
    _s("[Restore] Cmd22 - original long tag", 22, long=True, _data_from="ltag", post=0.4),
    _s("[Restore] Cmd38 - clear config-changed flag", 38, long=True,
       note="The restore writes set 'Configuration changed' again; clear it. Add the 2-byte counter if RC=5."),
    _s("[Restore] Cmd0 - final check", 0, pre=0.5,
       note="Device should answer at its original polling address."),
  ]
  # ---- Response content checks (pass/fail) ----
  i = next(k for k, st in enumerate(S) if st["label"].startswith("[3.3 UAL003] Cmd6 - write poll addr"))
  S.insert(i, _s("[3.3 UAL003] Cmd0 - config counter (before Cmd6)", 0, long=True, _save="cnt6",
                 note="Saves the configuration change counter to see whether Cmd6 changes it."))
  _attach(S, "[3.2 UAL001] Cmd1 - PV (short)", _save="pv")
  _attach(S, "[3.3 UAL003] Cmd0 short @ new addr", check=_chk_cmd6_counter)
  _attach(S, "[3.4 UAL004] Cmd11 - matching tag", _save="id11", check=_chk_cmd0_len)
  _attach(S, "[3.4 UAL004] Cmd0 - compare with Cmd11", check=_chk_same_id("id11"))
  _attach(S, "[3.7 UAL007] Cmd15 - output info", check=_chk_range_units)
  _attach(S, "[3.11 UAL011] Cmd9 - slots 0..3", check=_chk_cmd9([0, 1, 2, 3]))
  _attach(S, "[3.11 UAL011] Cmd9 - dynamic vars", check=_chk_cmd9([246, 247, 248, 249]))
  _attach(S, "[3.11 UAL011] Cmd9 - single var", check=_chk_cmd9([0]))
  _attach(S, "[3.11 UAL011] Cmd9 (long)", check=_chk_cmd9([0, 1, 2, 3]))
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
       "expect_no_response": True,
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

  "15 - Power-On Cold Start Validation": {
    "description": (
      "Validates if slave handles cold-start flags:\n"
      "• Sends baseline short-addr Cmd0\n"
      "• Checks if 'Configuration changed' / 'Cold start' bit is set\n"
      "• Sends second Cmd0 to see if bits clear properly."
    ),
    "steps": [
      {"label": "Initial Cmd0 Discovery", "cmd": 0, "data": b"", "use_long": False, "preambles": 5, "master": "primary", "delay_pre": 0.5, "delay_post": 0.3, "note": "Check device flags byte (st2) for Cold Start bit."},
      {"label": "Verify Flag Clearing Cmd0", "cmd": 0, "data": b"", "use_long": False, "preambles": 5, "master": "primary", "delay_pre": 0.2, "delay_post": 0.2, "note": "Flag must clear on second transmission according to spec."}
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
      {"label": "Send Unimplemented Cmd 99", "cmd": 99, "data": b"0000", "use_long": False, "preambles": 5, "master": "primary", "delay_pre": 0.3, "delay_post": 0.3, "note": "Slave must respond with error code 64, not time out."}
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
}
