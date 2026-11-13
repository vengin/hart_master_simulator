"""Offline tests for the HART master simulator (no serial port, no GUI window).

- protocol helpers: frame builders, parse_response, decode_cmd0, packed ASCII
- evaluate_step: the single pass/fail decision used by ScenarioRunner
- every scenario runs against FakeSlave (a model of the BUOS HART 7 device);
  the configuration/NV scenarios must pass completely, and must FAIL against a
  FakeSlave with the defect each scenario is meant to detect.

Run:  python -m pytest -q tests
"""
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

import hart_master_simulator as sim  # noqa: E402
from hart_codec import encode_packed_ascii, decode_packed_ascii  # noqa: E402
from scenarios import SCENARIOS  # noqa: E402
from fake_slave import FakeSlave, UNIQUE_ADDR  # noqa: E402

# The QApplication is created in conftest.py


@pytest.fixture(autouse=True)
def no_delays(monkeypatch):
  monkeypatch.setattr(sim.time, "sleep", lambda s: None)
  monkeypatch.setattr(sim.ScenarioRunner, "RESPONSE_TIMEOUT_S", 0.005)


def scenario(prefix):
  names = [n for n in SCENARIOS if n.startswith(prefix + " - ")]
  assert len(names) == 1, prefix
  return names[0]


def run_scenario(name, slave, poll_addr=0, on_manual=None):
  """Run a scenario synchronously; returns (ok, fail, [(idx, label, ok, reason)])."""
  steps = SCENARIOS[name]["steps"]
  runner = sim.ScenarioRunner(port=slave, steps=list(steps), unique_id=None, poll_addr=poll_addr)
  results, done = [], []
  runner.step_result.connect(lambda i, ok, r: results.append((i, steps[i]["label"], ok, r)))
  runner.step_error.connect(lambda i, lbl, msg: results.append((i, lbl, False, "IO error: " + msg)))
  runner.scenario_done.connect(lambda ok, fail: done.append((ok, fail)))

  def manual(idx, text):
    if on_manual:
      on_manual(slave)
    runner.confirm_manual(True)
  runner.manual_prompt.connect(manual)
  runner.run()
  assert done, "scenario_done not emitted"
  return done[0][0], done[0][1], results


def failures(results):
  return [(i, lbl, r) for i, lbl, ok, r in results if not ok]


# ---------------------------------------------------------------------------
# Protocol helpers
# ---------------------------------------------------------------------------

def test_short_frame_master_bit_in_address_not_delimiter():
  p = sim.build_short_frame(5, 0, b"", 5, "primary")
  s = sim.build_short_frame(5, 0, b"", 5, "secondary")
  assert p[5] == 0x02 and s[5] == 0x02          # STX for both masters
  assert p[6] == 0x80 | 5 and s[6] == 5         # bit 7 = primary
  assert p[-1] == sim.calc_checksum(p[5:-1])


def test_long_frame_uses_6_lsbs_and_never_sets_burst_bit():
  f = sim.build_long_frame(bytes([0xE7, 0x8B, 1, 2, 3]), 0, b"", 5, "secondary")
  assert f[5] == 0x82
  assert f[6] == 0x27                           # 0xE7 & 0x3F, secondary, burst bit 0
  f = sim.build_long_frame(bytes([0x27, 0x8B, 1, 2, 3]), 0, b"", 5, "primary")
  assert f[6] == 0xA7


def test_decode_cmd0_hart7_fields_and_long_address():
  payload = FakeSlave()._c0_data()
  d = sim.decode_cmd0(payload)
  assert d["_unique_id"] == UNIQUE_ADDR == bytes([0x27, 0x8B, 0x01, 0x23, 0x45])
  assert d["Expanded Device Type"] == "0xE78B"
  assert d["Manufacturer ID"].startswith("0x61F6")
  assert d["Last Device Variable code"] == 0 and d["Device Profile"] == 1


def test_parse_response_reads_master_and_burst_bits_from_address():
  out = bytes([0x06, 0x80 | 0x40 | 3, 1, 2, 0, 0])
  raw = bytes([0xFF] * 5) + out + bytes([sim.calc_checksum(out)])
  p = sim.parse_response(raw)
  assert p["is_ack"] and p["is_primary"] and p["is_burst"] and p["cs_ok"]
  assert (p["address"][0] & 0x3F) == 3


def test_parse_response_skips_echo_of_own_request():
  req = sim.build_short_frame(0, 1, b"", 5, "primary")
  out = bytes([0x06, 0x80, 1, 2, 0, 0])
  raw = req + bytes([0xFF] * 5) + out + bytes([sim.calc_checksum(out)])
  p = sim.parse_response(raw)
  assert p["is_ack"] and p["cmd"] == 1 and p["cs_ok"]


def test_comm_error_bits():
  out = bytes([0x06, 0x80, 1, 2, 0x80 | 0x10 | 0x08, 0])
  p = sim.parse_response(bytes([0xFF] * 5) + out + bytes([sim.calc_checksum(out)]))
  assert p["comm_error"]
  assert "Framing error" in p["status_text"] and "Longitudinal parity error" in p["status_text"]


def test_packed_ascii_matches_hart_rule():
  assert encode_packed_ascii("TAG1", 6).hex() == "5011f1820820"
  assert decode_packed_ascii(bytes.fromhex("5011f1820820")) == "TAG1    "
  assert sim._encode_packed_ascii("TAG1") == encode_packed_ascii("TAG1", 6)


# ---------------------------------------------------------------------------
# evaluate_step
# ---------------------------------------------------------------------------

def _resp(cmd=0, rc=0, st=0, data=b"", addr=0x80, delim=0x06):
  out = bytes([delim, addr, cmd, len(data) + 2, rc, st]) + data
  raw = bytes([0xFF] * 5) + out + bytes([sim.calc_checksum(out)])
  return raw, sim.parse_response(raw)


def ev(step, raw, parsed, saved=None, poll=0):
  return sim.evaluate_step(step, raw, parsed, poll, saved or {})


def test_eval_rc_and_status_checks():
  raw, p = _resp(rc=5, st=0x40)
  assert not ev({"cmd": 0, "expect_rc": 0}, raw, p)[0]
  assert ev({"cmd": 0, "expect_rc": [0, 5]}, raw, p)[0]
  assert ev({"cmd": 0, "expect_status_set": 0x40}, raw, p)[0]
  assert not ev({"cmd": 0, "expect_status_set": 0x20}, raw, p)[0]
  assert not ev({"cmd": 0, "expect_status_clear": 0x40}, raw, p)[0]


def test_eval_rejects_bad_checksum_wrong_cmd_and_wrong_master():
  raw, p = _resp()
  bad = raw[:-1] + bytes([raw[-1] ^ 1])
  assert not ev({"cmd": 0}, bad, sim.parse_response(bad))[0]
  assert not ev({"cmd": 1}, raw, p)[0]
  assert not ev({"cmd": 0, "master": "secondary"}, raw, p)[0]


def test_eval_multidrop_answer_at_wrong_address_fails():
  raw, p = _resp(addr=0x80 | 3)
  ok, reason = ev({"cmd": 0, "_addr": 3}, raw, p, poll=0)
  assert not ok and "answered" in reason
  assert ev({"cmd": 0, "_addr": 3}, b"", None, poll=0)[0]          # silence is right
  assert not ev({"cmd": 0, "_addr": 3, "expect_rc": 0}, b"", None, poll=0)[0]  # strict


def test_eval_payload_and_u16_refs():
  raw, p = _resp(data=bytes(14) + b"\x00\x07")
  saved = {"before": bytes(14) + b"\x00\x05"}
  assert ev({"cmd": 0, "expect_u16": {"offset": 14, "ref": "before", "delta": 2}}, raw, p, saved)[0]
  assert not ev({"cmd": 0, "expect_u16": {"offset": 14, "ref": "before", "delta": 1}}, raw, p, saved)[0]
  assert ev({"cmd": 0, "expect_payload": b"\x00\x07", "payload_slice": [14, 16]}, raw, p)[0]
  assert ev({"cmd": 0, "expect_payload": ("before", 0, 14), "payload_slice": [0, 14]}, raw, p, saved)[0]
  ok, reason = ev({"cmd": 0, "expect_payload": "missing"}, raw, p)
  assert not ok and "missing" in reason


def test_extract_reply_strips_echo_of_request():
  req = sim.build_short_frame(0, 1, b"", 5, "primary")
  raw, _ = _resp(cmd=1)
  assert sim.extract_reply(req + raw, req) == raw                      # exact echo
  assert sim.extract_reply(b"\xff\xff" + req.lstrip(b"\xff") + raw, req) == raw  # fewer preambles
  assert sim.extract_reply(req, req) == b""                             # echo only = silence
  assert sim.extract_reply(b"\xff\xff\xff", req) == b""
  assert sim.extract_reply(raw, req) == raw                             # no echo


def test_parse_response_without_preambles():
  raw, _ = _resp(cmd=1)
  p = sim.parse_response(raw.lstrip(b"\xff"))
  assert p["ok"] and p["cmd"] == 1 and p["cs_ok"]


def test_runner_data_u16_builds_counter_plus_delta():
  slave = FakeSlave()
  slave.ram["ccc"] = 0xFFFF
  steps = [
    {"label": "c0", "cmd": 0, "use_long": False, "expect_rc": 0, "save_as": "c0"},
    {"label": "c38", "cmd": 38, "use_long": True, "expect_rc": 9,
     "data_u16": {"ref": "c0", "offset": 14, "delta": 1}},
  ]
  runner = sim.ScenarioRunner(port=slave, steps=steps, unique_id=None, poll_addr=0)
  sent = []
  runner.step_done.connect(lambda i, lbl, tx, rx, lat: sent.append(tx))
  runner.run()
  c38 = sent[1].lstrip(b"\xff")
  assert c38[8:10] == b"\x00\x00"     # 0xFFFF + 1 wraps to 0x0000 (never matches 0xFFFF)


def test_eval_expect_no_response():
  raw, p = _resp()
  assert ev({"cmd": 0, "expect_no_response": True}, b"", None)[0]
  assert not ev({"cmd": 0, "expect_no_response": True}, raw, p)[0]


# ---------------------------------------------------------------------------
# Scenarios against a conformant device model
# ---------------------------------------------------------------------------

STRICT = ["11", "12", "14", "15", "16", "17", "18", "19", "20", "21"]


@pytest.mark.parametrize("prefix", STRICT)
def test_strict_scenario_passes_on_conformant_device(prefix):
  slave = FakeSlave()
  ok, fail, results = run_scenario(scenario(prefix), slave,
                                   on_manual=lambda s: s.reset(power_cycle=True))
  assert fail == 0, failures(results)
  assert ok == len(SCENARIOS[scenario(prefix)]["steps"])


@pytest.mark.parametrize("prefix", ["11", "12", "18", "19", "20", "21"])
def test_scenario_restores_device_configuration(prefix):
  slave = FakeSlave()
  before = dict(slave.ram)
  run_scenario(scenario(prefix), slave, on_manual=lambda s: s.reset(power_cycle=True))
  after = dict(slave.ram)
  for key in ("poll", "loop", "tag", "desc", "date", "msg", "fan", "long_tag"):
    assert after[key] == before[key], key
  assert after["ccflag"] == 0      # both masters' Config Changed cleared again


@pytest.mark.parametrize("echo", [True, "short"])
@pytest.mark.parametrize("prefix", STRICT)
def test_strict_scenario_passes_through_echoing_modem(prefix, echo):
  # Half-duplex modem echoing our request: silence checks must still work
  slave = FakeSlave(echo=echo)
  ok, fail, results = run_scenario(scenario(prefix), slave,
                                   on_manual=lambda s: s.reset(power_cycle=True))
  assert fail == 0, failures(results)


# ---------------------------------------------------------------------------
# Scenario 17 (Universal Command Conformance, upstream v1.1.x step keys:
# reject_rc, callable expect_rc, check, _save, _data_from)
# ---------------------------------------------------------------------------

def _ual(**kw):
  slave = FakeSlave(**kw)
  ok, fail, results = run_scenario(scenario("17"), slave)
  return slave, fail, results


def test_ual_write_protected_device_rc7_checks_pass_and_is_restored():
  slave = FakeSlave(write_protect=True)
  before = dict(slave.ram)
  ok, fail, results = run_scenario(scenario("17"), slave)
  failed = [lbl for i, lbl, r in failures(results)]
  # Known scenario limitation: setup cannot write the test tag / long tag /
  # polling address on a write-protected device, so Cmd11/21 (tag search) and
  # the short poll at the new address correctly get no answer.
  assert failed == ["[3.1 UAL000] Cmd11", "[3.1 UAL000] Cmd21",
                    "[3.3 UAL003] Cmd0 short @ new addr 1",
                    "[3.4 UAL004] Cmd11 - matching tag (long)",
                    "[3.15 UAL015] Cmd21 - matching long tag"], failed
  assert all(ok for i, lbl, ok, r in results if lbl.startswith("[3.9 UAL009]"))
  for key in ("poll", "tag", "desc", "date", "msg", "fan", "long_tag"):
    assert slave.ram[key] == before[key], key


def test_ual_restores_device_configuration():
  slave = FakeSlave()
  before = dict(slave.ram)
  run_scenario(scenario("17"), slave)
  for key in ("poll", "loop", "tag", "desc", "date", "msg", "fan", "long_tag"):
    assert slave.ram[key] == before[key], key


def test_ual_write_protect_off_is_reported_as_info():
  _, fail, results = _ual()
  assert fail == 0
  infos = [r for i, lbl, ok, r in results if ok and sim.INFO_SEP in r]
  assert any("RC 7 behaviour NOT tested" in r for r in infos), infos


@pytest.mark.parametrize("kw, step", [
  ({"faults": {"missing_cmd48"}}, "[3.1 UAL000] Cmd48"),
  ({"write_protect": True, "faults": {"wp_not_enforced"}}, "[3.9 UAL009] Cmd17 write attempt"),
  ({"faults": {"cmd9_no_timestamp_pad"}}, "[3.11 UAL011] Cmd9 - slots 0..3"),
])
def test_ual_detects_fault(kw, step):
  _, fail, results = _ual(**kw)
  assert step in [lbl for i, lbl, r in failures(results)], failures(results)


def test_ual_cmd9_rc14_warning_accepted_but_errors_fail():
  # RC 14 (Dynamic Variables Returned for Device Variables) is a warning with
  # full data, as the BUOS firmware answers Cmd9 for codes 0-3
  _, fail, results = _ual()
  assert fail == 0
  assert any("RC 14" in r for i, lbl, ok, r in results if lbl.startswith("[3.11 UAL011]"))
  step = next(st for st in SCENARIOS[scenario("17")]["steps"]
              if st["label"].startswith("[3.11 UAL011] Cmd9 - slots"))
  raw, parsed = _resp(cmd=9, rc=2)            # an error code must still fail
  assert not sim.evaluate_step(step, raw, parsed, 0, {})[0]


def test_ual_missing_saved_value_fails_the_data_from_step():
  # Cmd7 (poll addr) read fails -> every step sending _data_from="poll" must fail,
  # not send stale/empty data
  slave = FakeSlave()
  steps = [st for st in SCENARIOS[scenario("17")]["steps"]
           if not st["label"].startswith("[Setup] Cmd7")]
  runner = sim.ScenarioRunner(port=slave, steps=steps, unique_id=None, poll_addr=0)
  res = []
  runner.step_result.connect(lambda i, ok, r: res.append((steps[i], ok, r)))
  runner.run()
  dep = [(ok, r) for st, ok, r in res if st.get("_data_from") == "poll"]
  assert dep and all(not ok and "no saved value 'poll'" in r for ok, r in dep), dep


@pytest.mark.parametrize("name", list(SCENARIOS))
def test_every_scenario_runs_without_io_errors(name):
  ok, fail, results = run_scenario(name, FakeSlave(),
                                   on_manual=lambda s: s.reset(power_cycle=True))
  assert ok + fail == len(SCENARIOS[name]["steps"])
  assert not [r for r in results if r[3].startswith("IO error")]


def test_reset_scenarios_really_reset_the_device():
  for prefix in ("15", "18", "19", "20"):
    slave = FakeSlave()
    run_scenario(scenario(prefix), slave)
    assert slave.resets == 1, prefix


# ---------------------------------------------------------------------------
# Scenarios must detect the defects they target
# ---------------------------------------------------------------------------

DETECTS = [
  ("18", "no_persistence"),
  ("18", "shared_config_flag"),
  ("18", "c38_clears_both"),
  ("18", "no_cold_start"),
  ("21", "no_persistence"),
  ("20", "reset_drops_last_save"),
  ("19", "no_c6_persistence"),
  ("19", "no_persistence"),
  ("12", "shared_config_flag"),
  ("12", "c38_clears_both"),
  ("12", "c38_no_counter_check"),
  ("15", "shared_cold_start"),
  ("15", "no_cold_start"),
  ("14", "broadcast_any_cmd"),
  ("11", "counter_on_rc5"),
  ("3", "wrong_master_bit"),
  ("12", "wrong_master_bit"),
]


@pytest.mark.parametrize("prefix,fault", DETECTS)
def test_scenario_detects_defect(prefix, fault):
  slave = FakeSlave(faults={fault})
  ok, fail, results = run_scenario(scenario(prefix), slave,
                                   on_manual=lambda s: s.reset(power_cycle=True))
  assert fail > 0, f"scenario {prefix} did not detect '{fault}'"
