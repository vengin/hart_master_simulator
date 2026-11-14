"""Off-screen smoke test of the GUI scenario path: the main window runs real
scenarios on its QThread runner against FakeSlave, and the result table must
show the runner's verdict for every step."""
import os
import sys
import time

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

from PyQt5.QtWidgets import QApplication  # noqa: E402

import hart_master_simulator as sim  # noqa: E402
from scenarios import SCENARIOS  # noqa: E402
from fake_slave import FakeSlave, UNIQUE_ADDR  # noqa: E402


@pytest.fixture(scope="module")
def app():
  return QApplication.instance() or QApplication([])


@pytest.fixture
def window(app, monkeypatch, tmp_path):
  monkeypatch.setattr(sim.time, "sleep", lambda s: None)
  monkeypatch.setattr(sim.HartMasterSim, "SETTINGS_PATH", str(tmp_path / "settings.json"))
  monkeypatch.setattr(sim.ScenarioRunner, "RESPONSE_TIMEOUT_S", 0.005)
  w = sim.HartMasterSim()
  yield w
  w._worker._port = None
  w._worker.stop()
  w.close()


def _run_in_gui(app, w, name, slave):
  slave.is_open = True
  w._worker._port = slave
  w._connected = True
  w._set_all_scenarios_checked(False)
  w._scn_list.item(w._scn_rows[name], 0).setCheckState(sim.Qt.Checked)
  w._run_selected_scenarios()
  deadline = time.time() + 30
  while w._run_all_active or w._scenario_runner.isRunning():
    app.processEvents()
    if time.time() > deadline:
      raise AssertionError("scenario did not finish")
  app.processEvents()
  return [w._scenario_table.item(i, 2).text() for i in range(len(SCENARIOS[name]["steps"]))]


@pytest.mark.parametrize("prefix", ["11", "12", "14"])
def test_gui_runs_scenario_and_shows_results(app, window, prefix):
  name = next(n for n in SCENARIOS if n.startswith(prefix + " - "))
  texts = _run_in_gui(app, window, name, FakeSlave())
  assert all(t.startswith("✓") for t in texts), texts
  assert window._lbl_scenario_summary.text().startswith("Done: %d/%d OK" % (len(texts), len(texts)))
  assert window._unique_id == UNIQUE_ADDR       # learned through the uid_learned signal


def test_gui_shows_failure_reason(app, window):
  name = next(n for n in SCENARIOS if n.startswith("12 - "))
  texts = _run_in_gui(app, window, name, FakeSlave(faults={"shared_config_flag"}))
  bad = [t for t in texts if t.startswith("✗")]
  assert bad and any("expected" in t for t in bad), texts


def test_gui_common_practice_tab_sends_encoded_requests(app, window, monkeypatch):
  from hart_cp import CP_COMMANDS
  sent, logged = [], []
  monkeypatch.setattr(window, "_send_command", lambda c, d=b"", l="": sent.append((c, d)))
  monkeypatch.setattr(window, "_log_info", lambda m, *a, **k: logged.append(m))
  assert set(window._cp_inputs) == {c for c, l, k, d, t in CP_COMMANDS if k != "none"}
  window._cp_inputs[33].setText("0 246")
  window._send_cp_command(33, "Cmd33")
  window._send_cp_command(41, "Cmd41")
  window._cp_inputs[49].setText("not a number")
  window._send_cp_command(49, "Cmd49")
  assert sent == [(33, bytes([0, 246])), (41, b"")]
  assert any("bad request data" in m for m in logged)


def _run_all_in_gui(app, w, slave):
  slave.is_open = True
  w._worker._port = slave
  w._connected = True
  w._run_all_scenarios()
  deadline = time.time() + 120
  while w._run_all_active:
    app.processEvents()
    if time.time() > deadline:
      raise AssertionError("Run All did not finish")
  app.processEvents()
  table = w._scn_list
  return {table.item(r, 0).text(): table.item(r, 1).text() for r in range(table.rowCount())}


def test_gui_run_all_shows_overall_and_per_scenario_verdict(app, window, monkeypatch):
  monkeypatch.setattr(sim.HartMasterSim, "RUN_ALL_GAP_MS", 0)
  rows = _run_all_in_gui(app, window, FakeSlave())
  auto = [n for n in SCENARIOS if not SCENARIOS[n].get("manual_only")]
  manual = [n for n in SCENARIOS if SCENARIOS[n].get("manual_only")]
  assert set(rows) == set(SCENARIOS)
  assert all(rows[n] in ("✓ PASS", "✗ FAIL") for n in auto), rows
  assert all(rows[n].startswith("SKIPPED") for n in manual), rows
  overall = window._lbl_run_all_summary.text()
  n_fail = sum(rows[n] == "✗ FAIL" for n in auto)
  if n_fail:
    assert overall.startswith("RUN ALL: FAIL - %d/%d" % (n_fail, len(auto))), overall
  else:
    assert overall == "RUN ALL: PASS - %d/%d scenarios passed" % (len(auto), len(auto))
  assert window._btn_scenario_run.isEnabled() and window._btn_scenario_run_all.isEnabled()


def test_gui_run_all_reports_fail_overall(app, window, monkeypatch):
  monkeypatch.setattr(sim.HartMasterSim, "RUN_ALL_GAP_MS", 0)
  rows = _run_all_in_gui(app, window, FakeSlave(faults={"shared_config_flag"}))
  name12 = next(n for n in SCENARIOS if n.startswith("12 - "))
  assert rows[name12] == "✗ FAIL"
  assert window._lbl_run_all_summary.text().startswith("RUN ALL: FAIL")


def test_gui_run_selected_runs_only_checked_scenarios(app, window, monkeypatch):
  monkeypatch.setattr(sim.HartMasterSim, "RUN_ALL_GAP_MS", 0)
  picked = [n for n in SCENARIOS if n.split(" - ")[0] in ("2", "5", "11")]
  assert len(picked) == 3
  window._set_all_scenarios_checked(False)
  for n in picked:
    window._scn_list.item(window._scn_rows[n], 0).setCheckState(sim.Qt.Checked)
  slave = FakeSlave()
  slave.is_open = True
  window._worker._port = slave
  window._connected = True
  window._run_selected_scenarios()
  seen = {window._lbl_run_all_summary.text()}
  deadline = time.time() + 60
  while window._run_all_active:
    app.processEvents()
    seen.add(window._lbl_run_all_summary.text())
    assert time.time() < deadline, "Run Selected did not finish"
  assert "RUN SELECTED: Scenario 2; running 1/3 - 0 PASS, 0 FAIL" in seen, seen
  assert "RUN SELECTED: Scenario 11; running 3/3 - 2 PASS, 0 FAIL" in seen, seen
  assert all("Scenario" in t for t in seen if "running" in t), seen
  app.processEvents()
  results = {n: window._scn_list.item(r, 1).text() for n, r in window._scn_rows.items()}
  assert all(results[n] == "✓ PASS" for n in picked), results
  assert all(results[n] == "" for n in SCENARIOS if n not in picked), results
  assert window._lbl_run_all_summary.text() == "RUN SELECTED: PASS - 3/3 scenarios passed"


def test_gui_checked_scenarios_are_remembered(app, window):
  names = list(SCENARIOS)
  window._set_all_scenarios_checked(False)
  for n in (names[1], names[4]):
    window._scn_list.item(window._scn_rows[n], 0).setCheckState(sim.Qt.Checked)
  w2 = sim.HartMasterSim()          # same SETTINGS_PATH (fixture monkeypatch)
  try:
    assert w2._checked_scenarios() == [names[1], names[4]]
    assert w2._lbl_scn_checked.text() == "2/%d checked" % len(names)
  finally:
    w2._worker.stop()
    w2.close()
