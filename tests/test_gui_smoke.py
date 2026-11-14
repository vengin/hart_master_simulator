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
def window(app, monkeypatch):
  monkeypatch.setattr(sim.time, "sleep", lambda s: None)
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
  w._cb_scenario.setCurrentIndex(w._cb_scenario.findText(name))
  w._run_scenario()
  deadline = time.time() + 30
  while w._scenario_runner.isRunning() or not w._btn_scenario_run.isEnabled():
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
