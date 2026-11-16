"""Update the window-title version in the GUI screenshots in this folder.

NEW title: read from setWindowTitle("...") in hart_master_simulator.py.
Only if it cannot be found there, the version given with --version is used
(title = "HART Master Simulator  v<version>").

The script lets Windows draw the new title in the native title bar of a
temporary Qt window, captures that title bar and pastes its text area over
the old title in every screenshot, so the text has the exact native font
rendering.

OLD title (what the screenshots show now): found automatically by rendering
the titles from the git history of hart_master_simulator.py until one is
pixel-identical to the screenshot. --old VERSION skips that search.

Safety checks (a file is only written if all pass):
  * The old title rendered the same way is pixel-identical to the title in
    the screenshot. This proves that the capture matches the screenshots
    (same DPI scaling, theme, active window).
  * Only the title text area changes; every other pixel stays identical.
Files are saved through png_min.encode (smallest lossless RGB PNG).

Usage:  python docs/retitle.py [--dry-run] [--version 1.2.0] [--old 1.1.9]
Needs:  PyQt5, numpy, Pillow; Windows at 100 % scaling, light title bar.
Do not touch mouse/keyboard while it runs (the window must stay active).
"""
import argparse
import glob
import os
import re
import subprocess
import sys
import tempfile

import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SRC = "hart_master_simulator.py"
TITLE_RE = re.compile(r'setWindowTitle\("([^"]+)"\)')

# Title text area in the screenshots: below the top border, right of the
# window icon, well left of the caption buttons.
Y0, Y1 = 1, 31
X0, X1 = 26, 700
# Constant part used to locate the title bar in the capture ("HART Mas...")
REF_X1 = 120

sys.path.insert(0, HERE)
from png_min import encode  # noqa: E402


TITLE_PREFIX = "HART Master Simulator  v"
# How many titles from the git history to try when looking for the old one
MAX_HISTORY_TITLES = 10


def title_for(version: str) -> str:
  return TITLE_PREFIX + version.strip().lstrip("vV")


def current_title(fallback_version: str | None) -> str:
  """Title from hart_master_simulator.py, else built from --version."""
  try:
    with open(os.path.join(ROOT, SRC), encoding="utf-8") as f:
      m = TITLE_RE.search(f.read())
  except OSError:
    m = None
  if m:
    return m.group(1)
  if fallback_version:
    print(f"setWindowTitle(\"...\") not found in {SRC}; using --version")
    return title_for(fallback_version)
  sys.exit(f"setWindowTitle(\"...\") not found in {SRC} and no --version given")


def history_titles() -> list[str]:
  """Distinct window titles from the git history of SRC, newest first."""
  try:
    log = subprocess.run(
      ["git", "log", "--format=", "-p", "-G", "setWindowTitle", "--", SRC],
      cwd=ROOT, check=True, capture_output=True, text=True, encoding="utf-8").stdout
  except (subprocess.CalledProcessError, FileNotFoundError):
    return []
  titles = []
  for t in TITLE_RE.findall(log):
    if t not in titles:
      titles.append(t)
  return titles[:MAX_HISTORY_TITLES]


def _force_foreground(hwnd: int):
  """Windows only lets the foreground process activate a window; a synthetic
  Alt key press lifts that lock so the title bar is drawn as active."""
  import ctypes
  user32, kernel32 = ctypes.windll.user32, ctypes.windll.kernel32
  VK_MENU, KEYEVENTF_KEYUP = 0x12, 0x2
  fg_thread = user32.GetWindowThreadProcessId(user32.GetForegroundWindow(), None)
  my_thread = kernel32.GetCurrentThreadId()
  attached = fg_thread and fg_thread != my_thread and \
    user32.AttachThreadInput(my_thread, fg_thread, True)
  user32.keybd_event(VK_MENU, 0, 0, 0)
  user32.keybd_event(VK_MENU, 0, KEYEVENTF_KEYUP, 0)
  user32.BringWindowToTop(hwnd)
  user32.SetForegroundWindow(hwnd)
  user32.SetActiveWindow(hwnd)
  if attached:
    user32.AttachThreadInput(my_thread, fg_thread, False)


class Renderer:
  """Shows a window with a given title and captures it from the screen."""

  def __init__(self):
    from PyQt5.QtWidgets import QApplication
    self._app = QApplication.instance() or QApplication(sys.argv[:1])

  def capture(self, title: str) -> np.ndarray:
    from PyQt5.QtCore import QEventLoop, QTimer
    from PyQt5.QtGui import QImage
    from PyQt5.QtWidgets import QWidget
    from PyQt5.QtCore import Qt
    w = QWidget()
    w.setWindowTitle(title)
    w.setWindowFlag(Qt.WindowStaysOnTopHint)
    w.setGeometry(120, 120, 1000, 120)
    w.show()
    w.raise_()
    _force_foreground(int(w.winId()))
    w.activateWindow()
    loop = QEventLoop()
    QTimer.singleShot(800, loop.quit)
    loop.exec_()
    fg = w.frameGeometry()
    pm = self._app.primaryScreen().grabWindow(
      0, fg.x() - 20, fg.y() - 20, fg.width() + 40, 80)
    w.close()
    img = pm.toImage().convertToFormat(QImage.Format_RGB888)
    ptr = img.constBits()
    ptr.setsize(img.byteCount())
    a = np.frombuffer(ptr, np.uint8).reshape(img.height(), img.bytesPerLine())
    return a[:, :img.width() * 3].reshape(img.height(), img.width(), 3).copy()


def locate(cap: np.ndarray, ref: np.ndarray):
  """Offset (dy, dx) such that cap[dy+y, dx+x] == screenshot[y, x] in the
  reference area (screenshot rows Y0..Y1, columns X0..REF_X1)."""
  rh, rw = ref.shape[:2]
  # Use the reference row with the most detail to find candidates fast
  r = int(np.argmax([len(np.unique(row.reshape(-1, 3), axis=0)) for row in ref]))
  for y in range(cap.shape[0] - rh + 1):
    win = np.lib.stride_tricks.sliding_window_view(cap[y + r], (rw, 3))[:, 0]
    for x in np.flatnonzero((win == ref[r]).all(axis=(1, 2))):
      if np.array_equal(cap[y:y + rh, x:x + rw], ref):
        return y - Y0, x - X0
  return None


def title_area(renderer, title, ref):
  cap = renderer.capture(title)
  off = locate(cap, ref)
  if off is None:
    return None
  dy, dx = off
  if dy + Y1 > cap.shape[0] or dx + X1 > cap.shape[1]:
    return None
  return cap[dy + Y0:dy + Y1, dx + X0:dx + X1]


def main():
  ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
  ap.add_argument("--version", help="new version, used only if hart_master_simulator.py "
                                    "has no setWindowTitle(\"...\")")
  ap.add_argument("--old", help="version the screenshots show now (default: found "
                                "from the git history)")
  ap.add_argument("--dry-run", action="store_true", help="check and preview, write nothing")
  args = ap.parse_args()

  new_title = current_title(args.version)
  print(f"New title: {new_title!r}")
  candidates = [title_for(args.old)] if args.old else history_titles()

  paths = sorted(glob.glob(os.path.join(HERE, "*.png")))
  shots = {p: np.asarray(Image.open(p).convert("RGB")).copy() for p in paths}
  ref = next(iter(shots.values()))[Y0:Y1, X0:REF_X1]

  renderer = Renderer()
  new_area = title_area(renderer, new_title, ref)
  if new_area is None:
    sys.exit("Could not find the title bar in the capture (scaling/theme differs, "
             "or the window was not active). Nothing written.")

  rendered = {}   # title -> captured text area, rendered on demand

  def old_title_of(area):
    for t in candidates:
      if t not in rendered:
        rendered[t] = title_area(renderer, t, ref)
      if rendered[t] is not None and np.array_equal(area, rendered[t]):
        return t
    return None

  failed = 0
  for p in paths:
    name = os.path.basename(p)
    a = shots[p]
    area = a[Y0:Y1, X0:X1]
    if np.array_equal(area, new_area):
      print(f"{name:<52} already shows the new title")
      continue
    old = old_title_of(area)
    if old is None:
      print(f"{name:<52} old title not recognised ({len(candidates)} tried) - SKIPPED")
      failed += 1
      continue
    b = a.copy()
    b[Y0:Y1, X0:X1] = new_area
    mask = np.ones(a.shape[:2], bool)
    mask[Y0:Y1, X0:X1] = False
    assert np.array_equal(a[mask], b[mask])
    if args.dry_run:
      print(f"{name:<52} would be updated (was {old.rsplit(' ', 1)[-1]})")
      continue
    tmp = p + ".tmp.png"
    Image.fromarray(b).save(tmp)
    png, _ = encode(tmp)
    with open(tmp, "wb") as f:
      f.write(png)
    if not np.array_equal(np.asarray(Image.open(tmp).convert("RGB")), b):
      os.remove(tmp)
      print(f"{name:<52} re-encode mismatch - NOT replaced")
      failed += 1
      continue
    os.replace(tmp, p)
    print(f"{name:<52} updated (was {old.rsplit(' ', 1)[-1]})")

  preview = os.path.join(tempfile.mkdtemp(prefix="retitle_"), "new_title.png")
  Image.fromarray(new_area).resize(((X1 - X0) * 2, (Y1 - Y0) * 2), Image.NEAREST).save(preview)
  print(f"Preview of the new title: {preview}")
  return 1 if failed else 0


if __name__ == "__main__":
  sys.exit(main())
