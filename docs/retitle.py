"""Update the window-title version in the GUI screenshots in this folder.

Reads the title from setWindowTitle(...) in hart_master_simulator.py, lets
Windows draw it in the native title bar of a temporary Qt window, captures
that title bar and pastes its text area over the old title in every
screenshot. The text therefore has exactly the native font rendering.

Safety checks (a file is only written if all pass):
  * The OLD title (from git HEAD, or --old) rendered the same way must be
    pixel-identical to the title in the screenshot. This proves that the
    capture matches the screenshots (same DPI scaling, theme, active window).
  * Only the title text area changes; every other pixel stays identical.
Files are saved through png_min.encode (smallest lossless RGB PNG).

Usage:  python docs/retitle.py [--old "HART Master Simulator  v1.1.9"] [--dry-run]
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


def title_from(text: str) -> str:
  m = TITLE_RE.search(text)
  if not m:
    sys.exit(f"setWindowTitle(\"...\") not found in {SRC}")
  return m.group(1)


def head_title() -> str | None:
  try:
    text = subprocess.run(["git", "show", f"HEAD:{SRC}"], cwd=ROOT, check=True,
                          capture_output=True, text=True, encoding="utf-8").stdout
  except (subprocess.CalledProcessError, FileNotFoundError):
    return None
  return title_from(text)


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
  ap.add_argument("--old", help="title currently in the screenshots (default: from git HEAD)")
  ap.add_argument("--dry-run", action="store_true", help="check and preview, write nothing")
  args = ap.parse_args()

  with open(os.path.join(ROOT, SRC), encoding="utf-8") as f:
    new_title = title_from(f.read())
  old_title = args.old or head_title()
  print(f"New title: {new_title!r}")
  print(f"Old title: {old_title!r}")

  paths = sorted(glob.glob(os.path.join(HERE, "*.png")))
  shots = {p: np.asarray(Image.open(p).convert("RGB")).copy() for p in paths}
  ref = next(iter(shots.values()))[Y0:Y1, X0:REF_X1]

  renderer = Renderer()
  new_area = title_area(renderer, new_title, ref)
  old_area = title_area(renderer, old_title, ref) if old_title else None
  if new_area is None:
    sys.exit("Could not find the title bar in the capture (scaling/theme differs, "
             "or the window was not active). Nothing written.")

  preview_dir = tempfile.mkdtemp(prefix="retitle_")
  Image.fromarray(np.vstack([old_area if old_area is not None else new_area, new_area])) \
    .resize(((X1 - X0) * 2, (Y1 - Y0) * 4), Image.NEAREST) \
    .save(os.path.join(preview_dir, "old_vs_new_title.png"))

  failed = 0
  for p in paths:
    name = os.path.basename(p)
    a = shots[p]
    area = a[Y0:Y1, X0:X1]
    if np.array_equal(area, new_area):
      print(f"{name:<52} already shows the new title")
      continue
    if old_area is None or not np.array_equal(area, old_area):
      print(f"{name:<52} title does not match the rendered old title - SKIPPED")
      failed += 1
      continue
    b = a.copy()
    b[Y0:Y1, X0:X1] = new_area
    mask = np.ones(a.shape[:2], bool)
    mask[Y0:Y1, X0:X1] = False
    assert np.array_equal(a[mask], b[mask])
    if args.dry_run:
      print(f"{name:<52} would be updated")
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
    print(f"{name:<52} updated")

  print(f"Preview: {os.path.join(preview_dir, 'old_vs_new_title.png')}")
  return 1 if failed else 0


if __name__ == "__main__":
  sys.exit(main())
