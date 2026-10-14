import sys, os
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

os.environ["QT_QPA_PLATFORM"] = "offscreen"
os.environ["QTWEBENGINE_DISABLE_SANDBOX"] = "1"

import time
import uav_gpr_qt as air
from PySide6 import QtWidgets

app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

win = air.MainWindow()
win.remote_enable.setChecked(False)
win.count.setValue(0)
win._device_source = air.DEVICE_SOURCE_SIMULATED

finished_calls = []
def hook():
    print(f"--> HOOK CALLED: sender={win.sender()}, win.worker={win.worker}")
    finished_calls.append(True)
    orig_on_worker_finished()

orig_on_worker_finished = win.on_worker_finished
win.on_worker_finished = hook

win.start_acquisition()
# 让它采 1 秒
t0 = time.time()
while time.time() - t0 < 1.0:
    app.processEvents()
    time.sleep(0.05)

print("Stopping...")
win.stop_acquisition()

t0 = time.time()
while time.time() - t0 < 5.0:
    app.processEvents()
    time.sleep(0.05)
    if win.worker is None:
        break

print("Finished calls count:", len(finished_calls))
print("win.worker is now:", win.worker)
win.close()