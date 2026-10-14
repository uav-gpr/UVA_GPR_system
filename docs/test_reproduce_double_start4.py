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

logs = []
win.log = lambda msg: logs.append(msg)

win.start_acquisition()
t0 = time.time()
while time.time() - t0 < 1.0:
    app.processEvents()
    time.sleep(0.05)

win.stop_acquisition()

# 模拟等了 5 秒，直到全部事件都跑完
t0 = time.time()
while time.time() - t0 < 5.0:
    app.processEvents()
    time.sleep(0.05)
    if win.worker is None:
        break

print(f"5秒后: win.worker={win.worker}")
print("再次调用 start_acquisition():")
win.start_acquisition()

print("最新的几条日志:")
for l in logs[-5:]:
    print(" ", l)

win.close()