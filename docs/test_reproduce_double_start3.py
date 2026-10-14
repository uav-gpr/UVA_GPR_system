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
# 不等待完成，直接立即再次调用 start_acquisition()
print("立即再次 start_acquisition():")
win.start_acquisition()

print("Logs:")
for l in logs[-5:]:
    print(" ", l)

win.close()