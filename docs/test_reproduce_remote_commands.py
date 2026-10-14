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

print("1. 模拟收到远程命令: start")
win.handle_remote_command("start", {})

t0 = time.time()
while time.time() - t0 < 1.0:
    app.processEvents()
    time.sleep(0.05)

print(f"采集中状态: worker={win.worker}, isRunning={win.worker.isRunning() if win.worker else None}, service.state={win.acquisition_service.state}")

print("2. 模拟收到远程命令: stop")
win.handle_remote_command("stop", {})

# 等待 5 秒
t0 = time.time()
while time.time() - t0 < 5.0:
    app.processEvents()
    time.sleep(0.05)
    if win.worker is None:
        break

print(f"停止后状态: worker={win.worker}, isRunning={win.worker.isRunning() if win.worker else None}, service.state={win.acquisition_service.state}")

print("3. 模拟再次收到远程命令: start")
win.handle_remote_command("start", {})

t0 = time.time()
while time.time() - t0 < 1.0:
    app.processEvents()
    time.sleep(0.05)

print(f"第2次采集状态: worker={win.worker}, isRunning={win.worker.isRunning() if win.worker else None}, service.state={win.acquisition_service.state}")

win.close()