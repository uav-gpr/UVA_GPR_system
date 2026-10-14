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

print("1. 启动第一次采集...")
win.start_acquisition()
time.sleep(0.5)
app.processEvents()

print("2. 发送停止命令...")
win.stop_acquisition()

# 模拟用户在还未完全彻底退出的瞬间点击第二次启动
print("3. 模拟立即再次点击开始采集（带自愈收割）...")
win.start_acquisition()

print("当前日志最后 6 条:")
for l in logs[-6:]:
    print(" ", l)

print(f"最终状态: worker={win.worker}, isRunning={win.worker.isRunning() if win.worker else None}")
win.close()