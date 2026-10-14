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
win.remote_enable.setChecked(False) # 本地测试不连远端
win.count.setValue(0) # 无限循环采集
# 设为模拟设备避免无硬件报错
win._device_source = air.DEVICE_SOURCE_SIMULATED

print("=== 模拟点击启动第 1 次采集 ===")
win.start_acquisition()
print(f"刚启动: win.worker={win.worker}, isRunning={win.worker.isRunning() if win.worker else None}, service.state={win.acquisition_service.state}")

# 让它采 2 秒钟
t0 = time.time()
while time.time() - t0 < 2.0:
    app.processEvents()
    time.sleep(0.05)

print(f"运行2秒后: selected_count={win.worker._selected_count if win.worker else None}")

print("=== 模拟点击停止采集 ===")
win.stop_acquisition()
print(f"点击停止后立即状态: service.state={win.acquisition_service.state}, worker.isRunning={win.worker.isRunning() if win.worker else None}")

# 等待 10 秒看 worker 是否能结束
t0 = time.time()
while time.time() - t0 < 10.0:
    app.processEvents()
    time.sleep(0.1)
    if win.worker is None:
        print(f"-> 成功检测到 win.worker 变为了 None！耗时 {time.time()-t0:.2f}s")
        break
    if not win.worker.isRunning():
        print(f"-> worker.isRunning() 变为了 False，但 win.worker 仍是 {win.worker}")
        break

print(f"停止后最终状态: win.worker={win.worker}, isRunning={win.worker.isRunning() if win.worker else None}, service.state={win.acquisition_service.state}")

print("=== 模拟再次点击启动第 2 次采集 ===")
win.start_acquisition()
print(f"第2次启动后: win.worker={win.worker}, isRunning={win.worker.isRunning() if win.worker else None}, service.state={win.acquisition_service.state}")

win.close()