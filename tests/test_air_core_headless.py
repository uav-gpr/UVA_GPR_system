"""2026-10-15 重构守卫：空中端核心 air_core 无界面化、旧 GUI 退役。

用大白话说：计算棒上的黑匣子只需要"干活的零件"，不需要窗口、画图库、地图浏览器。
这组测试保证以后没人不小心把这些重家伙又塞回空中端。
"""

from __future__ import annotations

import ast
import os
import subprocess
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

# 空中端核心与黑匣子服务都不得直接 import 的界面/画图模块。
FORBIDDEN_IMPORT_ROOTS = {"pyqtgraph", "gnss_map_widget", "uav_gpr_ground_station"}
FORBIDDEN_QT_SUBMODULES = {"QtWidgets", "QtGui", "QtWebEngineWidgets", "QtWebEngineCore"}

HEAVY_MODULES = (
    "PySide6.QtWidgets",
    "PySide6.QtGui",
    "PySide6.QtWebEngineWidgets",
    "PySide6.QtWebEngineCore",
    "pyqtgraph",
)


def _imports(path: Path) -> tuple[set[str], set[str]]:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    roots: set[str] = set()
    qt_names: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                roots.add(alias.name.split(".")[0])
                if alias.name.startswith("PySide6."):
                    qt_names.add(alias.name.split(".")[1])
        elif isinstance(node, ast.ImportFrom) and node.module:
            roots.add(node.module.split(".")[0])
            if node.module == "PySide6":
                qt_names.update(alias.name for alias in node.names)
            elif node.module.startswith("PySide6."):
                qt_names.add(node.module.split(".")[1])
    return roots, qt_names


class AirCoreHeadlessTests(unittest.TestCase):
    def test_air_core_source_has_no_gui_imports(self) -> None:
        for name in ("air_core.py", "uav_gpr_air_service.py", "air_gnss.py"):
            roots, qt_names = _imports(ROOT / name)
            self.assertFalse(roots & FORBIDDEN_IMPORT_ROOTS, f"{name} 引入了界面模块: {roots & FORBIDDEN_IMPORT_ROOTS}")
            self.assertFalse(
                qt_names & FORBIDDEN_QT_SUBMODULES,
                f"{name} 引入了 Qt 界面子模块: {qt_names & FORBIDDEN_QT_SUBMODULES}",
            )

    def test_air_service_import_does_not_load_gui_stack(self) -> None:
        """在干净子进程里 import 黑匣子服务，确认没有把窗口/画图/WebEngine 拉进内存。"""
        code = (
            "import sys, json; sys.path.insert(0, %r); import uav_gpr_air_service; "
            "print(json.dumps([m for m in %r if m in sys.modules]))"
        ) % (str(ROOT), HEAVY_MODULES)
        result = subprocess.run(
            [sys.executable, "-c", code], capture_output=True, text=True, cwd=str(ROOT), timeout=120
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        loaded = result.stdout.strip().splitlines()[-1]
        self.assertEqual(loaded, "[]", f"黑匣子服务 import 时加载了界面模块: {loaded}")

    def test_air_service_uses_air_core(self) -> None:
        roots, _ = _imports(ROOT / "uav_gpr_air_service.py")
        self.assertIn("air_core", roots)
        self.assertNotIn("uav_gpr_qt", roots)

    def test_legacy_alias_is_same_module_object(self) -> None:
        """旧名 uav_gpr_qt 只是别名：同一个模块对象，patch 也照常生效。"""
        import air_core
        import uav_gpr_qt

        self.assertIs(uav_gpr_qt, air_core)

    def test_old_air_gui_is_retired(self) -> None:
        import air_core

        for name in ("MainWindow", "AirSettingsDialog", "BScanWidget", "_apply_trace_to_ui", "main"):
            self.assertFalse(hasattr(air_core, name), f"空中端旧 GUI 残留: {name}")

    def test_air_launcher_starts_black_box_service(self) -> None:
        text = (ROOT / "run_air_station.bat").read_text(encoding="utf-8")
        self.assertIn("uav_gpr_air_service.py", text)
        self.assertNotIn('"%~dp0uav_gpr_qt.py"', text)

    def test_ground_station_never_execs_modal_dialogs(self) -> None:
        """R-C-04（原 test_round2_air_ui_compact 迁来，只剩地面站一侧）。"""
        text = (ROOT / "uav_gpr_ground_station.py").read_text(encoding="utf-8")
        self.assertNotIn("calibration_dialog.exec", text)
        self.assertNotIn("settings_dialog.exec(", text)


if __name__ == "__main__":
    unittest.main()
