"""地面站「历史采集回看」窗口（独立非模态窗口，不动主界面布局）。

- 打开：采集文件夹（remote_runs/<run_id>）或合并文件（*_merged.npz）；
- 显示整次采集的 B-scan（HH / VV 各一幅，可拖动缩放看整条测线）；
- 离线重新做 Dewow / Flat Filter，色阶滑块与色图同实时界面；
- 「合并保存」把文件夹合并成一个文件；「导出处理结果」另存，不覆盖原始数据。

实时采集与回看互不影响：本窗口只读磁盘文件，不接触接收线程和实时 B-scan。
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pyqtgraph as pg
from PySide6 import QtCore, QtWidgets

from run_archive import (
    CHANNEL_LABELS,
    ProcessedRun,
    ProcessingParams,
    RunArchiveError,
    RunData,
    format_index_ranges,
    is_merged_file,
    load_run,
    merge_run,
    merged_path_for,
    reprocess,
    save_processed,
    stored_dewow_enabled,
)

CONTRAST_MIN = 1
CONTRAST_MAX = 100


def contrast_levels(max_abs: float, percent: float) -> tuple[float, float]:
    """对称色阶 (-v, +v)，v = max_abs × percent%（与实时界面同口径）。"""
    base = float(max_abs) if math.isfinite(float(max_abs)) and float(max_abs) > 0 else 1.0
    pct = min(max(float(percent), CONTRAST_MIN), CONTRAST_MAX)
    value = base * pct / 100.0
    return -value, value


class OfflineRunViewer(QtWidgets.QMainWindow):
    def __init__(self, default_dir: Path | None = None, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("offline_run_viewer")
        self.setWindowTitle("历史采集回看")
        self.resize(1200, 760)
        self.default_dir = Path(default_dir) if default_dir is not None else Path.cwd()
        self.run: RunData | None = None
        self.result: ProcessedRun | None = None
        self._max_abs: dict[str, float] = {}
        self._run_path: Path | None = None

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        outer = QtWidgets.QVBoxLayout(central)

        # ---- 文件操作 ----
        file_bar = QtWidgets.QHBoxLayout()
        self.open_folder_button = QtWidgets.QPushButton("打开采集文件夹…")
        self.open_folder_button.setObjectName("offline_open_folder_button")
        self.open_file_button = QtWidgets.QPushButton("打开合并文件…")
        self.open_file_button.setObjectName("offline_open_file_button")
        self.merge_button = QtWidgets.QPushButton("合并保存")
        self.merge_button.setObjectName("offline_merge_button")
        self.merge_button.setToolTip("把当前采集文件夹的 trace_*.npz 合并成一个文件（原始文件保留）")
        self.export_button = QtWidgets.QPushButton("导出处理结果…")
        self.export_button.setObjectName("offline_export_button")
        for button in (self.open_folder_button, self.open_file_button, self.merge_button, self.export_button):
            file_bar.addWidget(button)
        file_bar.addStretch(1)
        outer.addLayout(file_bar)

        self.info_label = QtWidgets.QLabel("还没有打开采集。")
        self.info_label.setObjectName("offline_info_label")
        self.info_label.setWordWrap(True)
        outer.addWidget(self.info_label)

        # ---- 处理参数 ----
        proc_bar = QtWidgets.QHBoxLayout()
        self.dewow_check = QtWidgets.QCheckBox("Dewow")
        self.dewow_check.setObjectName("offline_dewow_check")
        self.dewow_window = QtWidgets.QDoubleSpinBox()
        self.dewow_window.setObjectName("offline_dewow_window")
        self.dewow_window.setRange(0.1, 1000.0)
        self.dewow_window.setDecimals(1)
        self.dewow_window.setSingleStep(0.5)
        self.dewow_window.setValue(4.0)
        self.dewow_window.setSuffix(" ns")
        self.flat_check = QtWidgets.QCheckBox("Flat Filter")
        self.flat_check.setObjectName("offline_flat_check")
        self.flat_window = QtWidgets.QSpinBox()
        self.flat_window.setObjectName("offline_flat_window")
        self.flat_window.setRange(3, 10001)
        self.flat_window.setSingleStep(2)
        self.flat_window.setValue(31)
        self.flat_window.setSuffix(" 道")
        self.apply_button = QtWidgets.QPushButton("应用处理")
        self.apply_button.setObjectName("offline_apply_button")
        self.cmap_combo = QtWidgets.QComboBox()
        self.cmap_combo.setObjectName("offline_cmap_combo")
        self.cmap_combo.addItems(["gray", "jet"])
        self.contrast_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.contrast_slider.setObjectName("offline_contrast_slider")
        self.contrast_slider.setRange(CONTRAST_MIN, CONTRAST_MAX)
        self.contrast_slider.setValue(100)
        self.contrast_slider.setMinimumWidth(120)
        self.contrast_label = QtWidgets.QLabel("色阶 100%")
        for widget in (
            self.dewow_check,
            self.dewow_window,
            QtWidgets.QLabel("  "),
            self.flat_check,
            self.flat_window,
            self.apply_button,
            QtWidgets.QLabel("   色图"),
            self.cmap_combo,
            self.contrast_label,
            self.contrast_slider,
        ):
            proc_bar.addWidget(widget)
        proc_bar.addStretch(1)
        outer.addLayout(proc_bar)

        # ---- B-scan ----
        self.plot_area = pg.GraphicsLayoutWidget()
        self.plot_area.setObjectName("offline_plot_area")
        outer.addWidget(self.plot_area, 1)
        self.plots: dict[str, pg.PlotItem] = {}
        self.images: dict[str, pg.ImageItem] = {}

        self.status_label = QtWidgets.QLabel("")
        self.status_label.setObjectName("offline_status_label")
        self.statusBar().addWidget(self.status_label, 1)

        self.open_folder_button.clicked.connect(lambda: self.open_folder_dialog())
        self.open_file_button.clicked.connect(lambda: self.open_file_dialog())
        self.merge_button.clicked.connect(lambda: self.merge_current())
        self.export_button.clicked.connect(lambda: self.export_dialog())
        self.apply_button.clicked.connect(lambda: self.apply_processing())
        self.contrast_slider.valueChanged.connect(self._on_contrast_changed)
        self.cmap_combo.currentTextChanged.connect(lambda _text: self._update_images(levels_only=False))
        self.dewow_check.toggled.connect(self.dewow_window.setEnabled)
        self.flat_check.toggled.connect(self.flat_window.setEnabled)
        self.dewow_window.setEnabled(False)
        self.flat_window.setEnabled(False)
        self._update_buttons()

    # ---- 打开 ----

    def open_folder_dialog(self) -> None:
        folder = QtWidgets.QFileDialog.getExistingDirectory(self, "选择采集文件夹", str(self.default_dir))
        if folder:
            self.open_path(Path(folder))

    def open_file_dialog(self) -> None:
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "选择合并文件", str(self.default_dir), "合并文件 (*_merged.npz);;所有 npz (*.npz)"
        )
        if path:
            self.open_path(Path(path))

    def open_path(self, path: Path) -> bool:
        QtWidgets.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
        try:
            self._set_status(f"正在读取 {path} …")
            QtWidgets.QApplication.processEvents()
            run = load_run(path)
        except (RunArchiveError, OSError, ValueError, KeyError) as exc:
            self._set_status(f"打开失败：{exc}")
            QtWidgets.QApplication.restoreOverrideCursor()
            return False
        QtWidgets.QApplication.restoreOverrideCursor()
        self.run = run
        self._run_path = Path(path)
        # 默认参数 = 采集时的参数，先让离线结果与实时一致。
        dewow_info = run.processing.get("dewow") if isinstance(run.processing, dict) else None
        window = None
        if isinstance(dewow_info, dict):
            window = dewow_info.get("window_ns")
        try:
            if window is not None:
                self.dewow_window.setValue(float(window))
        except (TypeError, ValueError):
            pass
        self.dewow_check.setChecked(stored_dewow_enabled(run))
        self._build_plots()
        self._update_info()
        self.apply_processing()
        return True

    def _update_info(self) -> None:
        run = self.run
        if run is None:
            self.info_label.setText("还没有打开采集。")
            return
        parts = [
            f"采集：{run.run_id}",
            f"{run.n_traces} 道",
            "通道 " + "+".join(CHANNEL_LABELS.get(c, c) for c in run.channels),
        ]
        if run.frequency_hz.size:
            parts.append(
                f"{run.frequency_hz[0] / 1e6:.0f}–{run.frequency_hz[-1] / 1e6:.0f} MHz，{run.frequency_hz.size} 点"
            )
        if run.missing_trace_indices:
            parts.append(
                f"缺号 {format_index_ranges(run.missing_trace_indices)}（{len(run.missing_trace_indices)} 道）"
            )
        else:
            parts.append("无缺号")
        if run.skipped_files:
            parts.append(f"跳过 {len(run.skipped_files)} 个文件")
        parts.append("来源：合并文件" if is_merged_file(Path(run.source)) else "来源：原始文件夹")
        self.info_label.setText("；".join(parts))

    def _build_plots(self) -> None:
        self.plot_area.clear()
        self.plots.clear()
        self.images.clear()
        if self.run is None:
            return
        for row, name in enumerate(self.run.channels):
            plot = self.plot_area.addPlot(row=row, col=0)
            plot.setTitle(f"{CHANNEL_LABELS.get(name, name)} B-scan")
            plot.setLabel("bottom", "Trace")
            plot.setLabel("left", "时间", units="ns")
            plot.getViewBox().invertY(True)
            image = pg.ImageItem(axisOrder="row-major")
            plot.addItem(image)
            self.plots[name] = plot
            self.images[name] = image
        self._update_buttons()

    # ---- 处理 ----

    def current_params(self) -> ProcessingParams:
        window = int(self.flat_window.value())
        if window % 2 == 0:
            window -= 1
        return ProcessingParams(
            dewow_enabled=bool(self.dewow_check.isChecked()),
            dewow_window_ns=float(self.dewow_window.value()),
            flat_enabled=bool(self.flat_check.isChecked()),
            flat_window_traces=max(3, window),
        )

    def apply_processing(self) -> bool:
        if self.run is None:
            return False
        QtWidgets.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
        try:
            self._set_status("正在处理…")
            QtWidgets.QApplication.processEvents()
            result = reprocess(self.run, self.current_params())
        except (ValueError, TypeError) as exc:
            QtWidgets.QApplication.restoreOverrideCursor()
            self._set_status(f"处理失败：{exc}")
            return False
        QtWidgets.QApplication.restoreOverrideCursor()
        self.result = result
        self._update_images(levels_only=False)
        note = "；".join(result.notes)
        self._set_status("处理完成" + (f"（{note}）" if note else ""))
        self._update_buttons()
        return True

    def display_matrix(self, name: str) -> np.ndarray | None:
        if self.result is None or name not in self.result.matrices:
            return None
        return np.ascontiguousarray(self.result.matrices[name].real)

    def _lut(self):
        if self.cmap_combo.currentText() == "jet":
            return pg.colormap.get("CET-R4").getLookupTable(0.0, 1.0, 256)
        return None

    def _update_images(self, levels_only: bool) -> None:
        if self.result is None:
            return
        percent = int(self.contrast_slider.value())
        lut = None if levels_only else self._lut()
        time_ns = self.result.time_ns
        for name, image in self.images.items():
            matrix = self.display_matrix(name)
            if matrix is None:
                continue
            if levels_only:
                image.setLevels(contrast_levels(self._max_abs.get(name, 1.0), percent))
                continue
            finite = matrix[np.isfinite(matrix)]
            max_abs = float(np.max(np.abs(finite))) if finite.size else 1.0
            self._max_abs[name] = max_abs if max_abs > 0 else 1.0
            image.setImage(matrix, autoLevels=False, levels=contrast_levels(self._max_abs[name], percent))
            image.setLookupTable(lut)
            n_traces = matrix.shape[1]
            if time_ns.size:
                t0 = float(time_ns[0])
                span = max(float(time_ns[-1]) - t0, 1e-9)
                image.setRect(QtCore.QRectF(0.5, t0, float(n_traces), span))
                plot = self.plots[name]
                plot.setXRange(0.5, n_traces + 0.5, padding=0)
                plot.setYRange(t0, t0 + span, padding=0)

    def _on_contrast_changed(self, value: int) -> None:
        self.contrast_label.setText(f"色阶 {int(value)}%")
        self._update_images(levels_only=True)

    # ---- 合并 / 导出 ----

    def _run_folder(self) -> Path | None:
        if self.run is None:
            return None
        source = Path(self.run.source)
        if source.is_dir():
            return source
        return None

    def merge_current(self) -> bool:
        folder = self._run_folder()
        if folder is None:
            self._set_status("当前打开的已经是合并文件，无需再合并")
            return False
        QtWidgets.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
        try:
            report = merge_run(folder)
        except (RunArchiveError, OSError) as exc:
            QtWidgets.QApplication.restoreOverrideCursor()
            self._set_status(f"合并失败：{exc}")
            return False
        QtWidgets.QApplication.restoreOverrideCursor()
        self._set_status(report.summary_text())
        return True

    def export_dialog(self) -> None:
        if self.run is None or self.result is None:
            return
        base = self._run_folder() or Path(self.run.source).parent
        suggestion = base / f"{self.run.run_id}_processed.npz"
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "导出处理结果", str(suggestion), "npz (*.npz)")
        if path:
            self.export_to(Path(path))

    def export_to(self, path: Path) -> bool:
        if self.run is None or self.result is None:
            return False
        try:
            save_processed(self.run, self.result, path)
        except OSError as exc:
            self._set_status(f"导出失败：{exc}")
            return False
        self._set_status(f"已导出：{path}")
        return True

    # ---- 杂项 ----

    def _update_buttons(self) -> None:
        has_run = self.run is not None
        self.apply_button.setEnabled(has_run)
        self.merge_button.setEnabled(has_run and self._run_folder() is not None)
        self.export_button.setEnabled(has_run and self.result is not None)

    def _set_status(self, text: str) -> None:
        self.status_label.setText(text)


__all__ = ["OfflineRunViewer", "contrast_levels", "merged_path_for"]
