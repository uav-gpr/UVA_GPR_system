"""第4轮：采集引擎“默认走高速连续”契约测试（团队任务卡 t1 验收基准）。

真机实测约 2.75 s/帧 的根因（captain 已逐行核实，本模块把每条钉成可执行断言）：

1. 地面站 ``command_params`` 只发 count/interval_s/single，**不含引擎选择**——
   故引擎必须由空中端默认值保证（本模块 ``test_ground_command_params_*``）；
2. 空中端 ``start_acquisition`` 依 ``continuous_enabled`` 勾选框二选一；
3. 该勾选框默认值取环境变量 ``UAV_GPR_ACQUISITION_MODE``，未设即 legacy；
4. 第3轮界面精简删掉操作栏后 ``addWidget(self.continuous_enabled)`` 命中 0 ——
   勾选框失去布局宿主，界面上已无法打开它；
5. legacy 每帧调 ``acquire_sparameters``（``librevna_usb_s11.py:362 open_device`` …
   ``:393-398`` 释放关闭），故 2.75 s/帧且控制台每帧打印一次 ``Connected``。

验收核心（**非文字描述**）：不设任何环境变量的缺省 ``MainWindow``，点“开始”后进
``_start_continuous_acquisition`` / ``AcquisitionService`` 路径，而 legacy 的
``acquire_sparameters`` **一次都未被调用**（``test_default_start_never_calls_acquire_sparameters``）。
"""

from __future__ import annotations

import inspect
import os
import tempfile
import unittest
import unittest.mock
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("QTWEBENGINE_DISABLE_SANDBOX", "1")

try:
    import air_core as air
except (ImportError, ModuleNotFoundError) as exc:  # pragma: no cover
    air = None
    IMPORT_ERROR = exc
else:
    IMPORT_ERROR = None

from PySide6 import QtCore, QtWidgets

from librevna.librevna_continuous import PersistentLibreVnaBackend


class EngineDefaultContractTests(unittest.TestCase):
    """① 默认锁高速连续引擎（无界面部分）。

    2026-10-15 重构：空中端旧 GUI MainWindow 已退役，依赖窗口的勾选框/路由/日志
    用例随之删除；黑匣子服务的引擎默认值与 set_engine 由 test_round7_air_service
    覆盖。这里保留不依赖空中端窗口的契约。
    """

    def test_env_default_is_continuous(self) -> None:
        """③ 环境变量未设时默认值不再是 legacy——这正是真机走慢引擎的第一因。"""
        self.assertEqual(air.DEFAULT_ACQUISITION_MODE, "continuous")
        self.assertEqual(air.acquisition_mode(), "continuous")
        self.assertTrue(air.is_continuous_enabled())

    def test_legacy_engine_source_kept_as_fallback(self) -> None:
        """legacy 引擎代码不得删除（调试退路），只是不再做默认。"""
        self.assertTrue(hasattr(air, "AcquisitionWorker"))
        self.assertEqual(air.ENGINE_MODE_LEGACY, "legacy")

    def test_ground_command_params_has_no_engine_field(self) -> None:
        """① 地面站 command_params 仍只有 count/interval_s/single：采集命令
        本身不带引擎字段；引擎经第7轮 ``set_engine`` 专用 action 走设备面板。

        守卫口径修订（2026-09-30 captain 裁决 A，修订台账授权 engB 收窄）：
        第3轮原口径为全文源码 grep 禁 ``"engine"``/``'engine'`` 字面量
        （"引擎选择不靠协议"）。第7轮用户已审批的计划
        （``docs/第7轮_计划_空中端黑匣子与计算棒部署.md`` 第五节 D5）明确
        新增「空中端设备」面板，经既有 ``command`` 消息 + 新 action
        ``set_engine`` 远程切换引擎，全文 grep 因此不再成立。收窄后的
        三层守卫（仍有牙）：

        a) 语义冻结原样保留：``command_params()`` 键集合严格
           ``{count, interval_s, single}``（单次/连续两形态都查）——往采集
           命令塞引擎字段，本断言必红；
        b) 定向源码守卫：AST 解析 ``AcquisitionUiSettings`` 类区域
           （command_params 构造及其直接相关区域），任何 ``"engine"``
           字符串字面量 / ``engine=`` 关键字即判红——在该区域内引用引擎必红；
        c) 正向锁定豁免路径：引擎经协议只允许 ``set_engine`` 这一个 action，
           取值词表锁定 continuous|legacy（engA/t1 逐字对齐）。
        """
        from uav_gpr_ground_station import AcquisitionUiSettings

        params = AcquisitionUiSettings(count=5, interval_s=0.25).command_params()
        self.assertEqual(set(params), {"count", "interval_s", "single"})
        self.assertEqual(params["count"], 5)
        self.assertEqual(params["interval_s"], 0.25)
        self.assertFalse(params["single"])
        # 单次采集形态同样不得携带引擎字段（第3轮语义覆盖两种形态）。
        single_params = AcquisitionUiSettings(
            count=5, interval_s=0.25
        ).command_params(single=True)
        self.assertEqual(set(single_params), {"count", "interval_s", "single"})

        # ---- b) 定向源码守卫（替换原 234-236 全文 grep）----
        import ast

        import uav_gpr_ground_station as ground_module

        source = Path(ground_module.__file__).read_text(encoding="utf-8")
        tree = ast.parse(source)
        region: list[ast.AST] = []
        for node in ast.walk(tree):
            if isinstance(node, ast.ClassDef) and node.name == "AcquisitionUiSettings":
                region = list(ast.walk(node))
                break
        self.assertTrue(region, "AcquisitionUiSettings 类必须存在（守卫区域锚点）")
        offenders: list[str] = []
        for node in region:
            if isinstance(node, ast.Constant) and isinstance(node.value, str):
                if node.value.strip() == "engine":
                    offenders.append(f"字符串字面量 'engine' @line {node.lineno}")
            if isinstance(node, ast.keyword) and node.arg == "engine":
                offenders.append(f"调用关键字 engine= @line {node.lineno}")
            if isinstance(node, ast.arg) and node.arg == "engine":
                offenders.append(f"形参 engine @line {node.lineno}")
            if isinstance(node, ast.Attribute) and node.attr == "engine":
                offenders.append(f"属性访问 .engine @line {node.lineno}")
            if isinstance(node, ast.Name) and node.id == "engine":
                offenders.append(f"变量名 engine @line {node.lineno}")
        self.assertEqual(
            offenders,
            [],
            "采集参数区域（AcquisitionUiSettings）不得引入 engine 字段（第3轮语义冻结）",
        )

        # ---- c) 正向锁定第7轮豁免路径：引擎只经 set_engine 走协议 ----
        engine_actions = [
            action
            for action in ground_module.AIR_DEVICE_ACTIONS
            if "engine" in action
        ]
        self.assertEqual(
            engine_actions,
            ["set_engine"],
            "引擎相关协议 action 只允许 set_engine（第7轮 D5 唯一豁免路径）",
        )
        self.assertEqual(
            ground_module.AIR_ENGINE_VALUES,
            ("continuous", "legacy"),
            "引擎取值词表锁定（与空中端 air_config.json 一致）",
        )



@unittest.skipIf(air is None, f"uav_gpr_qt dependencies unavailable: {IMPORT_ERROR}")
class LegacyEngineCostEvidenceTests(unittest.TestCase):
    """⑤ 源码级证据：legacy 每帧开关一次 USB，因此不能做默认引擎。"""

    def test_legacy_worker_calls_acquire_sparameters_per_frame(self) -> None:
        source = inspect.getsource(air.AcquisitionWorker)
        self.assertIn(
            "acquire_sparameters", source,
            "legacy worker 的逐帧 acquire_sparameters 是其慢的根因（每帧 open/close USB）",
        )


if __name__ == "__main__":
    unittest.main()
