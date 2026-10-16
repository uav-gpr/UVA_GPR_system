"""兼容别名：``import uav_gpr_qt`` 等价于 ``import air_core``（同一个模块对象）。

2026-10-15 重构后，空中端旧 GUI 已退役，核心逻辑改名为 ``air_core.py``。
旧脚本（docs/ 下的基准与复现脚本等）仍可 ``import uav_gpr_qt as air``；
因为是同一个模块对象，``patch.object(air, ...)`` 也照常生效。
新代码请直接 ``import air_core``。
"""

import sys

import air_core

sys.modules[__name__] = air_core
