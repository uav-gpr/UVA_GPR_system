# 第8轮：用户授权队长收尾记录（t20）

## 授权与范围

用户在 `calibration-captain-takeover` 问答中选择“授权队长接管收尾”。t20 基于已完成的 t18，正式经 reassign_task 取得 Captain attempt 2，关闭第三轮审查报告及补充证据中的 GND-12/13/14。队长不自审；独立复审为 t21，最终同源双端/全量验证为 t11，打包及解压复验为 t7/t8。

修改仅涉及：
- [地面主程序](<D:/博士任务/UVA_GPR_system/uav_gpr_ground_station.py>)
- [地面专项回归](<D:/博士任务/UVA_GPR_system/tests/test_round8_calibration_ground.py>)
- 本记录。

未修改空中服务、共享 RemoteSender、校准数学模块、GNSS、独立 E2E、真实校准文件、用户配置/数据、旧便携包；没有部署或注册计划任务。

## 修复

1. **GND-12**：legacy fallback helper 不再为 None/True 启动 500ms 完成计时；已排队的 timeout 再检查明确 False 及当前步骤，不能凭时间判定硬件已结束。显式旧端 False 仍正常完成。注意：当前接收末样本的主路径本来已等待 step_done，遗留的是 helper/timeout 路径；本次回归分别覆盖两者，不混淆实际主路径与辅助函数。
2. **GND-13**：watchdog token 增加 `(cal_id, capture_request_id, step)` 上下文；开始新采集/重采、新会话、正常/旧端完成、取消/断线等路径统一清理旧查询。匹配 nonce 但不属于当前操作的旧回复无权改新步骤或在途状态。处理中的无关联快照也不能重写当前步骤行；合法当前查询仍可确认 busy/idle 并恢复。
3. **GND-14**：在解析导出内容、提示、打开文件框或写文件之前检查 pending ID 白名单。合法 None 目标与不存在的 ID 分开；未知/重复/迟到直接丢弃，不用旧标量路径兜底。合法无预设路径正常弹保存框，用户取消有明确提示，多目标独立。

## 专项回归与真实执行

新增 6 个测试方法，现有地面模块从 21 到 27 个方法，原方法未删除/跳过。覆盖：
- None/True 接收路径、fallback helper 和已排队 timeout 均不假完成。
- False 旧端正向完成并清理旧 query。
- 上一步 query 在新步骤开始后到达，不能解锁、消耗新 token 或改新步骤行；当前 query 正向恢复。
- 重采/断线/新会话边界失效。
- 未知及重复 export 无弹框/文件副作用。
- 合法 None 目标的保存框与并发显式目标各自落盘。

一项原 busy/idle 单测夹具补齐真实协议的 cal_id/request_id，且后续新快照对应新查询，不再把同一查询视为两个不同响应；原 busy 锁定及 idle 恢复断言保留。

修复前新增回归实跑：27 项，FAILED（7 个失败/子用例，其中 None/True 为同一方法的两个 subTest）。初次编辑产生一行语法错误，编译立即拦截并已修正；最终再次编译通过，残留文本检查无匹配。

最终执行：

```powershell
& .venv312\Scripts\python.exe -m py_compile uav_gpr_ground_station.py tests\test_round8_calibration_ground.py
& .venv312\Scripts\python.exe docs\run_tests_batched.py --only round8_calibration_ground ui3_calibration_wizard round7_ground_panel
```

```text
py_compile: exit 0
round7_ground_panel: 38 OK
round8_calibration_ground: 27 OK
ui3_calibration_wizard: 8 OK
BATCHED TOTAL: Ran 73 tests in 6.3s over 3 modules; failed modules: none
OVERALL: OK
```

## 独立 E2E 的诚实边界与后续

队长原样执行独立 E2E（后台 job pwsh-272，已收集）：26 项，3 项失败，23 项通过；没有把这个结果报成全绿。

读取失败点确认有三处夹具时序/观察错误，Captain 不越界改测试文件：
1. 未暂存合法 step_done，却在末样本后等 2 秒仍要求 capturing=True——会把真实正常完成误判失败。
2. send_command(start) 后立即读取 active_worker，未等待异步命令被处理。
3. 将在途行硬编码“请求采集中…”，未允许真实正常采样进度“采集中 1/2”。

t20 的 verify 经受控 amendment 保留语法/UI 专项，原 E2E 失败证据保留；**打包前的完整 E2E 与全量门槛没有删除**。独立 verifier 的 t22 负责校正这 3 处同步/观察夹具：真实消息层暂存再释放 step_done、稳定观察运行 worker/Through、按真实当前请求与合法进度判定陈旧拒绝；保留全部产品断言和 26 方法，不删/skip/expectedFailure。如有真正的产品失败必须报告。t21 等待 t20+t22，t11 等待最新独立复审及夹具校正，之后才允许 t7 打包。

## 冻结源码 SHA-256

```text
uav_gpr_ground_station.py
46E00701F073059C53C2A6527AF39F90A164847DA3E9A044323B74B64583F559

tests/test_round8_calibration_ground.py
ADD3108DB5E69846F03F567A5CD71A23D58449B7BC810F8E39DC99B38355D390

uav_gpr_qt.py
EEF2BE69B83AD4034F4D55705CF619663CDC48479CF1987FE7B79F9352A7E7E3

uav_gpr_air_service.py
F10F616328D17D1092DC60E7296E117EA5D8199493651959D2304F4CBC0850AA

calibration_service.py
47FD0D4CEAA70302DC11C1F8AB30F50515CD875738EA395345F525ADE14F8060
```

上述是 t20 自身修复及专项证据，不是独立审查通过、最终全量通过、打包完成或真机精度验证。真实 USB/标准件精度仍需现场复核。
