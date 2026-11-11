# just_for_testing 分支说明

本分支用于测试命名/符号重构 + 第一轮安全修复后的云台代码，`main` 保持为备份 + 工程快照。

## 这个分支包含什么

- pitch 轴的协议解析、常量、字段、函数、枚举命名重构
- 三个方向系数集中到 `balance_application/all_task/gimbal_configuration.h`
- C620 `DATA[7]` 错误码解析 + 20 ms 反馈超时切断（P3/P4）
- 设定值速率限制 0.30 rad/s（A3，只用于 IMU 关节角设定值）
- 外环速率上限从 300 rad/s 改为 0.35 rad/s（A4）
- GYRO 设定值每周期跟随实测（A5）
- 模式切换统一切换处理：同步目标 + 清空所有控制器（A8）
- **pitch 控制统一用 IMU 绝对角**：GYRO、AUTO、ENCODER 三种模式的 pitch 都闭环在 `absolute_angle_radian` 上；编码器只更新 `relative_angle_radian` 做诊断
- **非线性角度映射已回退**：等 A=0 的编码器计数和丝杆长度 `x0` 标定后再启用
- 文档：
  - `docs/pitch_3508_C620_协议核对与问题分析.md`
  - `docs/命名与符号规范.md`

## 上电前必须确认

- [ ] 电调 ID = 5（绿灯 1 秒闪 5 次）
- [ ] `pitch_axis_motor_rotation_to_joint_angle_sign`（默认 -1）
- [ ] `pitch_axis_inertial_measurement_angular_velocity_sign`（默认 +1）
- [ ] `pitch_axis_screw_stick_direction_sign`（默认 +1）
- [ ] 架空测试：`GIMBAL_OPERATING_MODE_ZERO_FORCE` 下确认 `encoder_count` 随手动转动变化
- [ ] 给 +500 电流确认转向，再进闭环
- [ ] 确认 `controller_error_code` 和 `last_update_time_millisecond` 已更新（VOFA / debug_task）
- [ ] 确认 20 ms 超时逻辑：拔掉 CAN 线，pitch/yaw 电流应归零

## 已知未完成（见问题文档第 0 节状态表）

| 编号 | 问题 | 状态 |
|---|---|---|
| P1 | 电调 ID=5 确认 | 待现场确认 |
| P5 | bus-off 不恢复、滤波器全通 | 未做 |
| A1 | 非线性角度映射 | 已回退，待标定 A=0 编码器计数与 `x0` |
| A2 | yaw 的相对/绝对角坐标 | 未统一（pitch 已统一） |
| A7 | GYRO 丝杆速度环 vs AUTO/ENCODER 串级 | 未统一 |
| B1–B6 | 静差、增益差、速度上限、摩擦、延迟 | 未做 |

## 本轮改动的风险点

- **ENCODER 模式的 pitch 行为变了**：以前用 LQR 闭环转子角，现在用 IMU 绝对角串级（`angle_controller` + `speed_controller`）。YAW 仍是相对角 PID。
- **GIMBAL_OPERATING_MODE_ENCODER_ANGLE 这个名字现在只对 yaw 准确**：pitch 已经不用编码器闭环了。
- `pitch_axis_absolute_angle_maximum_output_radian_per_second` 从 300 改到 **0.35**：外环饱和点变小，若实机觉得“跟不上”，先确认是速度上限还是这个限幅。
- `pitch_axis_setpoint_rate_limit_radian_per_second = 0.30`：遥控器/PC 目标变化被限速，手感会比以前“慢”，这是预期的。
- `Gimbal_HandleOperatingModeChange()` 现在会在任何模式切换时清空所有 PID。
- 机构参数已按 **导程 2 mm、减速比 19:1** 写入配置；等标定后再算 mm/count。

## 测试记录

| 日期 | 测试内容 | 结果 | 备注 |
|---|---|---|---|
| | | | |
