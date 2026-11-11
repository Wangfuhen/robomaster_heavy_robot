# just_for_testing 分支说明

本分支用于测试命名/符号重构后的云台代码，`main` 保持为备份 + 当前工程快照。

## 这个分支包含什么

- pitch 轴的协议解析、常量、字段、函数、枚举命名重构
- 三个方向系数集中到 `balance_application/all_task/gimbal_configuration.h`
- pitch 编码器解绕阈值修正（4096 / 8192）
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

## 已知未完成（见问题文档第 0 节状态表）

| 编号 | 问题 | 状态 |
|---|---|---|
| P3 | C620 `DATA[7]` 错误码 | 未解析 |
| P4 | 反馈超时 | 未处理 |
| A1 | 编码器多圈累计 | 未实现 |
| A2 | 三种角度语义统一 | 未动 |
| A3 | 指令速率限制 | 未动 |
| A4 | 外环 300 rad/s 假限幅 | 未动 |
| A5 | GYRO 冻结 `absolute_angle_setpoint_radian` | 未动 |
| A7 | 两套 pitch 控制器并存 | 未动 |
| A8 | 模式切换 bumpless | 未动 |

## 测试记录

| 日期 | 测试内容 | 结果 | 备注 |
|---|---|---|---|
| | | | |
