# 实验：FreeRTOS CAN PID 电机控制

## 任务

要求使用 PID 控制 6020 电机，完成以下功能：

- ​速度内环 (PI/PID)

  1. PI 重点：引入 I（积分） 是为了消除静差，确保电机能克服摩擦力达到目标转速。
  2. 要求防超调：需要严格限制积分限幅（Integral Separation），防止积分饱和（Anti-windup）导致速度冲过头。

- 角度外环 (P/PD)

  1. PD 重点：D（微分） 环节非常重要，它能起到“预测”和“阻尼”作用，吸收速度环带来的惯性，使角度收敛更快。
  2. 收敛精度：目标精度为 5 个编码器数值（Count）。这意味着你的死区（Deadzone）处理和静态补偿必须到位，确保电机在微小误差时仍有足够的力矩克服静摩擦。

## 配置

配置 CAN1 与 CAN2

- CAN1：连接到电机控制器。
  - `Prescaler = 3`: 设置预分频器为 3，以调整 CAN 总线的时钟频率。
  - `Time Quantum Bit Segment 1 = 10`: 设置时间量子位段 1 的长度为 10 个时间量子。(每个时间量子是 CAN 总线时钟周期的一个单位)
  - `Time Quantum Bit Segment 2 = 3`: 设置时间量子位段 2 的长度为 3 个时间量子。
  - `CAN1_RX = PD0`: 将 CAN1 的接收引脚 RX0 配置为 PD0。
  - `CAN1_TX = PD1`: 将 CAN1 的发送引脚 TX 配置为 PD1。
- CAN2：连接到电机控制器。
  - `Prescaler = 3`: 设置预分频器为 3，以调整 CAN 总线的时钟频率。
  - `Time Quantum Bit Segment 1 = 10`: 设置时间量子位段 1 的长度为 10 个时间量子。(每个时间量子是 CAN 总线时钟周期的一个单位)
  - `Time Quantum Bit Segment 2 = 3`: 设置时间量子位段 2 的长度为 3 个时间量子。
  - `CAN2_RX = PB5`: 将 CAN2 的接收引脚 RX0 配置为 PB5。
  - `CAN2_TX = PB6`: 将 CAN2 的发送引脚 TX 配置为 PB6。

## 操作

随心所欲
