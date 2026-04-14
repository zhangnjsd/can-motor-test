# 实验：CAN 电机控制

## 任务

要求使用 CAN 控制 6020 电机，完成以下功能：

- 通过 CAN 接口发送控制命令，控制电机的转速和方向。

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

此项目由 ARM-MDK 迁移到此，只保留测试部分，去除 FreeRTOS，仅作测试。
