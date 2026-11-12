# 3.10 CAN MIT

[上一节 3.9 速度估计与速度闭环](07-speed.md) · [下一节 3.11 开环 LUT 标定](09-lut.md)

在 Simulink 中，MIT 模式有五个输入。部署时，这五个量由主机经 CAN 更新，控制器还要判断命令是否有效、模式是否允许和通信是否超时。本节从一帧可离线生成的报文出发，读到输出轴力矩、电机侧 Iq 和测量回包。

<span id="通用-foc-部署"></span>

## 通用部署

### 从线上的八个字节恢复模型输入

在配套工程根目录执行下面的命令。默认只打印发送计划。它先请求 Disabled，再请求 MIT，周期目标帧为 `0x013  82 8E 80 60 10 05 18 16`，退出时安排发送 Disabled。 [主机命令](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/tools/can_demo.py#L246)

```powershell
python tools/can_demo.py mit --position 1 --velocity 0.5 --kp 2 --kd 0.1 --torque 0.2 --duration 2
```

表中的字节编号从 0 开始。低半字节是低四位，高半字节是高四位。同一字节可以各给两个不同的字段四位。每个无符号整数用 `low + raw × (high − low)/(2^bits − 1)` 换回物理量。编码向零截断，所以模型输入和解码后的量存在量化差异。 [量化实现](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/communication/protocol.c#L15) · [MIT 解码](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/communication/protocol.c#L66)

| 模型量 | 帧内位置 | 位宽和量程 | 本例解码 |
|---|---|---|---:|
| 目标位置 | 字节 0～1 | 16 位，−50～50 rad | 0.998703 rad |
| 目标速度 | 字节 2、字节 3 高四位 | 12 位，−150～150 rad/s | 0.476190 rad/s |
| 刚度 Kp | 字节 3 低四位、字节 4 | 12 位，0～500 N·m/rad | 1.953602 N·m/rad |
| 阻尼 Kd | 字节 5、字节 6 高四位 | 12 位，0～5 N·m/(rad/s) | 0.098901 N·m/(rad/s) |
| 前馈力矩 | 字节 6 低四位、字节 7 | 12 位，−18～18 N·m | 0.197802 N·m |

**源码摘录。** `p` 指向接收帧的数据。对速度字段，`p[2]` 的 `0x80` 左移四位得到 `0x800`，`p[3]` 的 `0x60` 右移四位得到 6。按位或后是 `0x806`。刚度字段用 `& 15U` 保留同一个 `p[3]` 的低四位，再拼上 `0x10`。解码成功后，`*command = result` 将结果写回调用者。 [字段位运算](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/communication/protocol.c#L78)

```c
result.speed_ref_rad_s = Unpack(((uint32_t)p[2] << 4) | (p[3] >> 4),
                                PROTOCOL_MIT_SPEED_MIN_RAD_S,
                                PROTOCOL_MIT_SPEED_MAX_RAD_S, 12U);
result.position_kp_Nm_per_rad = Unpack(((uint32_t)(p[3] & 15U) << 8) | p[4],
                                      0.0f, 500.0f, 12U);
```

### 经过应用检查后，才更新电流参考

`0x120` 的第 0 字节请求状态，`02` 为 Disabled，`06` 为 MIT。第 1 字节为 0，第 2 字节仅在 Disabled 请求中可设为 1 以清故障，余下字节为 0。工程也接受 `0x013` 前七字节为 `FF` 且末字节为 `FC` 或 `FD` 的兼容启停帧，成功后通过 `0x003` 回复。末字节为 `FE` 的兼容请求会被解码器拒绝。 [状态和兼容帧解码](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/communication/protocol.c#L29)

在进入 MIT 之前，`request_state` 检查构建阶段、停机状态、偏置采样、编码器、零位、母线和驱动就绪。状态切换清参考和 PI，并恢复 1 A 的教学运行限流。随后 `MotorApp_AcceptCommand` 只在当前状态匹配且无故障时发布完整目标、更新有效命令的接收时间。解码失败或状态不匹配的目标无法续期。 [模式切换](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L116) · [命令发布](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L339)

**执行示意。** 主机每 20 ms 发送目标。固件每 5 ms 用最近接受的目标算一次 MIT，每 50 μs 执行一次电流环。三条时间线各有自己的节拍，收到一帧只更新目标，不能据此判断力矩计算已经发生。

```text
0x013 目标 → 接收队列 → 解码 → 当前模式与故障检查 → 保存目标和时间
最近目标 + 输出轴位置与速度 → 200 Hz MIT → 限幅后的 Iq 参考
Iq 参考 + 电流与电角反馈 → 20 kHz 电流 PI → SVPWM → PWM
```

**源码摘录。** 电流拍计数每到 100 的倍数，应用在 MIT 状态把目标和输出轴测量传给 `Foc_MitStep`。下面是有限值和增益检查之后的计算。 [MIT 外环调用](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L527) · [MIT 到 Iq](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/control/foc.c#L185)

```c
float torque = command->position_kp_Nm_per_rad * (command->position_ref_rad - position) +
    command->speed_kd_Nm_per_rad_s * (command->speed_ref_rad_s - speed) +
    command->torque_feedforward_Nm;
if (!isfinite(torque)) return false;
float limit = fminf(foc->iq_limit_A, MOTOR_CURRENT_LIMIT_A);
/* MIT全部量都在输出轴；理想减速器tau_out=N*Kt*Iq，不混用电机轴速度。 */
float output_torque_per_ampere = MOTOR_GEAR_RATIO * MOTOR_TORQUE_CONSTANT_NM_PER_A;
foc->iq_ref_A = Clamp(torque / output_torque_per_ampere, -limit, limit);
foc->torque_output_Nm = foc->iq_ref_A * output_torque_per_ampere;
foc->id_ref_A = 0.0f;
```

位置误差乘刚度，速度误差乘阻尼，两个力矩分量与前馈相加。输出轴每安培力矩按理想减速器取 `N × Kt`，除后得到电机侧 Iq 请求。`torque_output_Nm` 是由限流后 Iq 推算的请求力矩，实物力矩还受传动与负载影响。若算法返回 `false`，应用用 `&&` 短路跳过该拍电流 PI，随后走采样故障关断。

**手算输入。** 将表中解码值和假定的输出轴位置 0.8 rad、速度 0.25 rad/s 代入，力矩请求约 0.60836 N·m。DM8009 的减速比为 9，Kt 为 0.132552 N·m/A，输出轴每安培对应 1.192968 N·m/A，因此 Iq 请求约 0.510 A。换成自己模型的电机参数时，仍在输出轴域计算力矩，在电机轴域约束 Iq。 [减速比与力矩常数](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/config/motor_config.h#L12)

### 把回包和本次计算区分开

| 帧 | 应用内容 | 要核对的轴与单位 |
|---|---|---|
| `0x003` | 接受 MIT 目标后排队回复 | 输出轴位置 ±50 rad、输出轴速度 ±150 rad/s、电机 Iq ±18 A |
| `0x182` | 开环、电流、速度状态的周期测量 | 输出轴位置 ±50 rad、电机轴速度 ±376.991 rad/s、电机 Iq ±60 A |
| `0x180` | 应用状态、16 位故障和编码器状态 | Fault 状态为 8，通信故障位为 `0x08` |

`0x003` 与 `0x182` 都用八字节测量格式。字节 0 是 `0x13`，字节 1～2 放位置，字节 3～5 拼两个 12 位的速度与电流，字节 6、7 放状态和故障低八位。`0x003` 在接受目标时抓取已有测量，可能早于目标在下一个 5 ms 节拍生效。离线对照时沿时间轴分别记下解码目标、用于计算的测量、力矩请求、限幅后 Iq 和回包。 [测量编码](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/communication/protocol.c#L126) · [回复生成与发送](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L363)

**离线核对。** 用上面一帧和同一组输出轴测量重放 MIT 计算。将算例中的有效限流设为 0.4 A，预期未限幅请求仍约为 0.510 A，交给电流环的参考为 0.4 A，`torque_output_Nm` 约为 0.477 N·m。再用相同目标分别检查 MIT 和 Disabled 的命令分支，标出哪一支会发布目标并续期。

<span id="stm32-部署示例"></span>

## STM32 实现

### 由 CubeMX 配置 H743 的 Classic CAN

在配套 `.ioc` 打开 **Pinout & Configuration → Connectivity → FDCAN1**，用 **Parameter Settings** 核对帧格式、位时序、滤波器数量和消息 RAM，再在 **NVIC Settings** 复核中断。内核时钟从 **Clock Configuration** 核对。 [CubeMX 配置](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/FOC_Arclab_H743_V3.ioc#L150) · [生成的 FDCAN 初始化](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/Core/Src/fdcan.c#L30)

| 项目 | 本工程设置 | 生成代码 |
|---|---|---|
| 模式、帧格式、引脚 | Normal、Classic CAN，PD0 RX、PD1 TX | `FDCAN_MODE_NORMAL`、`FDCAN_FRAME_CLASSIC`、`GPIO_AF9_FDCAN1` |
| 内核时钟 | PLL1 Q，48 MHz | `RCC_FDCANCLKSOURCE_PLL` |
| Nominal 位时序 | 预分频 3，Seg1 13，Seg2 2，SJW 1 | `NominalPrescaler`、`NominalTimeSeg1/2`、`NominalSyncJumpWidth` |
| Auto Retransmission | Disable | `AutoRetransmission = DISABLE` |
| 过滤与接收 | 1 个标准过滤项，0 个扩展项。Rx FIFO0 16 个元素，每元素 8 字节 | `StdFiltersNbr`、`ExtFiltersNbr`、`RxFifo0ElmtsNbr`、`RxFifo0ElmtSize` |
| 发送 | Tx FIFO 3 个元素，每元素 8 字节 | `TxFifoQueueElmtsNbr`、`TxElmtSize` |
| NVIC | FDCAN1 IT0，抢占优先级 4，子优先级 0 | `HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 4, 0)` |

48 MHz 时钟经 3 分频，每个时间量子为 62.5 ns。每位由同步段 1、Seg1 13、Seg2 2 共 16 个时间量子组成，位时间 1 μs，对应 1 Mbps。采样点在 Seg1 末尾，即 `(1+13)/16=87.5%`。换芯片时从所选 CAN 内核时钟重新计算。 [FDCAN 时钟和引脚](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/Core/Src/fdcan.c#L76)

`Board_Init` 在生成配置之后装入掩码为 0 的标准 ID 滤波项，匹配的标准数据帧进 FIFO0，扩展帧和远程帧由全局设置拒绝。`FDCAN1_IT0_IRQHandler` 进入 HAL，再到 `HAL_FDCAN_RxFifo0Callback`。回调核对标准 ID、数据帧、8 字节长度和队列空间，先复制数据和接收时间，再更新写索引。后台每轮最多取八帧，调用协议解码和应用的接受函数。BSP 发送同样用标准 11 位 ID、Classic CAN、8 字节和 BRS Off。 [BSP 配置](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/bsp/board.c#L161) · [中断入口](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/Core/Src/stm32h7xx_it.c#L276) · [队列收发](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/bsp/board.c#L243) · [后台解码](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L611)

### 核对通信超时与停机

本工程看门狗在运行态检查最近一次有效命令是否超过 100 ms。断开命令流时，ISR 直接关桥并进入 Fault，后台再尝试报告。公开快照的离线 LAB5 Current 测试还复现了时间戳误判。三相 ADC 回调传入的是从 TIM1 计数回推的触发参考时刻。若触发参考为 103550 μs，随后在 103552 μs 接受有效命令，再执行该次 ADC 回调，`(uint32_t)(now - last_command_us)` 会得到 4294967294 μs，导致 `state=8`、`faults=0x08`、驱动关闭。该判断被运行模式共用，MIT、速度和标定在板上是否遇到同一窗口仍待确认。排查时分别记录触发参考、接收时间和回调实际执行时间。 [回推触发时间](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/bsp/board.c#L298) · [看门狗](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L494) · [验证记录](reference.md#验证记录)

**实物选做。** 使用 `LAB_STAGE=LAB_CAN_MIT`。按 [3.3 的启停和接线](02-open-loop.md) 先核对控制侧供电、收发器、终端电阻、限流、空载与独立关断手段。PD0、PD1 连接板上收发器的逻辑侧，适配器接总线侧 CANH、CANL。确认 Disabled、编码器健康、电零位有效，再从零刚度、小阻尼做短时检查。下面以 SLCAN 的 COM5 为例，实际端口以适配器为准。

```powershell
python tools/can_demo.py --send --interface slcan --channel COM5 stop
python tools/can_demo.py --send --interface slcan --channel COM5 cal-status
python tools/can_demo.py --send --interface slcan --channel COM5 mit --position 0 --velocity 0 --kp 0 --kd 0.05 --torque 0 --duration 2
```

记录进入状态、`0x003` 测量和退出状态。正常结束时主机会尝试发送 Disabled。若要单独验收失联保护，在受控条件下中断周期指令流，记录最后一份有效命令到驱动关断的间隔，并确认输出已关断。总线仍能通信时，再读取 `0x180` Fault，核对通信故障位。收到故障或发现运动异常时先确保功率已关断。排除故障原因后用 `clear` 请求回到 Disabled，再检查能否重新使能。实物结果待记录。 [状态切换与故障清除](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L116)
