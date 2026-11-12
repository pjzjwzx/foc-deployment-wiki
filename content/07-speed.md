# 3.9 速度环与多速率调度

[上一节 3.8 固件分层与移植](06a-software-architecture.md) · [下一节 3.10 CAN MIT](08-can-mit.md)

速度环从编码器样本估速度，再用速度误差给电流环提供 Iq 目标。不同任务可以各用自己的节拍。

## 1.1 工程问题与通路

假设上一份角度是 359°，下一份是 1°。按正方向只转了 2°，所以要先处理跨圈，再除以两次有效采样的时间差。滤波后的速度参与速度 PI，产生受限的 Iq 参考。

~~~text
编码器新样本 → 展开角度差 ÷ 时间差 → 速度反馈
速度目标 − 速度反馈 → 较慢的速度 PI → 保持 Iq 参考 → 较快的电流 PI
~~~

新角度、外环执行和电流反馈各有自己的时刻。在两次外环更新之间，电流环持续用最近一次 Iq 目标和每拍的新电流。不同测速方法的比较见[第四章](chapter-4.md)。

## 1.2 高层代码领读

<span id="通用-foc-部署"></span>
<span id="通用部署"></span>

### 由角度样本估计电机轴速度

测速从相邻两份有效角度样本开始。单圈计数每满 65536 回到 0，程序须先处理跨圈，再将计数差乘以 `2π/65536` 转为机械 rad。下面摘自 `Rotor_Update`，`corrected` 是当前样本经过方向归一化和可选 LUT 后的计数。 [样本处理源码](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/control/rotor.c#L32)

```c
int32_t delta_count = (int32_t)corrected - (int32_t)rotor->corrected_angle_u16;
if (delta_count > 32767) delta_count -= 65536;
if (delta_count < -32768) delta_count += 65536;
float delta = (float)delta_count * (FOC_TWO_PI / 65536.0f);
rotor->multi_turn_angle_rad += delta;
```

转换成有符号 32 位再减，使负差可以直接表示。两个 `if` 把差值折到 −32768～32767 counts，要求相邻有效样本的真实位移绝对值小于半圈。多圈位置在上次结果上加这个机械角差。首份有效样本用本次机械角初始化多圈位置，速度初值为 0。

**手算输入。** 上一份修正计数为 65530，本次为 6。直接相减为 −65524，折回后为 +12 counts，即约 0.0011505 rad。假定间隔 250 μs，瞬时速度约 +4.602 rad/s。代码取的是两个样本的时间戳之差，传感器更新有抖动或漏帧时，不能用固定的 250 μs 代入每一次测速。

**源码摘录。** 在有效样本分支里，间隔在允许范围内才进入下面的计算。 [速度滤波源码](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/control/rotor.c#L47)

```c
float dt = (float)elapsed_us * 0.000001f;
float raw_speed = delta / dt;
float tau = 1.0f / (FOC_TWO_PI * FOC_SPEED_FILTER_CUTOFF_HZ);
float alpha = dt / (tau + dt);
rotor->speed_rad_s += alpha * (raw_speed - rotor->speed_rad_s);
```

`elapsed_us` 以微秒计，乘百万分之一得到秒。`tau` 是一阶低通的时间常数，截止频率为 100 Hz，本例 `alpha≈0.136`。若上一帧滤波速度为 0，这一帧更新到约 0.625 rad/s。`rotor->speed_rad_s` 保留上一份滤波速度，供下次更新使用。重复或逆序时间戳返回 `false`。若新样本间隔超过 10 ms，速度清零，从新的时刻恢复测速。 [时间戳与首帧处理](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/control/rotor.c#L35) · [滤波与超时配置](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/config/lab_config.h#L18)

电流环可能在两份编码器样本之间执行。`Rotor_ElectricalAngle` 先用 `当前拍时间 − 样本时间` 得到样本年龄，随后用滤波速度乘年龄并加到最后一份机械角，最后减机械零位、乘极对数并折回一电圈。年龄超过 10 ms 时函数返回 `NAN`，应用的编码器健康检查会在运行态触发关断。速度反馈在有效新样本到来时更新，外推电角可以每个电流拍重新计算。 [电角外推源码](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/control/rotor.c#L71) · [编码器健康检查](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L382)

### 速度 PI 更新 Iq，电流 PI 每拍执行

速度参考与反馈均用**电机轴 rad/s**。本例速度环 200 Hz，每 5 ms 更新一次 Iq，电流环则每 50 μs 使用最近的 Iq 参考。模型里可用不同速率的离散子系统实现这段关系，较慢一侧的输出在两次更新之间保持。

**源码摘录。** `Foc_SpeedStep` 已检查有限值，并把速度参考和 Iq 上限约束到电机配置。下面是计算和写回部分。 [速度 PI 源码](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/control/foc.c#L167)

```c
float error = reference - feedback;
float next = foc->speed_pi.integral + foc->speed_pi.ki * FOC_OUTER_DT_S * error;
float raw = foc->speed_pi.kp * error + next;
if (!isfinite(next) || !isfinite(raw)) return false;
float limited = Clamp(raw, -limit, limit);
if (error * (raw - limited) <= 0.0f) foc->speed_pi.integral = Clamp(next, -limit, limit);
foc->iq_limit_A = limit;
foc->iq_ref_A = Clamp(foc->speed_pi.kp * error + foc->speed_pi.integral, -limit, limit);
foc->id_ref_A = 0.0f;
```

`next` 是候选积分。未饱和时允许写回，饱和时只允许误差推动输出回到限幅区的方向。最终用实际保留的积分计算 Iq，再限幅。若误差为 2 rad/s、积分初值为 0、限流为 0.5 A，用本例 Kp=0.1、Ki=0.1 及 5 ms 步长，首个未饱和外环拍得到 0.201 A。外环返回 `false` 时，应用短路跳过该拍的 `Foc_CurrentStep`，随后按采样故障处理。 [速度环参数](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/config/motor_config.h#L53) · [外环调用与 PWM 提交](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L525)

**离线对照。** 在同一时间轴记录机械计数、样本时间戳、瞬时速度、滤波速度、速度参考和 Iq 参考。先确认跨圈处的速度符号，再查看 Iq 只在 5 ms 的外环节拍变化。将速度参考阶跃改到足以触发 Iq 限幅，检查积分是否停止向饱和方向增长。板上的速度跟踪还需对照 Iq 反馈，才能区分外环给定和电流环响应。

## 2.1 BSP 与 CubeMX 领读

<span id="stm32-部署示例"></span>
<span id="stm32-实现"></span>

### 在 H743 的同一套中断里安排不同节拍

速度环沿用 [3.5 的 TIM1 与 ADC 注入触发配置](04-sampling-zero.md)。三相样本就绪后进入 `MotorApp_CurrentIsr`，每次把 `divider` 加一。SPI1 采样沿用 [3.4 的编码器配置](03-sensors.md)，BSP 把新样本和时间戳提供给应用。CubeMX 配置触发和中断，速度环分频在应用中用取模实现。 [控制 ISR](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L462) · [SPI 样本读取](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/bsp/encoder.c#L383)

| 事件 | 触发条件 | 名义周期 |
|---|---|---:|
| 电流计算、尝试取已完成的编码器样本 | 完整三相 ADC 帧 | 50 μs |
| 请求下一份编码器样本 | `divider % 5 == 0` | 250 μs |
| 更新速度环 Iq | Speed 状态且 `divider % 100 == 0` | 5 ms |
| 发布 UART 波形快照 | `divider % 20 == 0` | 1 ms |

**执行示意。** 在 Speed 状态截取外环更新边界前后的几拍。表中的 `n` 是能被 100 整除的拍号，每往右一列经过 50 μs。假设请求的 SPI 样本在后一拍可读。实际取到有效样本的时刻需要看时间戳。

| 事件 | 第 `n−1` 拍 | 第 `n` 拍 | 第 `n+1` 拍 | 第 `n+2` 拍 |
|---|---|---|---|---|
| 编码器 | 检查是否有新样本 | 检查样本，再请求下一份 | 取到新样本，更新估计速度 | 检查是否有新样本 |
| 速度 PI | 沿用上次 Iq 参考 | 用已取得的速度更新 Iq 参考 | 沿用第 `n` 拍参考 | 沿用第 `n` 拍参考 |
| 电流 PI | 用本拍反馈计算 | 用本拍反馈和新参考计算 | 用本拍反馈和保持的参考计算 | 用本拍反馈和保持的参考计算 |

编码器请求与有效样本更新是两个事件。只有 `sample.fresh`、`sample.valid` 且 `Rotor_Update` 成功才更新速度历史。板端 UART4 的 LAB6 六通道依次为电机轴速度参考、估计速度、Iq 参考、Iq 反馈、输出轴位置、反馈电角。UART 快照适合看趋势，不能据此逐拍测量 50 μs 的控制时序。 [请求与取样](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L382) · [UART 通道](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L451)

## 2.2 LAB 实机操作与验收

### 核对速度命令与停机

在配套工程根目录执行下列离线命令，查看主机将速度和运行限流打包成的 `0x122` 帧。该例数据为 `80 40 28 00 00 00 00 00`，板端解码约为 0.829 rad/s 和 0.488 A。量化后的参考才是应与实测反馈对照的值。 [速度命令解码](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/communication/protocol.c#L45)

```powershell
python tools/can_demo.py speed --speed 1 --iq-limit 0.5 --duration 2
```

**实物选做。** 按 [3.3 的构建、下载与 CAN 启停步骤](02-open-loop.md) 使用 `LAB_STAGE=LAB_SPEED`，先确认第 3.7 节电流环和电零位已验收，再在空载、限流、有独立功率关断手段时做短时试验。确认 Disabled 和零位有效后，对上面的命令按适配器补齐子命令前的发送参数。先记正转的速度参考、反馈和 Iq，再停机确认 Disabled，随后用小负速度重复。若反馈方向与参考相反或 Iq 持续饱和，先停机核对编码器方向、相序和电零位。硬件记录待完成。

**读波形。** 对照正转和反转两段日志。在方向约定一致时，稳态估计速度应与已解码的目标同号。先看 Iq 参考是否长期顶到运行限值，再看 Iq 反馈是否跟上。若 Iq 跟踪良好但速度误差仍大，结合限流和负载判断外环给定是否足够。若 Iq 反馈偏离参考，回到 3.7 检查电流环。

公开快照在电流模式的离线测试中复现过 CAN 超时误判。同一个看门狗也用于 Speed，收到 `0x180` 的 Fault 状态和 `0x08` 故障位时先保持停机，按下一节的时间戳顺序排查。 [看门狗判断](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L489) · [验证记录](reference.md#验证记录)
