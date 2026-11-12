# 3.5 采样、中断与电零点

[上一节 3.4 电流与角度传感器](03-sensors.md) · [下一节 3.6 坐标变换与开环验算](05-transforms.md)

把模型的离散控制放到 MCU 上，先回答一拍怎样开始、反馈何时可用、算出的 duty 何时生效。这节先走完一拍，再回查 CubeMX。最后用一次低电压对齐建立电角基准，下一节仍在电压开环下核对 Clarke 和 Park。

## 通用 FOC 部署

### 给离散控制的一拍标出三个时刻

在 Simulink 里打开一段模型波形，把时间放大到几拍。假设控制步长是 50 μs，用竖线标出 `kTs`、`(k+1)Ts`。模型在一拍里取反馈并给出新 duty。实际 MCU 要为这一步留出采样、转换、响应和计算的时间。先标出三个时刻，再核对模型是否也考虑了对应延迟。

| 事件 | 对应的模型量 | 部署时要确认 |
|---|---|---|
| 触发采样 | 本拍输入的时间标签 | 由定时事件选在载波的指定位置，确认电流测量窗口与开关边沿的关系 |
| 三相结果齐备 | 可供算法使用的 `iabc[k]` | 等本组结果完成再调用控制。传感器更新较慢时，按时间戳选取反馈角 |
| 新比较值锁存 | duty 开始作用到功率级的时刻 | 计算结束先提交，定时器在更新事件采用预装载值。模型需要研究延迟时，按这个生效相位设置输出保持或延时 |

**执行示意**。图中只有顺序，间距不代表实测时间。

~~~text
触发 k ── 采样/转换 ── 结果齐备 ── 计算并提交 duty[k] ── 比较值生效 ── 触发 k+1
  │                         │                 │
  │                         │                 └─ 先写入预装载寄存器
  │                         └─ 读同一组电流与可用角度
  └─ 硬件为这一拍建立时间参考
~~~

控制代码把一次完整采样当成一次函数调用。`MotorApp_CurrentIsr(const uint16_t raw[3], uint32_t now)` 的 `raw` 是板级交来的三相计数，`now` 在这里是这组采样的触发参考时间。`const` 表示应用只读这份计数。函数先推进节拍、更新编码器并换算三相电流，再用本拍时间求反馈角。下面是它接上测量算法的几行。 [控制入口与测量链](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L462-L493)

**源码摘录**。

```c
float angle = rotor.valid ? Rotor_ElectricalAngle(&rotor, now) : 0.0f;
if (!isfinite(angle))
    angle = 0.0f;
if (!Foc_Measure(&foc, phase, angle))
    MotorApp_FaultIsr(APP_FAULT_MATH);
```

第一行的 `条件 ? 真值 : 假值` 用最近的有效角计算这一拍的电角。当前角无效或计算返回无效数值时，测量路径用 0 继续记录诊断。运行能否继续还受 `encoder_tick` 的健康与超时检查约束。最后两行将三相电流和角度交给 `Foc_Measure`，函数返回失败就记数学故障。之后由运行状态决定是否计算新的 duty。Open 用给定电角做逆 Park 和调制，算完才走板级提交。该提交失败也会进入故障处理。 [编码器保护](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L382-L413) · [Open 到 duty 提交](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L519-L549)

换 MCU 时，保留“拿到一组有效电流和采样时刻再运行一次控制”这个约定。硬件可以用转换完成中断、DMA 完成或其他方式交付结果。若目标硬件的通道依次采样，还要在接入前检查相间取样偏移。更换载波或触发配置后，重新确认每载波触发几次、控制实际周期和预装载截止点，再更新模型步长及控制代码里的 `dt`。 [示例的固定控制周期](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/config/lab_config.h#L18-L29)

### 给真实电角定义零点

模型知道转子磁场指向。实物编码器的机械零刻度由安装位置决定。把定子电压矢量固定在程序约定的电角 0，给 d 轴小电压且 q 轴为 0，等待能自由转动的转子对齐并静止。此时的机械角记为 `θ0`。完成对齐后，若不考虑样本延迟，反馈电角就是 `wrap((θm − θ0) × p)`，其中 `p` 是极对数。

电流零偏与电零点各减在自己的量上。电流减 ADC 基线后乘 A/count，电角先减对齐得到的机械角再乘极对数。示例把电零点以机械计数保存在 `electrical_zero_count`，进公式时才换成机械弧度。角样本比这拍电流更早时，还会在机械角上加 `速度 × 样本年龄`，然后减零位、乘极对数、折回一圈。 [电角计算](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/control/rotor.c#L71-L80)

**手算输入**。机械零位为 1 rad，最近机械角为 1.1 rad，样本比电流触发早 200 μs，机械速度 10 rad/s，电机有 21 极对。**答案**　先外推 `10 × 0.0002 = 0.002 rad`，再算 `(1.1 + 0.002 − 1) × 21 = 2.142 rad`，已在一圈以内。对应 Simulink 时可以先令速度为 0 对齐基准，再恢复速度项。

## STM32 部署示例

### 先跟代码走完 H743 的一拍

在 [3.2 的 TIM1 计数图](01-pwm-svpwm.md) 上沿 CNT 看一圈。从底端上数到 1 时，CH4 的 OC4REF 产生下降沿，经内部 TRGO 触发三个 ADC 的注入转换。到达顶端约需 25 μs，再下数约 25 μs 回到底端。ADC 选下降沿，所以一整个 50 μs 载波只取一组触发。 [TIM1 触发路由](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/Core/Src/tim.c#L45-L99) · [ADC 注入边沿](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/Core/Src/adc.c#L82-L101)

**示意时间轴**。以载波底端为 0，表中的数字来自配置或手算，ADC 与控制运行的具体耗时需实测。

| 从底端起 | 硬件与程序经过哪里 |
|---|---|
| 上数到 1，约 0.004 μs | CH4 下降沿经 TRGO 启动三路注入。该位置是代码用来回推的触发参考 |
| 注入结果陆续完成 | ADC1/2 共用 `ADC_IRQHandler`，分别调用两个句柄的 `HAL_ADC_IRQHandler`。ADC3 的 `ADC3_IRQHandler` 调用 ADC3 句柄。HAL 读取各自完成标志，进入同一个 BSP 完成回调 |
| 上数且 CNT 小于 3000，约 12.5 μs 内 | 回调的三位标志都置上后读取三路 `JDR1`，确认完成窗口，清位图并调用一次应用 |
| 上数且 CNT 小于 5520，约 23 μs 前 | 运行分支完成控制，`Board_ApplyDutyIsr` 核对 duty、驱动和计数相位后把三相写入 CCR 预装载。超过提交窗口会返回失败，应用走故障关桥 |
| 顶端 CNT 为 6000，约 25 μs | TIM1 更新事件采用新比较值。提交检查在顶端前留了约 2 μs |
| 下数到底，约 50 μs | 进入下一完整载波。若期间没有新的写入，比较值继续保持 |

[两条 IRQ 入口](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/Core/Src/stm32h7xx_it.c#L261-L343) · [BSP 注入完成回调](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/bsp/board.c#L298-L323) · [提交检查与写入](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/bsp/board.c#L104-L135)

**源码摘录**。完成回调先记录是哪路 ADC 报到。

```c
uint8_t bit = h == &hadc1 ? 1U : h == &hadc2 ? 2U : 4U;
if (adc_done_mask & bit)
{
    adc_done_mask = 0;
    MotorApp_FaultIsr(APP_FAULT_SAMPLE);
}
adc_done_mask |= bit;
```

`h` 是 HAL 交给回调的 ADC 句柄地址。`1U、2U、4U` 分别是 `001、010、100`，`&` 检查这一位原来有没有置上，`|=` 保留其他位并置上本位。假设完成顺序为 ADC2、ADC1、ADC3，位图依次为 `010 → 011 → 111`，到 7 才进入读数和计算。如果 ADC2、ADC1 之后又到 ADC2，第三次发现重复，清旧位图并报告采样故障，再记录新 ADC2，仍未到齐。 [位图汇合](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/bsp/board.c#L298-L310)

**源码摘录**。以下为三相齐备后的时间检查和调用。

```c
uint32_t now = BSP_GetTimeUs(), count = TIM1->CNT;
if ((TIM1->CR1 & TIM_CR1_DIR) || count >= TIM1->ARR / 2U)
{
    adc_done_mask = 0;
    MotorApp_FaultIsr(APP_FAULT_SAMPLE);
    return;
}
uint32_t trigger_time =
    now - (count > 1U ? (count - 1U) / (BOARD_TIM1_CLOCK_HZ / 1000000U) : 0U);
adc_done_mask = 0;
MotorApp_CurrentIsr(raw, trigger_time);
```

`TIM1->CNT` 通过寄存器地址读取计数，`DIR` 位为 1 表示下数。处在下数段或已经到达上数 3000 格，都会报故障并在调用应用前返回。窗口通过后，以每微秒 240 个计数回推上数到 1 的时刻。若软件时钟读到 12 345 μs，计数为 1201，手算得到 `(1201−1)/240 = 5 μs`，交给应用的是 12 340 μs。两次寄存器读数有先后，整数除法也会舍去余数。这个结果供反馈角对齐时间，实际 ADC 模拟取样窗口和转换完成时刻仍须各自核对。应用返回后，回调返回 HAL，HAL 完成本次中断处理，再回到中断前的代码。 [回推与调用](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/bsp/board.c#L307-L323)

### 再回查 CubeMX 和生成代码

在 **Pinout & Configuration** 展开 **Timers → TIM1** 和 **Analog → ADC1/ADC2/ADC3**。按触发、接收、完成通知的顺序核对，再到生成的 C 文件找到对应赋值。 [示例 .ioc](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/FOC_Arclab_H743_V3.ioc)

| CubeMX 设置 | 本工程取值 | 生成代码 |
|---|---|---|
| TIM1 Counter Settings、Channel 4 | 240 MHz，PSC=0，中心对齐 3，ARR=6000，RCR=0。CH4 用 PWM1、Pulse=1，只作内部触发 | `Period = PWM_COUNT-1`、`RepetitionCounter = 0`、CH4 的 `Pulse = 1`。[TIM1 初始化](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/Core/Src/tim.c#L45-L99) |
| TIM1 Trigger Output | TRGO 选 OC4REF，将 CH4 参考信号送给 ADC | `MasterOutputTrigger = TIM_TRGO_OC4REF`。[TRGO 赋值](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/Core/Src/tim.c#L65-L67) |
| ADCx Injected Conversion | 三个 ADC 均启用单端 Injected Rank 1，每组一次转换序列，外部触发 Timer 1 TRGO、Falling Edge。通道依次为 IN5、IN7、IN0 | `InjectedNbrOfConversion = 1`、`ADC_EXTERNALTRIGINJEC_T1_TRGO`、`ADC_EXTERNALTRIGINJECCONV_EDGE_FALLING`。[ADC1](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/Core/Src/adc.c#L82-L100) · [ADC2](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/Core/Src/adc.c#L161-L179) · [ADC3](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/Core/Src/adc.c#L227-L245) |
| ADCx 采样和过采样 | ADC 内核 36 MHz，分辨率 14 bit，单次取样时间 2.5 ADC 周期。注入组累加 16 次，结果右移 2 位 | `ADC_RESOLUTION_14B`、`ADC_SAMPLETIME_2CYCLES_5`、`Ratio = 16`、`ADC_RIGHTBITSHIFT_2`。[ADC1 配置](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/Core/Src/adc.c#L50-L100) · [ADC 时钟](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/FOC_Arclab_H743_V3.ioc#L399) |
| NVIC Settings | 启用 ADC1/2 共用中断与 ADC3 中断，抢占优先级都为 1 | `HAL_NVIC_SetPriority(ADC_IRQn, 1, 0)`、`HAL_NVIC_SetPriority(ADC3_IRQn, 1, 0)` 和各自的 `HAL_NVIC_EnableIRQ`。[NVIC 代码](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/Core/Src/adc.c#L286-L338) |

ADC 的 2.5 个时钟周期描述单次模拟取样时间，之后还要转换并累加。本例 ADC1 的 EOC 选择为序列完成，HAL 的注入中断使用 JEOS。ADC2/3 的 EOC 选择为单次完成，注入中断使用 JEOC。三个注入组各只有一个 Rank，两种完成通知都能标记本相本拍结果。配置生成后还要启动。搜索 `Board_Start`，它先用 `HAL_ADCEx_InjectedStart_IT` 为三路注入组准备中断和触发，再启动 TIM1 三相与 CH4。任一 HAL 调用失败就返回 `false`，由主函数进错误处理。 [板级启动](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/bsp/board.c#L175-L192) · [主函数](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/Core/Src/main.c#L73-L135)

母线电压也在 ADC1，但接 PC5/IN8，使用规则组 Rank 1、软件启动、64.5 ADC 周期采样。后台每隔至少 1000 μs 启动一次，到规则 EOC 才读结果并换算伏数。代码用 `HAL_ADCEx_RegularStop` 结束规则组，让注入组继续接受 TIM1 触发。母线数据超过新鲜窗口后 `Board_BusVoltage()` 返回 0。三相注入结果和母线规则结果有各自的寄存器与读取路径。 [规则组配置](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/Core/Src/adc.c#L50-L114) · [后台母线读取](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/bsp/board.c#L194-L225)

### 用 LAB4 标定电零点

在配套固件目录先运行离线命令。它打印开始帧 `0x124  05 01 01 90 00 00 00 00`，其中 `05` 选单点对齐，`01 90` 表示 400 mV。

```powershell
python tools/can_demo.py cal-zero --voltage-mv 400
```

应用收到请求后检查阶段至少为 LAB4、当前 Disabled、无故障、编码器已有足够有效帧、母线电压与标定电压满足条件。驱动准备成功后调用 `Calibration_Start`，本次 `lut` 参数为假。应用随后发布 `APP_CALIBRATION` 状态。电流中断每拍推进 `Calibration_FastStep`。它用 4000 拍在 0.2 s 内将 d 轴电压爬到目标，保持电角 0，等待 10 000 拍即 0.5 s，再开始接收 256 个有效新角。按 4 kHz 名义频率计，取样约需 64 ms。此时的 `Foc_VoltageStep` 用 d 轴对齐电压、q 轴 0 V。 [请求检查](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L241-L284) · [标定状态机](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/calibration.c#L10-L83) · [应用输出电压](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L508-L517)

**源码摘录**。每次收到有效新角，`Calibration_OnSample` 在收样阶段累计方向归一化后的计数。以下省略第一帧设置 `first_count` 的分支，后面的 LUT 分支留到专题。

```c
c->sum += (int64_t)c->first_count + delta(raw, c->first_count);
if (++c->samples < CAL_SAMPLES)
    return;
uint16_t mean = (uint16_t)(c->sum / (int64_t)CAL_SAMPLES);
if (!c->lut_mode)
{
    c->result.zero_count = mean;
    c->result.zero_valid = 1;
    c->result.lut_valid = 0;
    c->state = CAL_PROCESS;
    return;
}
```

`delta` 求相对于第一帧的环形短差。`++c->samples` 先加一再比较，前 255 帧累加后就返回。第 256 帧得到整数平均，保存为机械零位，标记电零点有效和 LUT 无效。后台见到 `CAL_PROCESS` 就先关桥，再处理、保存并读回验证。成功后才把结果交给 `rotor`，并等一个新角重新建立反馈。失败时记录标定故障。下一节用已标定电角检查 Park，完整扫圈和查表放到 [LUT 专题](09-lut.md)。 [圆周差与单点结果](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/calibration.c#L19-L105) · [关桥和保存](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L587-L609) · [Flash 读回](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/bsp/flash_store.c)

**离线过关**。把 65534、0、2 当成仅用于演算的三个对齐样本。相对首点展开后为 65534、65536、65538，平均为 65536，折回全圈计数后是 0。再回看本页时间轴，指出为什么计算频率可为 20 kHz，且在一拍的上数半程内就要完成提交。

### 实物选做｜先验证节拍，再对齐

1. 用 `LAB_STAGE=4` 构建并下载。保持 Disabled，先核对上一节的电流零偏和编码器。以独立计时观察 `MotorApp_Diagnostics.current_ticks`，正常应约每秒增加 20 000。若有示波器，可用另加的 GPIO 探针观察 IRQ 和提交相对载波的时刻。诊断量 `max_isr_cycles` 记录应用控制函数内部耗时，整条链还包含采样和中断入口开销。断点会暂停程序，计时记录应在连续运行时取得。 [诊断字段](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.h#L29-L42)
2. 确认电机配置、机械自由转动条件、空载、母线和停机方法。按 [3.3 的 CAN 步骤](02-open-loop.md) 选适配器，例中把 `COM5` 换成实际端口。标定中有异常电流、故障或机械卡滞时立即停机并检查。

   ```powershell
   python tools/can_demo.py --send --interface slcan --channel COM5 --bitrate 1000000 cal-zero --voltage-mv 400
   ```

3. 等脚本轮询完成并停机。查 `cal-status`，确认零位有效、LUT 无效。可再运行 `cal-read --index 0xFF` 读取零位，重启后核对是否恢复。0.4 V 是本页的请求示例。实际能否对齐与供电、摩擦、负载及电机配置有关，本页没有实测波形。
