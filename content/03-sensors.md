# 3.4 电流与角度传感器

[上一节 3.3 电压开环](02-open-loop.md) · [下一节 3.5 采样、中断与电零点](04-sampling-zero.md)

上一节的电压开环已经能从给定电角和电压算出 duty。本节把 Simulink 中理想的三相电流和转子角换成实际测量。仍用给定电角驱动电压，只记录测得的量。手边没有板也可以用同一段模型波形做换算和回放，核对数据到了哪里。

## 通用 FOC 部署

### 从 ADC 计数还原三相电流

在 Simulink 里找出电机模块输出的 `ia、ib、ic`。这三条线通常已经是安培。把它们接到 MCU 时，先约定相序和正方向。本工程中数组下标 `0、1、2` 对应 A、B、C，进入控制算法前先把采样计数换成 A。 [应用采样入口](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L462-L493)

**源码摘录**。在 `MotorApp_CurrentIsr` 逐相处理当前帧，以下两行算电流并记录结果，中间的过流检查略去。

```c
phase[i] = ((float)raw[i] - (float)MotorApp_Diagnostics.adc_zero[i]) *
           BOARD_CURRENT_A_PER_COUNT * BOARD_CURRENT_SIGN;
MotorApp_Diagnostics.phase_A[i] = phase[i];
```

1. `raw[i]` 是这一相本拍的 ADC 计数。`adc_zero[i]` 是同一相在零电流时测到的基线。先转成 `float` 再相减，保留低于基线时的负值。
2. 对采用分流电阻和放大器的电路，电流先经分流电阻 `Rshunt` 变成电压，再经放大器增益 `G` 送进 ADC。换算系数为 `ADC 每计数的伏数 ÷ (G × Rshunt)`，单位是 A/count。最后乘接线约定的符号。
3. 结果既写到诊断量，也保留在 `phase[3]`，随后由 `Foc_Measure(&foc, phase, angle)` 读取。零偏完成后，任一相的电流绝对值超过所选电机配置的阈值会触发故障关桥。Open 分支仍用 `open_angle` 计算电压。 [换算后的控制去向](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L479-L524)

零偏来自上电后关桥的实际样本。应用进入 `APP_OFFSET`，每帧把三相分别加到 `offset_sum[i]`。达到 `FOC_CURRENT_OFFSET_SAMPLES` 后，分别用整数除法保存三相均值，切到 `APP_DISABLED`。本工程是 2048 帧，若每帧相隔 50 μs，累计窗口约为 102.4 ms。读波形时从完成偏置之后开始看。换板时从电路和 ADC 输出刻度重算比例，测量链在基线以外还要检查饱和、相序及正负方向。 [零偏状态与切换](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L462-L478) · [样本数](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/config/lab_config.h#L18-L33)

### 把转子角当作带时间的样本

先在模型中确认你拿到的是**电机轴机械角**。若模型给的是输出轴角，先考虑减速比。若模型给的是电角，找到乘极对数之前的那条线。机械角每转一圈走 `2π rad`。实际程序还需要知道样本是否有效、何时取得，以及正转计数向哪边走。

**源码摘录**。每拍从 `MotorApp_CurrentIsr` 进入 `encoder_tick`，以下是接收后进入更新的入口。

```c
EncoderSample_t sample;
Encoder_TakeSample(&sample);
if (sample.fresh)
{
    if (sample.valid && Rotor_Update(&rotor, sample.raw_u16, sample.timestamp_us))
```

`EncoderSample_t sample` 为本次调用留出一份结构体。`&sample` 把它的地址交给驱动，驱动通过地址写回数据。`fresh` 表示取到新完成的帧，`valid` 表示该帧可用。`&&` 会先检查左边，校验失败时跳过 `Rotor_Update`。此时结构体里的 `raw_u16` 可能仍是上次好帧的角，不能当成新位置。 [应用接收与健康检查](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L382-L413) · [驱动写回](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/bsp/encoder.c#L383-L446)

`Rotor_Update` 先检查时间戳，再把原始角统一成约定方向的全圈计数，随后把修正后的计数乘 `2π/65536`，得到机械弧度。没有启用 LUT 时，修正步骤直接返回方向归一化后的计数。`rotor->sample_time_us` 和 `rotor->valid` 随成功更新一起写入。应用每五个 20 kHz 电流节拍请求一次新角，名义间隔 250 μs。两次角样本之间仍会有电流帧。连续收到 8 个坏帧，或在运行时有效角缺失、样本年龄超过 10 ms，都会走编码器故障路径。 [角度方向与更新](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/control/rotor.c#L14-L68) · [调度分频](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/config/lab_config.h#L18-L32)

### 用模型波形做一次离线回放

从模型导出 `时间、ia、ib、ic、机械角`。先保留原图，再复制一小段数据，用你的 ADC 零偏、增益和编码器刻度生成假计数，按上面的代码换回。检查相电流差异是否处于量化误差量级，角度对比先统一方向并折回一圈。把一帧角标成无效，再走一次条件分支，观察旧角是否被保留。这里得到的是用模型量构造的回放，板上噪声、增益和方向由下面的实物步骤核对。

| 模型信号 | 进入应用时的量 | 转换后查看 |
|---|---|---|
| `ia、ib、ic`，A | `raw[0..2]`，各相采样计数 | `MotorApp_Diagnostics.phase_A[0..2]`，A |
| 单圈机械角，rad | `sample.raw_u16`，全圈统一刻度 | `rotor.raw_angle_u16` 已统一方向，`rotor.mechanical_angle_rad` 还包含可能的 LUT 修正 |
| 采样时刻，s | `sample.timestamp_us`，μs | `rotor.sample_time_us`，随最近有效角更新 |

## STM32 部署示例

### 电流通道和板级比例

H743 示例板的三个注入组分别取 A 相 PB1/ADC1 IN5、B 相 PA7/ADC2 IN7、C 相 PC2_C/ADC3 IN0。CubeMX 的 **Pinout & Configuration → Analog → ADC1/ADC2/ADC3** 中，先核对引脚、单端通道和注入 Rank 1。生成的 `MX_ADCx_Init` 分别写入 `ADC_CHANNEL_5`、`ADC_CHANNEL_7`、`ADC_CHANNEL_0`。三路结果进入 `raw[0..2]`。这一节先核对通道与量纲，下一节再按一整拍讲触发和中断。 [本板 .ioc](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/FOC_Arclab_H743_V3.ioc) · [三路 ADC 配置](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/Core/Src/adc.c#L35-L249) · [读取三路结果](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/bsp/board.c#L307-L322)

`board_config.h` 使用 3.0 V 参考、0.5 mΩ 分流电阻和外置 INA241A3 的 50 V/V 增益。14 bit 的单次转换最大码为 16383，注入结果以 16 次累加后右移两位，满刻度是 `16383 × 16 ÷ 4 = 65532 counts`。因此程序使用 `3.0 ÷ (65532 × 50 × 0.0005) ≈ 0.00183117 A/count`，并设符号为 `+1`。启动中的 `HAL_ADCEx_Calibration_Start` 校准 ADC 本身，应用稍后求的 `adc_zero[]` 包含整条电流测量链的基线。 [板级参数](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/config/board_config.h#L7-L22) · [ADC 硬件校准](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/bsp/board.c#L144-L160)

### TLE5012 角度怎样交给应用

本例用 SPI1 单数据线发命令、收角度。DMA 在外设和内存间搬运字节，应用不在电流回调中等待整帧传完。驱动请求时记下 MCU 微秒时间并拉低片选，TX 完成后切换为 RX，接收角字段和安全字。之后取样入口校验 CRC、状态和角字段，把结果和请求时间戳交给应用。故障恢复期间也可能收到一帧但没有可用新角。 [发起请求](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/bsp/encoder.c#L250-L316) · [TX、RX 事务](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/bsp/tle5012.c#L397-L469) · [角字段与有效位](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/bsp/tle5012.c#L483-L549)

在 CubeMX 打开 **Connectivity → SPI1**，再按表回查对应的生成代码。换传感器时先看其通信方式、供电、电平与引脚表，再按需要实现同样的角度样本接口。

| 核对位置 | 本工程取值与用途 | 生成代码 |
|---|---|---|
| Pinout | PB3 是 SCK，PD7 是单线数据，PA15 是软件控制的片选，空闲高 | [SPI 引脚复用](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/Core/Src/spi.c#L141-L159) · [PA15 初值](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/Core/Src/gpio.c#L54-L70) |
| SPI1 Parameter Settings | Master、1 Line、8 Bits、CPOL Low、CPHA 2 Edge、MSB First、Software NSS。SPI123 时钟 48 MHz，除 8 得 6 MHz 位时钟 | [SPI1 参数](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/Core/Src/spi.c#L42-L64) · [时钟设置](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/FOC_Arclab_H743_V3.ioc#L466-L466) |
| SPI1 DMA Settings | RX 用 DMA1 Stream3，TX 用 Stream4，Normal 模式，内存和外设均按 Byte 搬运 | [DMA 初始化与句柄关联](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/Core/Src/spi.c#L161-L196) |
| NVIC Settings | SPI1 和 DMA1 Stream3/4 的抢占优先级都是 2，低于电流采样 IRQ 的 1 | [SPI1 NVIC](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/Core/Src/spi.c#L198-L200) · [中断配置](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/FOC_Arclab_H743_V3.ioc#L276-L292) |

角字段有效后，驱动取低 15 位的角度字段并左移一位，让一圈采用统一的 `uint16_t` 刻度。应用里的 `BOARD_ENCODER_REVERSE=1` 再用模一圈的减法统一方向。6 MHz 描述一次 SPI 事务内的位速率，4 kHz 描述应用名义请求频率。驱动若忙，实际间隔要看记录的时间戳。 [TLE5012 角度掩码](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/bsp/tle5012.c#L19-L26) · [角方向](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/control/rotor.c#L14-L17)

**手算检查**

1. 一相零偏为 32768、当前原始码为 32668，得到多少 A。**答案**　差值 −100 counts，乘本板系数后约 −0.183 A。
2. TLE5012 的有效角字段为 `0x6000`，开启反向且未启用 LUT，得到什么机械角。**答案**　统一刻度为 `0xC000`，反向为 `0x4000`，即四分之一圈 `π/2 rad`。
3. 新帧的 `fresh=1`、`valid=0`，如何更新机械角。**答案**　不调用 `Rotor_Update`，保留上次有效角，并累计坏帧。手转波形停住时，应同时查有效帧和样本年龄。

### 实物选做｜先关桥手转，再短时开环

1. 以 `LAB_STAGE=3` 构建并下载，按上一节的 UART4 接法观察 JustFloat 波形。上电后保持 Disabled，等零偏采集完成。在调试器中看 `MotorApp_Diagnostics.adc_zero[0..2]`，对照波形 ch0、ch1、ch2 应在 0 A 附近。若某相原始读数一直贴近量程端点，或关桥后仍有明显偏置，先停止并查引脚、前端和比例。 [工具与构建](00-project.md) · [UART4 波形观察](02-open-loop.md#uart4-波形观察)
2. 保持桥关，在机械装置允许手转时慢慢沿规定的正方向转轴。观察 ch3 的单圈机械角是否上升，以及跨圈时是否从接近 `2π` 回到 0。注明手转的是电机轴还是输出轴。调整方向后需重新构建，并重做角度标定。
3. 按上一节的低电压短时开环步骤试转，记录相序、实际转向和三相电流波形。三相和接近零可以作为一致性检查，再用引脚映射和下一节的坐标验算确认各相。LAB3 的 ch4 是由测量角算出的电角，ch5 是开环给定电角。完成电零点以前，先观察变化方向和连续性。若有故障、转子卡住或电流明显异常，立即停机。 [LAB3 六通道映射](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L414-L440)
