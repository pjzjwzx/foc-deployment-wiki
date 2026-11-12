# 3.11 开环 LUT 标定

[上一节 3.10 CAN MIT](08-can-mit.md) · [课程首页](index.md)

电零点给出一个角度基准。整圈读数若仍有随位置变化的误差，可以用一张周期查找表修正。

## 1.1 工程问题与通路

设编码器在对齐位置读得正确，但转到某段角度时总是偏大。单个零位只能修正固定偏移。标定把电机带到一组已知的给定电角，记录稳定后的编码器读数，正反向各采一遍，再生成修正量。

~~~text
对齐 → 正反向扫描 → 生成整圈修正表 → 两个相邻表项之间插值
~~~

转到一圈末尾时，最后一个表项与第一个表项相邻。这样修正量可以跨圈连续变化。后台在扫描结束、功率输出关闭后保存标定结果。

## 1.2 高层代码领读

<span id="通用-foc-部署"></span>
<span id="通用部署"></span>

### 对齐、正扫、反扫

标定的角度参考来自开环给定。电流拍执行 `Calibration_FastStep`，把给定电角和缓慢上升的 Vd 送进已读过的 `Foc_VoltageStep`，Vq 为 0。达到对齐时间后，用固定目标角收集编码器样本。仅零位模式采完就生成零位候选。LUT 模式沿 32 个目标位置从 0 到 31，再从 31 沿原路回到 0，末点换向前在同一点重新静置并取样。 [标定状态推进](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/calibration.c#L37) · [给定电压输出](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L508)

| 阶段 | 电压、角度或样本怎样更新 | 本工程名义时间 |
|---|---|---:|
| RAMP | 电角固定在 0，Vd 从近零线性升到给定值 | 4000 个电流拍，200 ms |
| ALIGN | 保持对齐电压和初始角 | 10000 拍，500 ms |
| MOVE | 电角线性移到下一格，机械目标步距 `2π/32` | 1250 拍，62.5 ms |
| SETTLE | 在目标点保持，等待转子静置 | 6000 拍，300 ms |
| SAMPLE | 累积 256 份新鲜、有效的角度样本 | 样本连续为 4 kHz 时约 64 ms |

每格给定的电角差等于机械步距乘极对数。应用在 `Rotor_Update` 成功后，把方向归一化的原始计数交给标定器，所以本次标定的采样坐标与已有 LUT 无关。下面摘自 `Calibration_OnSample`。 [原始样本分发](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L382) · [收集样本](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/calibration.c#L86)

```c
if (c->state != CAL_SAMPLE)
    return;
/* 相对首样本展开，避免在65535/0附近直接平均得到半圈错误。 */
if (c->samples == 0)
    c->first_count = raw;
c->sum += (int64_t)c->first_count + delta(raw, c->first_count);
if (++c->samples < CAL_SAMPLES)
    return;
uint16_t mean = (uint16_t)(c->sum / (int64_t)CAL_SAMPLES);
```

`return` 让非采样阶段和未凑满的样本提前离开函数。每份样本先相对首帧求环形差，再加回首帧进入累计。若只用两点示意，首帧 65530、次帧 6 等价于展开域的 65530、65542，平均 65536 转回单圈为 0。仅零位模式将均值存为 `zero_count`，置 `zero_valid=1`、清 `lut_valid`。扫描模式分别将均值记到 `measured[pass][point]`，并将正扫起点保存为初始零位。

### 将扫描误差变成按原始计数等距的表

**源码摘录。** `Calibration_Process` 在循环中对每个目标点处理正扫和反扫的平均读数。 [扫描数据处理源码](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/calibration.c#L125)

```c
int32_t separation = delta(c->measured[1][k], c->measured[0][k]);
if (abs(separation) > 512)
    goto fail;
uint16_t mean = (uint16_t)(c->measured[0][k] + separation / 2);
int32_t ideal = (int32_t)c->initial_zero + (int32_t)(k * 65536U / FOC_LUT_SIZE);
int32_t error = delta((uint16_t)ideal, mean);
if (abs(error) > 2200)
    goto fail; /* 相序/方向错误或失步，不发布候选表 */
measured[k] = ideal - error;
correction[k] = error;
if (k && (measured[k] <= measured[k - 1]))
    goto fail;
```

`separation` 是反扫相对正扫的环形差，加一半到正扫值便得到环形平均。`ideal` 在展开域递增，理想与实测之间的最短差 `error` 是要加回读数的修正。用 `ideal - error` 将实测位置写回同一个展开域。若正反差超过 512 counts、修正绝对值超过 2200 counts 或展开后的实测位置未严格递增，`goto fail` 跳到函数末尾写失败状态并返回 `false`，跳过后续保存。

**手算输入。** 假定初始零位为 1024 counts，格点 0 的正反读数均为 1024。格点 1 正扫 3050、反扫 3054，平均 3052，理想位置 3072，得到 +20 counts。格点 2 正扫 5078、反扫 5082，平均 5080，理想位置 5120，得到 +40 counts。三组插值节点的坐标是 `(1024, 0)`、`(3052, 20)`、`(5080, 40)`，其中横轴为实测计数。

程序要用原始单圈计数直接索引表，所以再次插值，把修正值重采样到 0、2048、4096 等等距位置。补上的末节点使用首点计数加 65536，修正仍取首点值，首尾构成一圈。在上述输入中，2048 位于 1024 和 3052 之间，重采样修正为 `(2048 − 1024) × 20/2028`，整数除法得 10。4096 位于 3052 和 5080 之间，修正得 30。 [环形重采样](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/calibration.c#L150)

### 在运行时把表接回角度反馈

**源码摘录。** `Rotor_Update` 先做方向归一化，再把该单圈计数作为 `raw` 传给 `Rotor_CorrectRaw`。表的元素为有符号计数，可加也可减。 [运行时查表源码](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/control/rotor.c#L19)

```c
if (rotor == NULL || !rotor->lut_valid) return raw;
/* 65536/32=2048计数一个格点。末点与首点插值，整圈没有表边界断口。 */
const uint32_t step = BOARD_ENCODER_COUNTS_PER_TURN / FOC_LUT_SIZE;
uint32_t index = raw / step;
uint32_t next = (index + 1U) % FOC_LUT_SIZE;
float fraction = (float)(raw % step) / (float)step;
float correction = (1.0f - fraction) * (float)rotor->offset_lut[index] +
    fraction * (float)rotor->offset_lut[next];
return (uint16_t)((int32_t)raw + (int32_t)lroundf(correction));
```

整数除法选出左格，余数除以格长得到插值权重。`next` 用取模将第 31 格的下一点接回第 0 格。`lroundf` 将插值修正取整，加回原始计数，再转换成单圈无符号计数。无有效表时直接返回输入。

**手算对照。** 取同一个方向归一化计数 3072、零位 1024、21 极对，并令样本年龄为 0。用上面的假定表，3072 落在第 1 格正中，修正为 `0.5 × 10 + 0.5 × 30 = +20`，返回 3092。只标零位时仍用 3072。两条路径将计数换成机械角，减零位、乘极对数再折回一电圈，电角分别约为 4.16361 rad 与 4.12334 rad。差值约 0.04027 电 rad。这是手算输入，真实表值来自各次扫描。 [反馈电角与外推](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/control/rotor.c#L71)

这条处理链把 LUT 放在方向归一化之后、计数转机械角之前。第 31 格到第 0 格的插值也遵循同样的周期边界。修正后的计数也用于多圈位置和速度估计。 [位置和速度更新](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/control/rotor.c#L32)

**离线核对。** 用一张测试表令第 31 格修正为 `+6`、第 0 格为 `+2`。输入 `65535 → 0` 时，插值并取整的修正都是 `+2`，修正后单圈计数为 `1 → 2`。无表路径为 `65535 → 0`，两条路径按环形差计算都只前进 1 count。把测试表设为无效再重放，结果应回到无表路径。实机重新执行仅零位标定成功后，会发布新零位并清除 `lut_valid`，后续角度仍按新零位计算。 [查表有效位](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/control/rotor.c#L19) · [仅零位结果](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/calibration.c#L98)



### 核对扫描入口和采样配置

标定阶段和采样计数由应用与课程配置决定，电压上限来自电机配置。`accept_calibration` 要求 LAB4 或以上、Disabled、无故障、编码器连续 25 份有效样本、母线至少 2 V 和驱动器使能成功。START 和 ZERO 均可在没有旧零位的情况下开始。 [接受标定请求](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L241)

400 mV 示例的起始帧为 `0x124  02 01 01 90 00 00 00 00`。操作码 `02` 请求 LUT，`01` 是事务号，`01 90` 是大端 400 mV。接着主机安排每秒 50 次 STATUS 请求，在标定期间由应用用该请求续期。仅做零位时用 `cal-zero`，起始操作码为 `05`。 [协议与标志定义](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/communication/protocol.h#L19) · [标定命令构造](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/tools/can_demo.py#L267)

| `0x124` 操作码 | 主机命令 | 回包用途 |
|---|---|---|
| 1 | `cal-info` | profile、标定电压上限、点数 32、协议版本 2 |
| 2 / 5 | `cal-lut` / `cal-zero` | 启动整圈扫描或仅找零位 |
| 3 | `cal-status` | 状态、进度及有效标志 |
| 4 | `cal-read` | 停机时读取有效结果。索引 `0xFF` 为零位，0～31 为有符号表值 |

`0x181` 回显 `opcode | 0x80` 和事务号，再给结果码。STATUS 末字节 bit0 为 LUT 有效、bit1 为本次标定完成、bit2 为正在远程标定、bit3 为零位有效。复位后从有效位和读回值确认加载，本次完成位跟随本次上电的标定状态。公开快照与其他运行状态共用 CAN 看门狗，出现 `0x08` 时可参照上一节核对时间戳。 [回包字段](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L195) · [看门狗复现说明](08-can-mit.md#核对通信超时与停机)

## 2.1 BSP 与 CubeMX 领读

<span id="stm32-部署示例"></span>
<span id="stm32-实现"></span>

H743 沿用 [3.5 的 TIM1 与 ADC 触发](04-sampling-zero.md)、[3.4 的 SPI1 编码器](03-sensors.md) 和 [3.10 的 FDCAN1](08-can-mit.md)。这里跟读保存和恢复接口。

### 在关桥之后保存并验证

扫描到 `CAL_PROCESS` 时，电流 ISR 关桥。后台也先关桥并进入 Disabled，再执行下面的短路条件。处理失败时，`&&` 使 Flash 保存函数不被调用。两步都成功才在临界区发布新零位和 LUT，清除 `rotor.valid`，复位 PI。下一份有效样本重新建立位置和速度历史。 [电流拍关桥](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L508) · [后台保存](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L587)

```c
if (Calibration_Process(&calibration) && FlashStore_Save(&calibration.result))
{
    mask = Board_EnterCritical();
    rotor.zero_valid = calibration.result.zero_valid;
    rotor.lut_valid = calibration.result.lut_valid;
    rotor.electrical_zero_count = calibration.result.zero_count;
    memcpy(rotor.offset_lut, calibration.result.lut, sizeof(rotor.offset_lut));
    rotor.valid = false;
    Foc_Reset(&foc);
    Board_ExitCritical(mask);
}
```

H743 的链接脚本把程序 Flash 限为 1920 KiB，末尾从 `0x081E0000` 起留 128 KiB 给标定记录。BSP 核对 TIM1 的 MOE 和功率驱动器 EN_GATE 已关闭，擦除 Bank 2 Sector 7，把 160 字节记录写成五个 32 字节 Flashword，再读回解码并比较。上电时 `MotorApp_Init` 用同一读取接口恢复标定，记录校验检查标识、版本、配置键、有效位、CRC 和表值范围。改电机 profile、极对数、编码器方向或位数会改变配置键。当前只有一份记录，擦写中断电可能同时丢失原标定。换芯片要重新确认程序空间、擦除区域和写入粒度，且在 CubeMX 再生成后核对链接脚本的保留区。 [Flash 保存实现](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/bsp/flash_store.c#L11) · [链接布局](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/STM32H743VITX_FLASH.ld#L47) · [记录编码与校验](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/calibration.c#L176) · [启动加载](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L72)

## 2.2 LAB 实机操作与验收

### LAB4 及以上扫描与读回

先在配套工程目录用 `python tools/can_demo.py cal-lut --voltage-mv 400` 离线预览帧和轮询计划。

**实物选做。** 在已确认相序、角度方向与电流反馈的 LAB4 或以上工程中，使用空载、限流、有扫描空间及独立功率关断的条件。从 Disabled 读 `cal-info`，确认该电机允许的电压上限，再用已确认可稳定对齐的小电压扫描。下面以 SLCAN 的 COM5 和 400 mV 示意。

```powershell
python tools/can_demo.py --send --interface slcan --channel COM5 cal-info
python tools/can_demo.py --send --interface slcan --channel COM5 cal-lut --voltage-mv 400
python tools/can_demo.py --send --interface slcan --channel COM5 cal-status
python tools/can_demo.py --send --interface slcan --channel COM5 cal-read --index 0xFF
python tools/can_demo.py --send --interface slcan --channel COM5 cal-read --index 0
```

扫描通常持续数十秒。等待工具轮询完成，确认停机、零位与 LUT 有效，记录零位和至少一个表项。保持停机复位，再查有效标志、同一个表项和零位，核对它们与复位前一致。若要比较有表和仅零位的两份实物记录，先保留各次读回值，因为成功执行 ZERO 会保存新的零位并清除旧 LUT。出现异常运动、故障或失联，先确认功率已关断，再按失败阶段核对条件。实物结果待记录。 [验证记录](reference.md#验证记录)
