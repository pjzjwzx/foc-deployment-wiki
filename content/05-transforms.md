# 3.6 坐标变换与开环验算

[上一节 3.5 采样、中断与电零点](04-sampling-zero.md) · [下一节 3.7 PI 电流闭环](06-current-pi.md)

## 通用 FOC 部署

### 用一组输入核对 Clarke 和 Park

在 Simulink 里，把一段三相电流和同一时刻的转子电角接到 Clarke、Park。先记下四个输入的单位和正方向。示例固件每次收齐一组电流采样，也把这四个量送给同一条变换链。

**源码摘录**。输入通过检查后，`Foc_Measure` 先保存三相电流，再执行下面三句。函数开头的检查和复制省略。 [Foc_Measure](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/control/foc.c#L78)

~~~c
Foc_Clarke(phase_A, &foc->alpha_current_A, &foc->beta_current_A);
Foc_Park(foc->alpha_current_A, foc->beta_current_A, angle, &foc->id_A, &foc->iq_A);
return isfinite(foc->id_A) && isfinite(foc->iq_A);
~~~

- `phase_A` 按 A、B、C 顺序存放三相电流，单位 A。Clarke 把它们写成固定坐标的 α、β 两个分量。`&` 将字段的地址传给函数，函数用 `*alpha`、`*beta` 将结果写到调用者指定的位置。
- Park 用刚算出的 α、β 和 `angle` 旋转坐标轴。这个角是反馈电角，单位 rad。两个结果通过地址写进 `foc->id_A`、`foc->iq_A`，单位仍是 A。
- 两个结果都是有限数才返回 `true`。空指针、非有限输入或输出返回 `false`。应用收到失败后记录数学故障并关桥。 [变换定义](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/control/foc.c#L27) · [应用调用和失败处理](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L489)

**手算输入**。选同一时刻的 `[ia, ib, ic] = [1, −0.5, −0.5] A` 和反馈电角 `θe = π/2 rad`。在模型和 C 代码里各核对一次。

~~~text
iα = (2ia − ib − ic) / 3       = 1 A
iβ = (ib − ic) / √3           = 0 A
Id =  iα cos θe + iβ sin θe    = 0 A
Iq = −iα sin θe + iβ cos θe    = −1 A
~~~

把坐标轴旋转 90° 后，原来指向正 α 轴的电流在新坐标里指向负 q 轴。用同一个角做逆 Park 会得到原来的 `[iα, iβ] = [1, 0] A`。如果模型在这组输入上得出 `Iq = +1 A`，先查它的 q 轴定义、Park 角度方向和 A/B/C 顺序。

### 同一拍有两条角度通路

**执行示意**。上路测量这一拍的电流，下路生成开环电压。

~~~text
实测 ia/ib/ic → Clarke → Park（反馈电角）→ Id/Iq
给定速度 × 周期 → 累加给定电角 → 逆 Park（给定电角）→ SVPWM → duty
~~~

在 `APP_OPEN` 中，`open_angle` 按命令推进。以 10 电 rad/s 和 50 μs 为例，一拍增加约 0.0005 rad。随后 `Foc_VoltageStep(&foc, open_angle, 0, reference.vq_V, bus)` 用给定角调制。仅在离线回放里改反馈角时，Id/Iq 会变化，按原给定重算的 duty 保持原值。实物运行中的传感器健康和故障检查仍然有效。 [开环分支](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L519) · [逆变换和调制](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/control/foc.c#L108)

### 怎样看出开环数据可用于闭环

取开环起步之后的一段稳定记录。将电流、反馈角、给定角的时间轴对齐。跨过 0 的角度先展开，比较方向和斜率。下面是**观察判据**，实际幅值受电机、电压、负载及采样影响。

| 看哪几条波形 | 一段可用于继续验证的记录 | 优先排查 |
|---|---|---|
| 三相电流 `ia/ib/ic` | 电机同步转动且绕组均衡时，三相通常围绕零值，幅值相近、相位依次差约 120 电角度，三相和接近零。开关纹波和失真允许存在 | 静止时已偏移就查零偏。单相幅值特别小或过大就查通道、量程和换算。电流符号全反时，三相和仍可能为零 |
| 给定、反馈电角 | 同步跟随的区间中，展开后斜率和方向接近，允许相差一段角度。停住、持续滑移、换向或失步时，另选稳定区间 | 斜率反号查编码器方向和相序。倍率明显不符查机械角到电角的极对数。给定角动而反馈角基本不动时，先确认电机是否实际转动 |
| 用反馈角算出的 Id/Iq | 同步、稳速且电流基波均衡时，两条曲线常围绕某个值小幅变化。开环可出现非零 Id，Iq 的大小也随负载变化 | 先用记录的同一组三相和反馈角重算。重算吻合但稳态 dq 明显摆动时，回查相序、反馈角方向、角度倍率及取样对齐。固定电零偏移会旋转 dq，可用低压对齐来区分 |

回放时依次比对三相 A、Clarke 的 αβ、Park 的 Id/Iq。先找到第一个对不上的中间量。若模型和固件都给出同样的异常波形，下一步查输入和运行条件。没有板时，可用自己的仿真波形做同样的逐列比较。

## STM32 部署示例

沿用 LAB4（`LAB_TRANSFORMS`）和 3.5 的电零位。示例在三相 ADC 到齐后换算电流，用本拍采样的触发参考时刻计算反馈电角，再调用 `Foc_Measure`。上板前先检查零位有效，按 3.3 完成启停和 UART4 观察的准备。LAB4 的 JustFloat 六通道依次为 `ia, ib, ic, Id, Iq, 反馈电角`。给定角可另看 `MotorApp_Diagnostics.open_angle_rad`。 [电流中断和遥测](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L414) · [3.3 开环部署和 UART4](02-open-loop.md)

先离线确认短时低压开环命令，再按 3.3 的供电、限流、空载和停止条件决定是否实发。下面命令只生成报文和启停计划。

~~~powershell
python tools/can_demo.py open --electrical-speed 10 --vq 0.2 --duration 2
~~~

已有有效电零位时，可以再做一次低压对齐，观察电流极性。转子在正 d 轴对齐电压下稳定后，反馈电角应接近电角零点，Id 应主要为正，Iq 相对小。开环转动时再按上表观察稳态区间。若 0.2 V 未带动转子，记录这一现象并停机检查运行条件。出现异常电流、振动或跟随丢失时，停止输出再排查。配套工程的实板结果待记录。 [验证记录](reference.md#验证记录)
