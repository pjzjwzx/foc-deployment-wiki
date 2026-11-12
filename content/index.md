# 如何从基础工科到电机控制FOC实操-简易精准的无痛入门教程

“无痛”指少迷茫，少走弯路，少自己摸索该怎么学、怎么做。这个过程会带来discourage和迷茫。

这份教程主要做统合、并补齐现有开源教材没有覆盖的点，所以部分模块会直接推荐已有的很好的教材/教程

这份教材不能够完全cover工业化的、完备的工业电机程序，但是能快速从0入门

在使用这份教程时，要善用ai来解决一些让人烦躁的小问题


## 开始学习

| 课程 | 当前安排 |
|---|---|
| 第 1 章 | 理解FOC的几个部分 |
| 第 2 章 | 在Simulink中搭建一套简易的FOC |
| 第 3 章　FOC 实际部署 | 11 节（3.1–3.11） |

<div class="cards">
<a class="card" href="00-project.html"><span class="eyebrow">3.1 · 工程启动</span><strong>看懂工程怎样启动</strong><span>从 main() 区分 CubeMX 初始化和手写应用，找到独立的软件操作手册。</span></a>
<a class="card" href="01-pwm-svpwm.html"><span class="eyebrow">3.2 · PWM 与 SVPWM</span><strong>把模型的占空比交给定时器</strong><span>先把 min/max 与扇区法对上，再看 CubeMX、互补输出和死区。</span></a>
<a class="card" href="02-open-loop.html"><span class="eyebrow">3.3 · 电压开环</span><strong>沿给定电角完成一次启停</strong><span>追到逆 Park 和功率输出，再按 LAB2 的构建、SWD 下载和 CAN 启停步骤部署。</span></a>
</div>

### 从反馈到闭环

| 小节 | 源码走读 |
|---|---|
| [3.4 电流与角度传感器](03-sensors.md) | 从 ADC 计数和编码器帧得到安培与机械角。 |
| [3.5 采样、中断与电零点](04-sampling-zero.md) | 从离散步长追到触发、控制计算和 CCR 生效，再看电角基准。 |
| [3.6 坐标变换与开环验算](05-transforms.md) | 保持电压开环，沿 Clarke、Park 核对波形。 |
| [3.7 PI 电流闭环](06-current-pi.md) | 跟一拍离散 PI，从电流误差走到限幅与占空比。 |
| [3.8 从一拍控制理解软件分层](06a-software-architecture.md) | 沿命令与过流停机认识各层接口。 |
| [3.9 速度估计与速度闭环](07-speed.md) | 从两个角度样本估速度，再由速度 PI 产生 Iq 参考。 |
| [3.10 CAN MIT](08-can-mit.md) | 沿一帧报文追到输出轴力矩、电机电流和反馈。 |
| [3.11 开环 LUT 标定](09-lut.md) | 读懂正反向扫描、角度修正与掉电保存。 |

## 配套固件

本章的源码摘录、配置与构建步骤对应 `chapter-3`。要跟着构建或上板，请下载该版本，解压后按 3.1 导入工程。 [示例工程](https://github.com/pjzjwzx/foc-stm32-example) · [下载 chapter-3 ZIP](https://github.com/pjzjwzx/foc-stm32-example/archive/refs/tags/chapter-3.zip)

## 怎么使用这份 Wiki

- **读通用 FOC 部署。** 从 3.1 开始，用自己的 Simulink 信号对照 C 变量、控制周期和运行条件。
- **看 STM32 部署示例。** 想在板上复现时，按 3.1 的工具手册导入并构建。CubeMX 部分用 STM32H743 作示范，换器件时按实际定时器、触发和关断接口配置。
- **准备上板。** 先核对板卡、电机与电源条件，再进入 3.3 的操作流程。软件构建及 CAN 离线预览已经核对，实机试转待验证。

[配置与验证参考](reference.md) · [源码定位](source.md)
