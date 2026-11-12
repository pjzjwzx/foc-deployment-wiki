# 如何从基础工科到电机控制FOC实操-简易精准的无痛入门教程

从 FOC 的基本概念走到能读懂一套固件、判断运行状态并定位故障。第 3 章以 STM32H743 为例，从工程通路读到板级实现，再进入实机实验。

## 开始学习

| 课程 | 当前安排 |
|---|---|
| [第 1 章　大致了解 FOC 算法](chapter-1.md) | 提纲与指定阅读 |
| [第 2 章　搭建 Simulink FOC 仿真](chapter-2.md) | 预留 |
| [第 3 章　FOC 工程部署](00-project.md) | 11 节正文 |
| [第 4 章　滤波与编码器位置、速度估计](chapter-4.md) | 提纲 |
| [第 5 章　理解离散控制](chapter-5.md) | 提纲 |
| [第 6 章　实践与 Debug 故障记录](chapter-6.md) | 记录模板 |

<div class="cards">
<a class="card" href="00-project.html"><span class="eyebrow">3.1 · 工程启动</span><strong>看懂工程怎样启动</strong><span>从上电走到应用接手，2.2 提供工具安装和首次构建手册。</span></a>
<a class="card" href="01-pwm-svpwm.html"><span class="eyebrow">3.2 · 逆 Park、SVPWM 与 PWM</span><strong>从 dq 电压到三路 PWM</strong><span>用同一组数值追到三相占空比和 TIM1 比较值。</span></a>
<a class="card" href="02-open-loop.html"><span class="eyebrow">3.3 · 电压开环</span><strong>沿给定电角完成一次启停</strong><span>追到逆 Park 和功率输出，再按 LAB2 的构建、SWD 下载和 CAN 启停步骤部署。</span></a>
</div>

### 从反馈到闭环

| 小节 | 源码走读 |
|---|---|
| [3.4 电流与角度传感器](03-sensors.md) | 从 ADC 计数和编码器帧得到安培与机械角。 |
| [3.5 采样、中断与一拍控制](04-sampling-zero.md) | 沿时序读触发、反馈帧、控制计算和 CCR 生效，再查 CubeMX 配置。 |
| [3.6 电零点与开环波形验算](05-transforms.md) | 建立电角基准，用同一组三相电流与反馈电角回放 Clarke、Park。 |
| [3.7 电流闭环与初步调试](06-current-pi.md) | 算一拍 PI，读积分和限幅，用小阶跃看跟踪与饱和。 |
| [3.8 固件分层与移植](06a-software-architecture.md) | 沿命令与过流停机认识各层接口。 |
| [3.9 速度环与多速率调度](07-speed.md) | 从两个角度样本估速度，再由速度 PI 产生 Iq 参考。 |
| [3.10 CAN MIT](08-can-mit.md) | 沿一帧报文追到输出轴力矩、电机电流和反馈。 |
| [3.11 开环 LUT 标定](09-lut.md) | 读懂正反向扫描、角度修正与掉电保存。 |

## 配套固件

本章的源码摘录、配置与构建步骤对应 `chapter-3`。要跟着构建或上板，请下载该版本，解压后按 3.1 导入工程。 [示例工程](https://github.com/pjzjwzx/foc-stm32-example) · [下载 chapter-3 ZIP](https://github.com/pjzjwzx/foc-stm32-example/archive/refs/tags/chapter-3.zip)

## 怎么使用这份 Wiki

- **1.1 与 1.2** 从实际信号通路读到应用、控制和协议代码。
- **2.1** 对照 H743 的配置、生成代码和 BSP，理解同一条通路怎样落到硬件。前三部分都可以直接在网页上读完。
- **2.2** 集中提供工具、构建、接线和各 LAB 的实机验收。软件构建及 CAN 离线预览已经核对，实机试转待验证。

[配置与验证参考](reference.md) · [源码定位](source.md)
