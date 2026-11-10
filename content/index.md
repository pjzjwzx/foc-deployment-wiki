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
| 第 3 章　STM32 部署 | 规划 11 节，已公开 3 节（3.1–3.3） |

<div class="cards">
<a class="card" href="00-project.html"><span class="eyebrow">3.1 · 工程启动</span><strong>看懂工程怎样启动</strong><span>从 main() 区分 CubeMX 初始化和手写应用，找到独立的软件操作手册。</span></a>
<a class="card" href="01-pwm-svpwm.html"><span class="eyebrow">3.2 · PWM 与 SVPWM</span><strong>把模型的占空比交给定时器</strong><span>先配 CubeMX，再看计数、互补输出和死区，最后把 min/max 与扇区法对上。</span></a>
<a class="card" href="02-open-loop.html"><span class="eyebrow">3.3 · 电压开环</span><strong>沿给定电角完成一次启停</strong><span>追到逆 Park 和功率输出，再按 LAB2 的构建、SWD 下载和 CAN 启停步骤部署。</span></a>
</div>

## 配套固件

本章的源码摘录、配置与构建步骤对应 `chapter-3`。要跟着构建或上板，请下载该版本，解压后按 3.1 导入工程。 [示例工程](https://github.com/pjzjwzx/foc-stm32-example) · [下载 chapter-3 ZIP](https://github.com/pjzjwzx/foc-stm32-example/archive/refs/tags/chapter-3.zip)

## 怎么使用这份 Wiki

- **对照仿真。** 从 3.1 开始，继续用自己的电压、占空比和角度波形对照固件。
- **构建固件。** 按 3.1 的工具手册导入并构建。CubeMX 部分用 STM32H743 作示范，换器件时按实际定时器、触发和关断接口配置。
- **准备上板。** 先核对板卡、电机与电源条件，再进入 3.3 的操作流程。软件构建及 CAN 离线预览已经核对，实机试转待验证。

[配置与验证参考](reference.md) · [源码定位](source.md) · [后续课程](reference.md#后续课程)
