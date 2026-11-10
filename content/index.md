# 从 Simulink 到 STM32 实机

你已经在 Simulink 里搭过电机控制模型。这里从熟悉的电压矢量和占空比出发，沿实际代码走到定时器、PWM 引脚和第一次电压开环。

## 开始学习

<div class="cards">
<a class="card" href="00-project.html"><span class="eyebrow">00 · 工程入口</span><strong>看懂工程怎样启动</strong><span>从 main() 区分 CubeMX 初始化和手写应用，找到独立的软件操作手册。</span></a>
<a class="card" href="01-pwm-svpwm.html"><span class="eyebrow">01 · PWM 与 SVPWM</span><strong>把模型的占空比交给定时器</strong><span>先配 CubeMX，再看计数、互补输出和死区，最后把 min/max 与扇区法对上。</span></a>
<a class="card" href="02-open-loop.html"><span class="eyebrow">02 · 电压开环</span><strong>沿给定电角完成一次启停</strong><span>追到逆 Park 和功率输出，再按 LAB2 的构建、SWD 下载和 CAN 启停步骤部署。</span></a>
</div>

## 怎么使用这份 Wiki

- **已有仿真模型。** 从 00 章开始，继续用自己的电压、占空比和角度波形对照固件。正文保留可直接读的 C 源码摘录。
- **已另行取得配套工程。** 按工具手册导入并构建。CubeMX 部分用 STM32H743 的工程作示范，换器件时按实际定时器、触发和关断接口配置。
- **准备上板。** 先核对板卡、电机与电源条件，再进入 02 章的操作流程。软件构建及 CAN 离线预览已经核对，实机试转待验证。

[配置与验证参考](reference.md) · [源码定位](source.md) · [后续课程](reference.md#后续课程)
