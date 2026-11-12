# FOC 部署 Wiki

面向已搭建 Simulink 电机控制模型、第一次将 FOC 部署到 MCU 的读者。STM32H743 用于演示工具和外设配置。本仓库包含第 3 章的 11 个小节。每节先讲通用 FOC 部署，再用 STM32 工程走读实现。

- [在线 Wiki](https://pjzjwzx.github.io/foc-deployment-wiki/)
- [3.1 看懂工程怎样启动](content/00-project.md)
- [3.2 从 Simulink 占空比到 STM32 PWM](content/01-pwm-svpwm.md)
- [3.3 第一次实际部署电压开环](content/02-open-loop.md)
- [3.4 电流与角度传感器](content/03-sensors.md)
- [3.5 采样、中断与一拍控制](content/04-sampling-zero.md)
- [3.6 电零点、坐标变换与开环波形验算](content/05-transforms.md)
- [3.7 离散 PI 电流闭环与初步调试](content/06-current-pi.md)
- [3.8 从一拍控制理解软件分层](content/06a-software-architecture.md)
- [3.9 速度估计与速度闭环](content/07-speed.md)
- [3.10 CAN MIT](content/08-can-mit.md)
- [3.11 开环 LUT 标定](content/09-lut.md)
- [配置与验证参考](content/reference.md) · [源码定位](content/source.md)

配套工程使用固定标签 `chapter-3`。构建、下载和上板时，请使用与本章源码摘录对应的版本。 [示例工程](https://github.com/pjzjwzx/foc-stm32-example) · [下载 chapter-3 ZIP](https://github.com/pjzjwzx/foc-stm32-example/archive/refs/tags/chapter-3.zip)

## 维护

逐节修订时可以查阅[第 3 章教学大纲](content/teaching-plan-revised.md)。

修改 `content/` 下的 Markdown，再重新生成 `docs/`。GitHub Pages 从 `main` 的 `/docs` 发布。

    python -m pip install -r requirements.txt
    python build.py
