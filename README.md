# FOC 部署 Wiki

面向理解 FOC 基础、第一次读电机控制固件的读者。第 3 章用 STM32H743 示范工程部署。每节从工程通路走到高层代码、BSP 与 CubeMX，最后给出 LAB 实机步骤与验收条件。

| 章节 | 当前内容 |
|---|---|
| [第 1 章　大致了解 FOC 算法](content/chapter-1.md) | 提纲与指定阅读 |
| [第 2 章　搭建 Simulink FOC 仿真](content/chapter-2.md) | 预留 |
| [第 3 章　FOC 工程部署](content/00-project.md) | 11 节正文 |
| [第 4 章　滤波与编码器位置、速度估计](content/chapter-4.md) | 提纲 |
| [第 5 章　理解离散控制](content/chapter-5.md) | 提纲 |
| [第 6 章　实践与 Debug 故障记录](content/chapter-6.md) | 记录模板 |

- [在线 Wiki](https://pjzjwzx.github.io/foc-deployment-wiki/)
- [3.1 工程启动](content/00-project.md)
- [3.2 逆 Park、SVPWM 与 PWM 输出](content/01-pwm-svpwm.md)
- [3.3 第一次电压开环](content/02-open-loop.md)
- [3.4 电流与角度传感器](content/03-sensors.md)
- [3.5 采样、中断与一拍控制](content/04-sampling-zero.md)
- [3.6 电零点与开环波形验算](content/05-transforms.md)
- [3.7 电流闭环与初步调试](content/06-current-pi.md)
- [3.8 固件分层与移植](content/06a-software-architecture.md)
- [3.9 速度环与多速率调度](content/07-speed.md)
- [3.10 CAN MIT](content/08-can-mit.md)
- [3.11 开环 LUT 标定](content/09-lut.md)
- [配置与验证参考](content/reference.md) · [源码定位](content/source.md)

配套工程使用固定标签 `chapter-3`。构建、下载和上板时，请使用与本章源码摘录对应的版本。 [示例工程](https://github.com/pjzjwzx/foc-stm32-example) · [下载 chapter-3 ZIP](https://github.com/pjzjwzx/foc-stm32-example/archive/refs/tags/chapter-3.zip)

## 维护

逐节修订时可以查阅[六章教学大纲](content/teaching-plan-revised.md)。

修改 `content/` 下的 Markdown，再重新生成 `docs/`。GitHub Pages 从 `main` 的 `/docs` 发布。

    python -m pip install -r requirements.txt
    python build.py
