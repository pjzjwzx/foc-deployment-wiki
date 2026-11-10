# FOC 部署 Wiki

面向已搭建 Simulink 电机控制模型、第一次将 FOC 部署到 STM32 的读者。当前公开第 3 章的三个小节，第 1、2 章预留。

- [在线 Wiki](https://pjzjwzx.github.io/foc-deployment-wiki/)
- [3.1 看懂工程怎样启动](content/00-project.md)
- [3.2 从 Simulink 占空比到 STM32 PWM](content/01-pwm-svpwm.md)
- [3.3 第一次实际部署电压开环](content/02-open-loop.md)
- [配置与验证参考](content/reference.md) · [源码定位](content/source.md)

配套工程使用固定标签 `chapter-3`。构建、下载和上板时，请使用与本章源码摘录对应的版本。 [示例工程](https://github.com/pjzjwzx/foc-stm32-example) · [下载 chapter-3 ZIP](https://github.com/pjzjwzx/foc-stm32-example/archive/refs/tags/chapter-3.zip)

## 维护

修改 `content/` 下的 Markdown，再重新生成 `docs/`。GitHub Pages 从 `main` 的 `/docs` 发布。

    python -m pip install -r requirements.txt
    python build.py
