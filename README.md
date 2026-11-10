# FOC 部署 Wiki

面向已搭建 Simulink 电机控制模型、第一次将 FOC 部署到 STM32 的读者。前三章依次讲工程启动、PWM 与 min/max SVPWM、第一次电压开环部署。

- [在线 Wiki](https://pjzjwzx.github.io/foc-deployment-wiki/)
- [00 看懂工程怎样启动](content/00-project.md)
- [01 从 Simulink 占空比到 STM32 PWM](content/01-pwm-svpwm.md)
- [02 第一次实际部署电压开环](content/02-open-loop.md)
- [配置与验证参考](content/reference.md)

教程正文含必要的源码摘录。构建、下载和上板时需另行取得配套工程。软件验证与实物验证状态见配置附录。

## 维护

修改 content/ 下的 Markdown，再重新生成 docs/。GitHub Pages 从 main 的 /docs 发布。

    python -m pip install -r requirements.txt
    python build.py
