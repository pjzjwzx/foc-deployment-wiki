# 源码定位

第 3 章的源码链接对应配套固件的固定标签 `chapter-3`。换版本时可以优先搜索函数名或字段名。 [示例工程](https://github.com/pjzjwzx/foc-stm32-example) · [下载 chapter-3 ZIP](https://github.com/pjzjwzx/foc-stm32-example/archive/refs/tags/chapter-3.zip)

## 按信号查找源码

| 从哪条信号开始 | 源码落点 | 对应小节 |
|---|---|---|
| 三相原始计数、编码器读数 | [采样帧入口](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L462) · [编码器取样](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/bsp/encoder.c#L383) | [3.4 电流与角度传感器](03-sensors.md) |
| ADC 转换完成、控制计算与 PWM 写入 | [ADC 中断](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/Core/Src/stm32h7xx_it.c#L261) · [三相数据汇合](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/bsp/board.c#L298) · [比较值提交](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/bsp/board.c#L118) | [3.5 采样、中断与一拍控制](04-sampling-zero.md) |
| 电零点、反馈电角、相电流与 Id/Iq | [对齐与取样](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/calibration.c#L47) · [电角计算](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/control/rotor.c#L71) · [测量与变换](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/control/foc.c#L78) | [3.6 电零点与开环波形验算](05-transforms.md) |
| 电流目标、PI 积分与三相 duty | [电流 PI](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/control/foc.c#L134) · [比较值提交](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/bsp/board.c#L118) | [3.7 电流闭环与初步调试](06-current-pi.md) |
| 电流命令与故障 | [应用接收命令](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L306) · [故障处理](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L53) | [3.8 软件分层](06a-software-architecture.md) |
| 机械角、速度与 Iq 参考 | [速度估计](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/control/rotor.c#L32) · [外环调度](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L527) | [3.9 速度环与多速率调度](07-speed.md) |
| MIT 命令、输出轴力矩与反馈 | [报文解码](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/communication/protocol.c#L29) · [MIT 计算](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/control/foc.c#L185) | [3.10 CAN MIT](08-can-mit.md) |
| 原始角、零位与修正表 | [扫描](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/calibration.c#L125) · [插值](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/control/rotor.c#L19) · [Flash 保存](https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/bsp/flash_store.c#L11) | [3.11 开环 LUT 标定](09-lut.md) |

## app/motor_app.c

| 位置 | 去看什么 | 对应小节 |
|---|---|---|
| <a id="app-motor-app-c-l72" href="https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L72">第 72 行</a> | 应用初始化 | [3.1 工程启动](00-project.md) |
| <a id="app-motor-app-c-l116" href="https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L116">第 116 行</a> | 状态入口 | [3.3 电压开环部署](02-open-loop.md) |
| <a id="app-motor-app-c-l150" href="https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L155">第 155 行</a> | 状态发布 | [3.3 电压开环部署](02-open-loop.md) |
| <a id="app-motor-app-c-l306" href="https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L306">第 306 行</a> | 目标接收与限制 | [3.3 电压开环部署](02-open-loop.md) |
| <a id="app-motor-app-c-l414" href="https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L414">第 414 行</a> | 发布位置 | [3.3 电压开环部署](02-open-loop.md) |
| <a id="app-motor-app-c-l467" href="https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L467">第 467 行</a> | 零偏完成时进入 Disabled | [3.3 电压开环部署](02-open-loop.md) |
| <a id="app-motor-app-c-l479" href="https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L479">第 479 行</a> | 本拍的保护判断 | [3.3 电压开环部署](02-open-loop.md) |
| <a id="app-motor-app-c-l498" href="https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L498">第 498 行</a> | LAB1 与 Open 分支、LAB1 分支 | [3.2 PWM 与 SVPWM](01-pwm-svpwm.md) · [3.3 电压开环部署](02-open-loop.md) |
| <a id="app-motor-app-c-l519" href="https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L519">第 519 行</a> | 开环分支 | [3.3 电压开环部署](02-open-loop.md) |
| <a id="app-motor-app-c-l548" href="https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L548">第 548 行</a> | 返回后的提交 | [3.3 电压开环部署](02-open-loop.md) |
| <a id="app-motor-app-c-l575" href="https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L575">第 575 行</a> | 后台函数 | [3.1 工程启动](00-project.md) |

## bsp/board.c

| 位置 | 去看什么 | 对应小节 |
|---|---|---|
| <a id="bsp-board-c-l69" href="https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/bsp/board.c#L69">第 69 行</a> | 关断入口 | [3.3 电压开环部署](02-open-loop.md) |
| <a id="bsp-board-c-l104" href="https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/bsp/board.c#L104">第 104 行</a> | 比较值写入 | [3.2 PWM 与 SVPWM](01-pwm-svpwm.md) |
| <a id="bsp-board-c-l118" href="https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/bsp/board.c#L118">第 118 行</a> | 提交检查、观察与运行提交 | [3.3 电压开环部署](02-open-loop.md) |
| <a id="bsp-board-c-l137" href="https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/bsp/board.c#L137">第 137 行</a> | 引脚输出接口 | [3.2 PWM 与 SVPWM](01-pwm-svpwm.md) |
| <a id="bsp-board-c-l144" href="https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/bsp/board.c#L144">第 144 行</a> | 板级准备、板级启动 | [3.1 工程启动](00-project.md) · [3.3 电压开环部署](02-open-loop.md) |
| <a id="bsp-board-c-l175" href="https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/bsp/board.c#L175">第 175 行</a> | 启动采样与通道、板级启动 | [3.1 工程启动](00-project.md) · [3.2 PWM 与 SVPWM](01-pwm-svpwm.md) |

## bsp/board.h

| 位置 | 去看什么 | 对应小节 |
|---|---|---|
| <a id="bsp-board-h" href="https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/bsp/board.h">文件</a> | 板级接口 | [3.3 电压开环部署](02-open-loop.md) |

## config/lab_config.h

| 位置 | 去看什么 | 对应小节 |
|---|---|---|
| <a id="config-lab-config-h" href="https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/config/lab_config.h">文件</a> | 对应配置文件、控制周期定义 | [3.1 工程启动](00-project.md) · [3.3 电压开环部署](02-open-loop.md) |

## config/motor_config.h

| 位置 | 去看什么 | 对应小节 |
|---|---|---|
| <a id="config-motor-config-h" href="https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/config/motor_config.h">文件</a> | 电机参数 | [3.3 电压开环部署](02-open-loop.md) |

## control/foc.c

| 位置 | 去看什么 | 对应小节 |
|---|---|---|
| <a id="control-foc-c-l20" href="https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/control/foc.c#L20">第 20 行</a> | 折回函数 | [3.3 电压开环部署](02-open-loop.md) |
| <a id="control-foc-c-l41" href="https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/control/foc.c#L41">第 41 行</a> | 逆 Park 定义 | [3.2 PWM 与 SVPWM](01-pwm-svpwm.md) |
| <a id="control-foc-c-l90" href="https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/control/foc.c#L90">第 90 行</a> | 电压检查与限幅 | [3.2 PWM 与 SVPWM](01-pwm-svpwm.md) |
| <a id="control-foc-c-l108" href="https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/control/foc.c#L108">第 108 行</a> | 电压到 duty | [3.3 电压开环部署](02-open-loop.md) |
| <a id="control-foc-c-l119" href="https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/control/foc.c#L119">第 119 行</a> | 调用与三相投影 | [3.2 PWM 与 SVPWM](01-pwm-svpwm.md) |
| <a id="control-foc-c-l125" href="https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/control/foc.c#L125">第 125 行</a> | 调制源码 | [3.2 PWM 与 SVPWM](01-pwm-svpwm.md) |

## Core/Src/adc.c

| 位置 | 去看什么 | 对应小节 |
|---|---|---|
| <a id="core-src-adc-c" href="https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/Core/Src/adc.c">文件</a> | ADC 触发配置 | [3.2 PWM 与 SVPWM](01-pwm-svpwm.md) |

## Core/Src/main.c

| 位置 | 去看什么 | 对应小节 |
|---|---|---|
| <a id="core-src-main-c-l73" href="https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/Core/Src/main.c#L73">第 73 行</a> | 启动入口 | [3.1 工程启动](00-project.md) |
| <a id="core-src-main-c-l118" href="https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/Core/Src/main.c#L118">第 118 行</a> | 启动保留区域 | [3.1 工程启动](00-project.md) |
| <a id="core-src-main-c-l123" href="https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/Core/Src/main.c#L123">第 123 行</a> | 回看调用处 | [3.1 工程启动](00-project.md) |
| <a id="core-src-main-c-l133" href="https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/Core/Src/main.c#L133">第 133 行</a> | 后台调用处 | [3.1 工程启动](00-project.md) |
| <a id="core-src-main-c-l148" href="https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/Core/Src/main.c#L148">第 148 行</a> | 时钟初始化 | [3.2 PWM 与 SVPWM](01-pwm-svpwm.md) |
| <a id="core-src-main-c-l265" href="https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/Core/Src/main.c#L265">第 265 行</a> | 错误处理 | [3.1 工程启动](00-project.md) |

## Core/Src/tim.c

| 位置 | 去看什么 | 对应小节 |
|---|---|---|
| <a id="core-src-tim-c" href="https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/Core/Src/tim.c">文件</a> | TIM1 初始化代码 | [3.1 工程启动](00-project.md) |
| <a id="core-src-tim-c-l30" href="https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/Core/Src/tim.c#L30">第 30 行</a> | TIM1 初始化 | [3.1 工程启动](00-project.md) |
| <a id="core-src-tim-c-l44" href="https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/Core/Src/tim.c#L44">第 44 行</a> | 定时器初始化 | [3.2 PWM 与 SVPWM](01-pwm-svpwm.md) |
| <a id="core-src-tim-c-l135" href="https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/Core/Src/tim.c#L135">第 135 行</a> | PWM 引脚初始化 | [3.2 PWM 与 SVPWM](01-pwm-svpwm.md) |

## tools/can_demo.py

| 位置 | 去看什么 | 对应小节 |
|---|---|---|
| <a id="tools-can-demo-py-l179" href="https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/tools/can_demo.py#L179">第 179 行</a> | 工具启停顺序 | [3.3 电压开环部署](02-open-loop.md) |
| <a id="tools-can-demo-py-l322" href="https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/tools/can_demo.py#L322">第 322 行</a> | 状态回读实现 | [3.3 电压开环部署](02-open-loop.md) |
