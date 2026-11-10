# 源码定位

三章正文提供本课用到的真实源码摘录。下面按文件汇总其余入口与行号，便于拿到配套工程后定位。行号对应编写本版教程时的工程，换版本可以优先搜索函数名或字段名。

## app/motor_app.c

| 位置 | 去看什么 | 出现在哪一章 |
|---|---|---|
| <span id="app-motor-app-c-l72">第 72 行</span> | 应用初始化 | [00 工程启动](00-project.md) |
| <span id="app-motor-app-c-l116">第 116 行</span> | 状态入口 | [02 电压开环部署](02-open-loop.md) |
| <span id="app-motor-app-c-l150">第 150 行</span> | 状态发布 | [02 电压开环部署](02-open-loop.md) |
| <span id="app-motor-app-c-l306">第 306 行</span> | 目标接收与限制 | [02 电压开环部署](02-open-loop.md) |
| <span id="app-motor-app-c-l414">第 414 行</span> | 发布位置 | [02 电压开环部署](02-open-loop.md) |
| <span id="app-motor-app-c-l467">第 467 行</span> | 零偏完成时进入 Disabled | [02 电压开环部署](02-open-loop.md) |
| <span id="app-motor-app-c-l479">第 479 行</span> | 本拍的保护判断 | [02 电压开环部署](02-open-loop.md) |
| <span id="app-motor-app-c-l498">第 498 行</span> | LAB1 与 Open 分支、LAB1 分支 | [01 PWM 与 SVPWM](01-pwm-svpwm.md) · [02 电压开环部署](02-open-loop.md) |
| <span id="app-motor-app-c-l519">第 519 行</span> | 开环分支 | [02 电压开环部署](02-open-loop.md) |
| <span id="app-motor-app-c-l548">第 548 行</span> | 返回后的提交 | [02 电压开环部署](02-open-loop.md) |
| <span id="app-motor-app-c-l575">第 575 行</span> | 后台函数 | [00 工程启动](00-project.md) |

## bsp/board.c

| 位置 | 去看什么 | 出现在哪一章 |
|---|---|---|
| <span id="bsp-board-c-l69">第 69 行</span> | 关断入口 | [02 电压开环部署](02-open-loop.md) |
| <span id="bsp-board-c-l104">第 104 行</span> | 比较值写入 | [01 PWM 与 SVPWM](01-pwm-svpwm.md) |
| <span id="bsp-board-c-l118">第 118 行</span> | 提交检查、观察与运行提交 | [02 电压开环部署](02-open-loop.md) |
| <span id="bsp-board-c-l137">第 137 行</span> | 引脚输出接口 | [01 PWM 与 SVPWM](01-pwm-svpwm.md) |
| <span id="bsp-board-c-l144">第 144 行</span> | 板级准备、板级启动 | [00 工程启动](00-project.md) · [02 电压开环部署](02-open-loop.md) |
| <span id="bsp-board-c-l175">第 175 行</span> | 启动采样与通道、板级启动 | [00 工程启动](00-project.md) · [01 PWM 与 SVPWM](01-pwm-svpwm.md) |

## bsp/board.h

| 位置 | 去看什么 | 出现在哪一章 |
|---|---|---|
| <span id="bsp-board-h">文件</span> | 板级接口 | [02 电压开环部署](02-open-loop.md) |

## config/lab_config.h

| 位置 | 去看什么 | 出现在哪一章 |
|---|---|---|
| <span id="config-lab-config-h">文件</span> | 对应配置文件、控制周期定义 | [00 工程启动](00-project.md) · [02 电压开环部署](02-open-loop.md) |

## config/motor_config.h

| 位置 | 去看什么 | 出现在哪一章 |
|---|---|---|
| <span id="config-motor-config-h">文件</span> | 电机参数 | [02 电压开环部署](02-open-loop.md) |

## control/foc.c

| 位置 | 去看什么 | 出现在哪一章 |
|---|---|---|
| <span id="control-foc-c-l20">第 20 行</span> | 折回函数 | [02 电压开环部署](02-open-loop.md) |
| <span id="control-foc-c-l41">第 41 行</span> | 逆 Park 定义 | [01 PWM 与 SVPWM](01-pwm-svpwm.md) |
| <span id="control-foc-c-l90">第 90 行</span> | 电压检查与限幅 | [01 PWM 与 SVPWM](01-pwm-svpwm.md) |
| <span id="control-foc-c-l108">第 108 行</span> | 电压到 duty | [02 电压开环部署](02-open-loop.md) |
| <span id="control-foc-c-l119">第 119 行</span> | 调用与三相投影 | [01 PWM 与 SVPWM](01-pwm-svpwm.md) |
| <span id="control-foc-c-l125">第 125 行</span> | 调制源码 | [01 PWM 与 SVPWM](01-pwm-svpwm.md) |

## Core/Src/adc.c

| 位置 | 去看什么 | 出现在哪一章 |
|---|---|---|
| <span id="core-src-adc-c">文件</span> | ADC 触发配置 | [01 PWM 与 SVPWM](01-pwm-svpwm.md) |

## Core/Src/main.c

| 位置 | 去看什么 | 出现在哪一章 |
|---|---|---|
| <span id="core-src-main-c-l73">第 73 行</span> | 启动入口 | [00 工程启动](00-project.md) |
| <span id="core-src-main-c-l118">第 118 行</span> | 启动保留区域 | [00 工程启动](00-project.md) |
| <span id="core-src-main-c-l123">第 123 行</span> | 回看调用处 | [00 工程启动](00-project.md) |
| <span id="core-src-main-c-l133">第 133 行</span> | 后台调用处 | [00 工程启动](00-project.md) |
| <span id="core-src-main-c-l148">第 148 行</span> | 时钟初始化 | [01 PWM 与 SVPWM](01-pwm-svpwm.md) |
| <span id="core-src-main-c-l265">第 265 行</span> | 错误处理 | [00 工程启动](00-project.md) |

## Core/Src/tim.c

| 位置 | 去看什么 | 出现在哪一章 |
|---|---|---|
| <span id="core-src-tim-c">文件</span> | TIM1 初始化代码 | [00 工程启动](00-project.md) |
| <span id="core-src-tim-c-l30">第 30 行</span> | TIM1 初始化 | [00 工程启动](00-project.md) |
| <span id="core-src-tim-c-l44">第 44 行</span> | 定时器初始化 | [01 PWM 与 SVPWM](01-pwm-svpwm.md) |
| <span id="core-src-tim-c-l135">第 135 行</span> | PWM 引脚初始化 | [01 PWM 与 SVPWM](01-pwm-svpwm.md) |

## tools/can_demo.py

| 位置 | 去看什么 | 出现在哪一章 |
|---|---|---|
| <span id="tools-can-demo-py-l179">第 179 行</span> | 工具启停顺序 | [02 电压开环部署](02-open-loop.md) |
| <span id="tools-can-demo-py-l322">第 322 行</span> | 状态回读实现 | [02 电压开环部署](02-open-loop.md) |
