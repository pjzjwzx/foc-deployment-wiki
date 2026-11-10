# 源码定位

第 3 章的 3.1–3.3 提供正文用到的源码摘录。下表按文件汇总入口与行号，对应配套固件的固定标签 `chapter-3`。换版本时可以优先搜索函数名或字段名。 [示例工程](https://github.com/pjzjwzx/foc-stm32-example) · [下载 chapter-3 ZIP](https://github.com/pjzjwzx/foc-stm32-example/archive/refs/tags/chapter-3.zip)

## app/motor_app.c

| 位置 | 去看什么 | 对应小节 |
|---|---|---|
| <a id="app-motor-app-c-l72" href="https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L72">第 72 行</a> | 应用初始化 | [3.1 工程启动](00-project.md) |
| <a id="app-motor-app-c-l116" href="https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L116">第 116 行</a> | 状态入口 | [3.3 电压开环部署](02-open-loop.md) |
| <a id="app-motor-app-c-l150" href="https://github.com/pjzjwzx/foc-stm32-example/blob/chapter-3/app/motor_app.c#L150">第 150 行</a> | 状态发布 | [3.3 电压开环部署](02-open-loop.md) |
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
