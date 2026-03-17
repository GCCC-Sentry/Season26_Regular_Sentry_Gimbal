# 此为齐鲁工业大学Adam战队26赛季电控代码统一管理仓库
>   这是齐鲁工业大学Adam战队26赛季全兵种统一管理的代码仓库，采用达妙开发板与gcc开发环境编译，各兵种独立代码进行条件编译，如果你想了解代码目录与其它说明，请继续阅读此readme，如果你想了解各兵种具体框架结构，请前往以下readme文件：
>>全向步兵：[readme_omni](User\infantry_omni\readme_omni.md)
>>舵轮步兵：[readme_helm](User\infantry_helm\readme_helm.md)
>>轮腿步兵：[readme_leg](User\infantry_leg\readme_leg.md)
>>英雄：[readme_hero](User\hero\readme_hero.md)
>>哨兵：[readme_sentry](User\sentry\readme_sentry.md)

---

## 软件环境

- Toolchain:arm-none-eabi
- package version:STM32Cube STM32H7 Series 1.12.1
- STM32CubeMX version: 6.13.0

> 没用过gcc?请看这篇文章[VSCode+Ozone使用方法](.doc/ch/VSCode+Ozone使用方法.md)

---
## 仓库结构

### 目录说明

SEASON—26—CODE/
├── build/ # 编译输出目录（包含.o/.d文件及最终固件，已忽略）
├── Core/ # CubeMX生成的共用核心系统文件（启动文件、链接脚本等）
├── debug/ # 调试相关配置
├── Drivers/ # 硬件驱动层（HAL库、外设驱动）
├── Middlewares/ # 中间件（FreeRTOS、USB协议栈等）
├── USB_DEVICE/ # USB设备配置
├── User/ # 用户代码
│  &nbsp;&nbsp;&nbsp;&nbsp; ├── common/ # 通用模块(Algorithm，BSP，Hardware)
│  &nbsp;&nbsp;&nbsp;&nbsp; ├── infantry/ # 步兵专属软件层
│  &nbsp;&nbsp;&nbsp;&nbsp; ├── hero/ # 英雄专属软件层
│  &nbsp;&nbsp;&nbsp;&nbsp; └── sentry/ # 哨兵专属软件层
│
└── Makefile            # 条件编译控制核心

### 代码结构介绍

![image](.doc\.assets\code.png)
整体代码采用分层设计的原则，每层内部模块化涉及，秉承高内聚低耦合的原则。
HAL：ST官方提供的库
Algorithm:算法模块（包含AHRS、滤波、PID等核心算法）
BSP：对常用外设，如串口，can等
Hardware：硬件设备驱动，主要存放非官方自己编写的硬件驱动，或对于官方驱动的进一步封装
Software：软件层，机器人各个组件及其功能
>由于采用多兵种代码统一管理框架，代码中Algorithm，BSP，Hardware三层高度吻合，将其放在公用目录中，实现模块复用，大大提高开发效率；而软件层放在各兵种独立代码仓库中，采用条件编译。

### 代码规范简述

如上面所言，我们的代码使用了分层设计，原则上，每层只能沿图中箭头调用下层代码，不能跨层调用，也不能调用上层代码
但是在实际使用中，在特定场景会打破这一规则，如外设回调函数。
也可以使用函数回调等方式去遵守严格的调用规则，但本工程代码量较小，不严格遵守分层思想不会造成大问题。

### 分支说明
- main分支：存放当前阶段各兵种最稳定的代码
- dev分支：如infantry_omni_dev，兵种专属分支，用于测试兵种独立代码部分
- test分支：如IMU_test，通用模块分支，用于测试通用代码部分
  
---

## 条件编译说明

### Makefile文件说明
- 兵种类型选择设置：可直接手动修改，也可以通过通过命令行覆盖 
- 兵种专属源文件和公共源文件：User文件中.c文件不必手动添加，其他.c文件请手动添加
- 合并所有源文件
![image](.doc\.assets\type.png)

---

## 部分可能遇到的问题以及解决方案

### CubeMX自动生成.ld文件Bug

    实践发现，从某个版本开始CubeMx自动生成的.ld文件无法使用，至笔者目前使用的版本（6.14.0）仍存在这个问题。
    我们准备了一个可以使用的.ld文件：[stm32h723vgtx_flash.ld](./stm32h723vgtx_flash.ld)
    最直接的解决方案便是每次使用CubeMx生成代码之后，在Makefile中将使用的.ld文件改为这个。

### printf无法打印浮点数解决方案

    嵌入式开发的调试过程中，我们经常使用printf打印一些数据
    由于GCC为了缩减编译后的代码尺寸，未使用可以打印浮点数的printf，我们使用%f打印float数据的时候，会发现无法打印
    解决方案，在Makefile对应位置添加如下编译选项

```
 # printf float
    LDFLAGS += -lc -lrdimon -u _printf_float
```

### 串口DMA收发缓冲区域问题

使用DMA进行串口数据收发的过程中，全局变量会默认被放置在TCM RAM区域，此区域与DMA外设无总线相连，外设通过DMA向内存搬运的数据无法送达，需要通过更改全局变量位置至其他区域解决，前文提供的.ld文件已经更改。

欲详细了解这一块的只是，可以看这篇文章[STM32H7系列教程3 SRAM区详解和DMA1、2无法访问TCM的解决方法](https://zhuanlan.zhihu.com/p/4218673539)
