# 闭环步进电机驱动器 (Closed-Loop Stepper Motor Driver)

基于 **STM32F103CBT6** 的闭环步进电机驱动固件。通过磁编码器实时检测转子角度，采用 FOC（磁场定向控制）思想驱动两相步进电机，使步进电机具备类似伺服电机的闭环位置 / 速度 / 力矩控制能力，并支持力矩前馈的轨迹跟踪。

> 本固件可用于机械臂关节、云台、精密进给等需要步进电机闭环伺服的场合。

---

## ✨ 主要功能

- **磁编码器闭环**：使用 MT6816 14-bit 磁编码器读取转子绝对角度，消除步进电机失步问题。
- **FOC 矢量控制**：通过查表正弦换相（`sin_map.h`）和 DCE 控制器（kp/kv/ki/kd）实现平滑、低噪声的闭环控制。
- **多种控制模式**（见 `Ctrl/Motor/motor.h`）：
  - `MODE_COMMAND_CURRENT` —— 力矩 / 电流模式
  - `MODE_COMMAND_VELOCITY` —— 速度模式
  - `MODE_COMMAND_POSITION` —— 位置模式（带速度 / 加速度规划）
  - `MODE_COMMAND_Trajectory` / `MODE_COMMAND_POSITION_TRAJECTORY` —— 轨迹跟踪模式（位置 + 速度 + 加速度前馈）
  - `MODE_PWM_*` —— 对应的 PWM 直接控制模式
- **轨迹跟踪**：支持上位机通过 UART 实时下发 (位置, 速度, 加速度) 序列，进行带前馈的轨迹跟踪（`AddTrajectorySetPoint`）。
- **编码器自动校准**：上电时长按双键（或发送指令）触发，自动标定编码器零位与电角度对齐（`EncoderCalibrator`）。
- **运动规划**：内置速度 / 位置梯形规划器，支持限速、限加速度（`Ctrl/Motor/motion_planner`）。
- **参数掉电保存**：使用模拟 EEPROM（Flash 模拟）保存 CAN 节点 ID、电流 / 速度限制、PID 参数、零位等配置（`Port/Platform/Memory`）。
- **双通信接口**：
  - **UART**：ASCII 指令交互 + 实时数据回传（DMA 发送状态帧）。
  - **CAN**：二进制指令接口，适合多关节总线组网（支持拨码开关设定节点 ID）。
- **保护与状态指示**：芯片温度监测（ADC 内部温度传感器）、堵转保护、状态 LED、两路按键交互。

---

## 🔌 硬件平台

| 部件 | 型号 | 说明 |
|------|------|------|
| 主控 MCU | **STM32F103CBT6** | Cortex-M3 @ 72 MHz, 128 KB Flash |
| 电机驱动 | **TB67H450** ×2 | 两相 H 桥驱动，每相一片，可调电流 |
| 磁编码器 | **MT6816** | 14-bit 绝对式磁编码器，SPI 接口 |
| 通信 | UART / CAN | CAN 用于多节点总线组网 |
| 电机 | 两相步进电机 | 例如 NEMA14 / NEMA17 |
| 人机交互 | 2× 按键、状态 LED、拨码开关 | 拨码开关设定 CAN 节点 ID (ID0~ID2) |

### 片上外设使用
- **SPI**：读取 MT6816 编码器
- **TIM1**：100 Hz 主循环中断（按键 / LED / 轨迹下发 / 温度采样）
- **TIM4**：20 kHz 闭环控制中断（FOC 换相 tick）
- **TIM**（PWM）：驱动 TB67H450 电流设定
- **CAN**：总线指令接口
- **USART1 + DMA**：调试与上位机数据交互
- **ADC**：内部温度传感器，过温监测
- **GPIO**：按键、LED、拨码开关 (PA8/PA9/PA10)

---

## 🎮 通信指令

### UART 指令（ASCII，`UserApp/protocols/interface_uart.cpp`）

| 指令 | 功能 |
|------|------|
| `s` | 停止（进入 STOP 模式） |
| `c <电流A>` | 电流 / 力矩模式 |
| `v <速度r/s>` | 速度模式 |
| `p <位置r>` | 位置模式 |
| `a <...>` | 加速度相关设置 |
| `h` | 启动内置轨迹跟踪演示 |
| `z` | 触发编码器校准 |
| `i` | 打印当前配置（节点 ID 与 PID 参数） |
| `kp/kv/ki/kd <值>` | 在线整定 DCE 控制器参数并保存到 EEPROM |
| `r` | 复位 / 恢复 |

### CAN 指令（二进制，`UserApp/protocols/interface_can.cpp`）

| CMD | 功能 |
|-----|------|
| `0x01` | 使能 / 失能电机 |
| `0x02` | 执行编码器校准 |
| `0x03` | 设定电流 |
| `0x04` | 设定速度 |
| `0x07` | 带速度限制的位置模式 |
| `0x11`~`0x1B` | 设定并保存：节点 ID / 电流限制 / 速度限制 / 加速度 / 零位 / 上电自启 / PID / 堵转保护 |
| `0x21` | 回传速度与加速度 |

---

## 📁 项目结构（必要文件）

```
.
├── CMakeLists.txt              # 构建脚本（arm-none-eabi 交叉编译）
├── STM32F03CBT6.ioc           # STM32CubeMX 工程（外设 / 引脚配置）
├── STM32F103CBTX_FLASH.ld     # 链接脚本
├── stlink.cfg                 # OpenOCD/ST-Link 烧录配置
│
├── Core/                      # CubeMX 生成的 HAL 外设初始化
│   ├── Inc/                   #   main.h, *_it.h, 外设头文件
│   ├── Src/                   #   main.c, 外设驱动, 中断服务
│   └── Startup/               #   startup_stm32f103cbtx.s 启动文件
│
├── Drivers/                   # ST 官方 HAL 库 + CMSIS（第三方依赖）
│
├── Ctrl/                      # ★ 平台无关的核心控制逻辑
│   ├── Motor/                 #   电机控制器 + 运动规划 (motor, motion_planner)
│   ├── Driver/                #   TB67H450 驱动抽象 + 正弦表 (sin_map.h)
│   ├── Sensor/Encoder/        #   MT6816 编码器 + 校准器
│   └── Signal/                #   按键 / LED 抽象
│
├── Port/                      # ★ 硬件移植层（将 Ctrl 抽象绑定到 STM32）
│   ├── *_stm32.cpp/.h         #   编码器/驱动/按键/LED 的 STM32 实现
│   └── Platform/              #   EEPROM 模拟、retarget(printf)、芯片工具
│
└── UserApp/                   # ★ 应用层
    ├── main.cpp               #   主入口、定时器中断回调、按键事件
    ├── configurations.h       #   板级配置结构体 (BoardConfig_t)
    ├── common_inc.h
    └── protocols/             #   UART / CAN 指令解析
```

> `Ctrl/`、`Port/`、`UserApp/` 三层为本项目核心代码；`Core/` 与 `Drivers/` 由 STM32CubeMX 与 ST HAL 库生成，构建时必需但不是手写逻辑。
> `cmake-build-*/`（构建产物）与 `.idea/`（IDE 状态）已通过 `.gitignore` 排除。

### 架构分层
```
UserApp (应用 / 协议)
   │
Ctrl (平台无关控制算法)  ←—抽象基类 (driver_base / encoder_base ...)
   │
Port (STM32 移植实现)
   │
Core + Drivers (STM32 HAL / CMSIS)
```

---

## 🛠️ 构建与烧录

依赖工具链：`arm-none-eabi-gcc`、`cmake`、`openocd`（或 ST-Link）。

```bash
# 配置 + 编译
cmake -B build -G "Unix Makefiles"
cmake --build build

# 使用 ST-Link / OpenOCD 烧录
openocd -f stlink.cfg -c "program build/STM32F03CBT6.elf verify reset exit"
```

也可直接使用 **CLion** 或 **STM32CubeIDE** 打开本工程编译调试。

---

## ⚙️ 使用流程

1. 拨码开关设定 CAN 节点 ID（多关节组网时）。
2. **首次上电校准**：同时按住两个按键上电（或发送 UART `z` / CAN `0x02`），等待编码器自动标定完成。
3. 校准参数自动写入 EEPROM，之后正常上电即进入默认模式。
4. 通过 UART 或 CAN 下发控制指令，或使用 `h` 指令运行内置轨迹跟踪演示。

---

## 📷 控制原理简述

20 kHz 控制中断中：读取 MT6816 角度 → 计算电角度 → DCE 控制器输出力矩指令 → 经正弦表换相 → 设定 TB67H450 两相电流。100 Hz 循环负责运动规划、轨迹点下发、按键 / LED / 温度等慢任务。

---

## 🙏 致谢与参考

本驱动板项目参考自稚晖君（peng-zhihui）的开源机械臂项目 **[Dummy-Robot](https://github.com/peng-zhihui/Dummy-Robot)** 中的 **Ctrl-Step** 闭环步进电机驱动方案，沿用了其 MT6816 编码器 + TB67H450 驱动 + STM32 FOC 闭环的硬件选型，以及 `Ctrl`（平台无关算法）/ `Port`（硬件移植）的分层架构。在此基础上针对本项目需求做了修改与扩展（如 UART 轨迹跟踪、力矩前馈等）。

> 更多参考资料后续补充。
