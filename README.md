# AD9850 三路信号发生器（STM32F103C8T6）

基于 AD9850 DDS 模块 + AD605 可变增益放大器 + MCP4921 DAC 的三路信号源：
正弦、方波、锯齿，频率和幅度用两只 EC11 编码器调，128x64 OLED 显示。

固件是裸机超级循环（不启 FreeRTOS），全部用 LL 库写成。

## 功能

| 通道 | 产生方式 | 频率范围 | 幅度范围 |
|---|---|---|---|
| 正弦 SINE | AD9850 OUT1 -> AD605#1 | 2 kHz ~ 40 MHz | 0 ~ 2.00 Vpp |
| 方波 SQR | AD9850 OUT3 -> AD605#2 | 2 kHz ~ 40 MHz | 0.10 ~ 1.30 Vpp |
| 锯齿 SAW | STM32 + MCP4921 直出 | 1 Hz ~ 100 kHz | 0 ~ 3.50 Vpp |

- 频率步进 1MHz / 100kHz / 10kHz / 1kHz / 100Hz / 10Hz / 1Hz / 0.1Hz / 0.01Hz，短按 PA2 切换档位
- 正弦/方波最细到 1kHz 档（本身在 MHz 段），锯齿 9 档全用
- 幅度步进 0.01 Vpp
- 三路各自记忆自己的频率和幅度，切换波形不会丢参数
- 开机三路默认关闭，屏幕第一行显示 OFF
- 正弦/方波共用一颗 AD9850，物理上只能同频；切换到哪路就把那路的频率写进 AD9850

## 硬件组成

| 器件 | 说明 |
|---|---|
| STM32F103C8T6 | 主控，72MHz，20KB RAM / 64KB Flash |
| AD9850 模块 | 125MHz 晶振，串行模式，模块自带低通滤波 |
| AD605 | 双通道可变增益放大器，正弦/方波各走一路 |
| MCP4725 x2 | 12 位 I2C DAC，输出 VGN 控制 AD605 的增益 |
| MCP4921 | 12 位 SPI DAC，直接产生锯齿波 |
| SSD1306 | 128x64 OLED，I2C |
| EC11 x2 | 带按压开关的旋转编码器 |

AD605 侧：`VOCM = VREF = 2.5V`，`FBK` 短接 `OUT`，此时增益
`G(dB) = 20*VGN(V) - 19`，可用 VGN 0.1~2.9V，VGN < 50mV 整通道掉电。

## 引脚接线

| 功能 | 引脚 | 备注 |
|---|---|---|
| AD9850 D7 | PA8 | 串行数据 |
| AD9850 W_CLK | PA9 | |
| AD9850 FQ_UD | PA10 | |
| AD9850 RESET | PA11 | |
| MCP4921 SCK | PA5 | SPI1 |
| MCP4921 MOSI | PA7 | SPI1 |
| MCP4921 CS | PA0 | 由 TIM4 比较事件经 DMA 翻转 |
| MCP4921 LDAC | GND | 靠 CS 上升沿锁存 |
| OLED SCL / SDA | PB6 / PB7 | I2C1，400kHz，地址 0x3C/0x3D 自动探测 |
| MCP4725 SCL / SDA | PB10 / PB11 | I2C2，100kHz |
| MCP4725 #1 | 0x60 | 正弦那路 AD605 的 VGN |
| MCP4725 #2 | 0x61 | 方波那路 AD605 的 VGN |
| 编码器 1 A/B | PA15 / PB3 | TIM2 编码器模式，调频率 |
| 编码器 2 A/B | PB4 / PB5 | TIM3 编码器模式，调幅度 |
| 按键 | PA2 | 两只 EC11 的按压开关并联 |

注意：

- PA15/PB3/PB4/PB5 是 JTAG 脚，`main.c` 已关 JTAG、保留 SWD，并打开 TIM2/TIM3 部分重映射。
- MCP4725/MCP4921 用 5V 供电（MCP4725 没有外部 VREF，基准就是 VDD）。
- AD9850 模块侧 D0/D1 上拉、D2 下拉，即串行模式；D3~D6 悬空。

## 目录结构

```
Core/
  Inc/     ad9850.h  mcp4725.h  mcp4921.h  ssd1306.h  encoder.h  ui.h  ...
  Src/     ad9850.c  mcp4725.c  mcp4921.c  ssd1306.c  encoder.c  ui.c  main.c
Drivers/   STM32F1xx LL 库与 CMSIS
MDK-ARM/   Keil 工程（F1AD9805.uvprojx）
```

`ui.c` 是界面层，负责参数、编码器/按键处理、屏显和下发硬件；
各外设驱动在对应的 `.c` 里，`main.c` 只做初始化和调用 `ui_init()/ui_task()`。

## 编译与烧录

编译环境：Keil MDK 5，ARMCC V5.06，target `F1AD9805`。

1. 打开 `MDK-ARM/F1AD9805.uvprojx`
2. Build（F7）
3. 用 ST-Link 下载（F8），或烧 `MDK-ARM/F1AD9805/F1AD9805.hex`

命令行编译：

```bat
UV4.exe -b MDK-ARM\F1AD9805.uvprojx -t F1AD9805 -j0 -o build.log
```

用 pyocd 烧写（本机验证过）：

```bat
pyocd cmd -t stm32f103rc -f 500k -O connect_mode=under-reset ^
  -c halt -c erase -c "load MDK-ARM/F1AD9805/F1AD9805.hex" -c reset -c go
```

正常编译结果约 `Code=11KB / ZI=12KB`，20KB RAM 够用。

## 操作说明

两只编码器：

- 编码器 1（TIM2）：调当前通道频率
- 编码器 2（TIM3）：调当前通道幅度

按键 PA2（两只编码器按压开关并联）：

| 动作 | 功能 |
|---|---|
| 短按 | 切换频率步进档 |
| 双击 | 总输出 ON/OFF |
| 长按 600ms | 正弦 -> 方波 -> 锯齿 循环切换 |

屏幕布局：

```
SINE        0.88Vpp        OFF
      1.000000 MHz
STEP  1kHz
```

- 第一行：波形名、当前幅度（实测 Vpp）、总输出状态
- 中间：频率大字，当前调频位反白
- 第三行：频率步进档

频率单位：正弦/方波自动切 Hz/kHz/MHz；锯齿固定用 kHz（`0.100kHz`、`100.00kHz`）。

## 幅度控制与标定

MCP4725 输出 VGN，按固定比例把屏上 Vpp 换成 DAC 电压：

```
DAC(mV) = Vpp(mV) * 3000 / 3500
```

所以正弦 2.00 Vpp 对应 1714mV，方波 1.30 Vpp 对应 1114mV，不会顶到 3V。

屏幕显示的是**实测的真实输出 Vpp**，用下面两组标定点做分段线性插值
（`ui.c` 里的 `s_show_sin_x/y`、`s_show_sqr_x/y`）：

| 正弦 | -20dB | -5dB | -1dB | 0dB | 满幅 |
|---|---|---|---|---|---|
| 输出 | 0.23V | 0.38V | 0.72V | 0.88V | 3.50Vpp |
| 设定 | 100 | 562 | 891 | 1000 | 1767+ mVpp |

| 方波 | -20dB | -9.9dB | -3.9dB | 满幅 |
|---|---|---|---|---|
| 输出 | 1.00V | 1.20V | 2.00V | 3.80Vpp |
| 设定 | 100 | 320 | 638 | 1300 mVpp |

正弦在设定值 1767mVpp 以上实际已到 3.50Vpp，屏显也就钉在 3.50Vpp。

## 关键实现

**AD9850**：GPIO 位操作串行写 40bit（32 位频率字 + 8 位控制字），
`FTW = round(f * 2^32 / (125MHz * 100))`，固定 125MHz、不开 6 倍频。

**MCP4921 锯齿**：全硬件、0% CPU。

- 采样表 4096 点（8KB），低频每周期最多 4096 点，台阶约 1.2mV
- TIM4 更新事件 -> DMA1_CH7 -> `SPI1->DR`（16 位命令字）
- TIM4 CH1 比较 -> DMA1_CH1 写 `GPIOA->BRR` 拉低 CS
- TIM4 CH3 比较 -> DMA1_CH5 写 `GPIOA->BSRR` 拉高 CS 锁存
- 采样率上限 600kHz（SPI 18MHz，一帧 16 位约 0.89us，CS 低电平要盖住整帧）

**I2C 总线自愈**：F1 的 I2C 一旦 BUSY 卡住只能 SWRST 复位，
代码里做了"复位外设 + 引脚当 GPIO 敲 9 个 SCL + 补 STOP"的恢复链，
开机和写失败时都会自动调用。

## 常改参数速查

| 宏 | 文件 | 作用 |
|---|---|---|
| `UI_FREQ_MIN_cHz` / `UI_FREQ_MAX_DDS_cHz` | ui.c | 正弦/方波频率范围 |
| `UI_FREQ_MAX_SAW_cHz` | ui.c | 锯齿频率上限（当前 100kHz） |
| `UI_AMP_MAX_SINE_mVpp` 等 | ui.c | 各通道幅度范围 |
| `UI_AMP_STEP_mVpp` | ui.c | 幅度步进（当前 10mV） |
| `AMP_MAP_DB` | ui.c | 0=线性映射，1=按 AD605 增益曲线反推 |
| `AMP_SHOW_DB` | ui.c | 0=屏显 Vpp，1=屏显 dB |
| `MCP4725_DAC_MAX_mV` / `MCP4725_DAC_REF_mVpp` | ui.c | 幅度到 DAC 电压的比例 |
| `s_show_*_x/y` | ui.c | 屏显 Vpp 的实测标定表 |
| `MCP4921_POINTS_MAX` / `MCP4921_SAMPLE_MAX_HZ` | mcp4921.h | 锯齿表长与采样率 |
| `MCP4921_ISR_MODE` | mcp4921.c | 0=全 DMA，1=中断版 |
| `ENC_INVERT_FREQ` / `ENC_INVERT_AMP` | encoder.c | 旋钮方向取反 |
| `APP_BRINGUP_MODE` | main.c | 6=完整应用，其余为硬件自检模式 |

## 已知限制

- 正弦和方波共用 AD9850，两者只能同频（各自记住自己的设定值）
- 锯齿高频时每周期点数受 600kHz 采样率限制：10kHz 约 32 点，100kHz 只有 4 点，台阶明显
- AD605 最低增益约 -19dB（VGN=0），再低只能靠掉电静音
- 方波幅度上限 1.30Vpp 是软件量程限制，改 `UI_AMP_MAX_SQR_mVpp` 可放开
- 屏幕 Vpp 是实测点的分段线性近似，中间值不保证精确
