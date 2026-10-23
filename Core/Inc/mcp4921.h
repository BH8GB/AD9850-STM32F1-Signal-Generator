#ifndef __MCP4921_H
#define __MCP4921_H

#include "stm32f1xx_ll_spi.h"
#include "stm32f1xx_ll_tim.h"
#include <stdint.h>
#include <stdbool.h>

/* MCP4921 锯齿发生：SPI1(PA5=SCK PA7=MOSI)，CS=PA0
 * CS 两条边沿由 TIM4 CH1/CH3 比较事件经 DMA 翻，数据由 DMA1_CH7 从采样表
 * 搬进 SPI1->DR，全程不占 CPU。VDD=VREF=5V，VOUT=VREF*D/4096，LDAC 接地，
 * 靠 CS 上升沿锁存。采样节拍用 TIM4（本文件自建）
 */

#define MCP4921_POINTS_MAX      4096u   /* 表长（8KB RAM），低频台阶细 */
#define MCP4921_POINTS_MIN      4u      /* 最高频只给 4 点 */
#define MCP4921_SAMPLE_MAX_HZ   600000u /* 采样率上限，TIM4 最快 120 拍 @72MHz */
#define MCP4921_FREQ_MIN_mHz    1000u   /* 1Hz */
#define MCP4921_FREQ_MAX_mHz    ((MCP4921_SAMPLE_MAX_HZ / MCP4921_POINTS_MIN) * 1000u)

/* 调试探针，挂调试器直接读 */
extern volatile uint32_t g_saw_isr;   /* TIM4 中断次数，约等于采样率 */
extern volatile uint32_t g_saw_late;  /* 上一帧没发完就抬 CS 的次数，恒 0 才对 */
extern volatile uint32_t g_saw_arr;   /* 当前 TIM4 重载值 */
extern volatile uint16_t g_saw_pts;   /* 当前每周期点数 */
extern volatile uint16_t g_saw_amp;   /* 当前满幅码值 0~4095 */

void mcp4921_init(void);
void mcp4921_write(uint16_t code);

/* 启停锯齿。freq_mHz 单位毫赫兹(1Hz=1000)，amp_permille 0~1000 */
bool mcp4921_ramp_start(uint32_t freq_mHz, uint16_t amp_permille);
void mcp4921_ramp_set_freq(uint32_t freq_mHz);
void mcp4921_ramp_set_amp(uint16_t amp_permille);
void mcp4921_ramp_stop(void);

#endif
