#ifndef __AD9850_H
#define __AD9850_H

#include "stm32f1xx_ll_gpio.h"
#include <stdint.h>
#include <stdbool.h>

/* AD9850 串行模式，125MHz 晶振、不开 6 倍频
 * PA8=D7  PA9=W_CLK  PA10=FQ_UD  PA11=RESET
 */

#define AD9850_REFCLK_HZ        125000000UL     /* 模块晶振 */

/* 频率用厘赫兹(0.01Hz)；频率字用 64 位整数算（AC5 无 __int128） */
#define AD9850_FREQ_MIN_cHz     100ULL          /* 1.00 Hz */
#define AD9850_FREQ_MAX_cHz     4000000000ULL   /* 40.00 MHz */

#define AD9850_FTW_NUM          4294967296ULL   /* 2^32 */
#define AD9850_FTW_DEN          12500000000ULL  /* fs * 100 */
#define AD9850_FTW_HALF         6250000000ULL

/* 控制字 bit0=6倍频 bit1..2=掉电 bit3..7=相位；125MHz 晶振固定 0x00 */
#define AD9850_CTRL_BYTE        0x00u

void     ad9850_init(void);                 /* GPIO + 复位，默认输出 1kHz */
void     ad9850_reset(void);

uint32_t ad9850_ftw_from_cHz(uint64_t f_cHz);
uint32_t ad9850_ftw_from_hz(double f_hz);

bool     ad9850_set_freq_cHz(uint64_t f_cHz);
bool     ad9850_set_freq_hz(double f_hz);
bool     ad9850_set_ftw(uint32_t ftw);      /* 直接写频率字，不查范围 */

uint32_t ad9850_get_ftw(void);
uint64_t ad9850_get_freq_cHz(void);
double   ad9850_get_freq_hz(void);

void     ad9850_write_raw(uint8_t w0, uint32_t ftw);

#endif /* __AD9850_H */
