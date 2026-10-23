#ifndef __UI_H
#define __UI_H

#include <stdint.h>

/* 信号发生器界面层
 *   [0] 正弦 SINE  AD9850 OUT1 -> AD605#1，幅度走 MCP4725 0x60
 *   [1] 方波 SQR   AD9850 OUT3 -> AD605#2，幅度走 MCP4725 0x61
 *   [2] 锯齿 SAW   MCP4921（SPI1），幅度在数字域缩放
 * 正弦/方波共用一颗 AD9850，只能同频；两路各记自己的频率和幅度，
 * 切到哪路才写哪路的参数。
 * 编码器 1(TIM2) 调当前通道频率，编码器 2(TIM3) 调幅度。
 * 按键 PA2：短按换步进档，双击总输出 ON/OFF，长按切波形。
 */

typedef enum {
    UI_CH_SINE = 0,
    UI_CH_SQUARE,
    UI_CH_SAW,
    UI_CH_COUNT
} ui_ch_t;

void     ui_init(void);                 /* 外设 + 屏 + 默认参数，最后刷一屏 */
void     ui_task(void);                 /* 主循环里反复调用，内部自己限速 */

ui_ch_t  ui_current(void);
uint64_t ui_freq_cHz(ui_ch_t ch);
uint16_t ui_amp_mVpp(ui_ch_t ch);
uint8_t  ui_output_on(void);

#endif /* __UI_H */
