#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f1xx_ll_tim.h"
#include "stm32f1xx_ll_gpio.h"
#include <stdint.h>
#include <stdbool.h>

/* 编码器 1 调频 TIM2: PA15=CH1 PB3=CH2
 * 编码器 2 调幅 TIM3: PB4=CH1 PB5=CH2
 * 按键只有 PA2（两只 EC11 的按压开关并联），动作判定在 ui.c
 */

typedef enum {
    ENC_FREQ = 0,
    ENC_AMP  = 1
} enc_id_t;

void    encoder_init(void);
int16_t encoder_delta(enc_id_t id);

void    buttons_init(void);
bool    button_down(void);      /* true = PA2 当前被按下（未消抖） */

#endif
