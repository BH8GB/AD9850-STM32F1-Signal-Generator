#include "encoder.h"
#include "stm32f1xx_ll_bus.h"

/* 输入滤波：16 个采样周期、连续 8 次一致 */
#define ENC_FILTER      LL_TIM_IC_FILTER_FDIV16_N8

/* X4 解码下 4 个计数 = 机械一格；攒够一格才输出 1，零头留在 rem 里抵消抖动。
 * 编码器一格只给 2 个计数的话，这里改成 2。
 */
#define ENC_CNT_PER_STEP   4

/* 旋钮方向：1 = 取反。两只分开设 */
#define ENC_INVERT_FREQ    1
#define ENC_INVERT_AMP     1

static uint16_t s_last[2];
static int16_t  s_rem[2];       /* 不足一格的零头计数 */
static const uint8_t s_inv[2] = { ENC_INVERT_FREQ, ENC_INVERT_AMP };

/* PA15/PB3/PB4/PB5 都是重映射脚，必须打开 AFIO 定时器重映射，
 * 否则 TIM2/TIM3 还在默认引脚（PA0 是 MCP4921 的 CS、PA7 是 SPI1_MOSI）。
 * 这几个脚同时是 JTAG 脚，main.c 已经关 JTAG 留 SWD。
 */
static void enc_remap(void)
{
    uint32_t m;

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);

    /* 不用 LL_GPIO_AF_RemapPartial* 那几个内联函数：它们会把 SWJ_CFG
     * 写成非法值。这里自己读写 MAPR，保持"JTAG 关 / SWD 留"。
     */
    m  = AFIO->MAPR;
    m &= ~(AFIO_MAPR_TIM2_REMAP | AFIO_MAPR_TIM3_REMAP | AFIO_MAPR_SWJ_CFG);
    m |= AFIO_MAPR_TIM2_REMAP_PARTIALREMAP1    /* TIM2_CH1=PA15, CH2=PB3 */
       | AFIO_MAPR_TIM3_REMAP_PARTIALREMAP     /* TIM3_CH1=PB4,  CH2=PB5 */
       | AFIO_MAPR_SWJ_CFG_JTAGDISABLE;        /* JTAG 关，SWD 留 */
    AFIO->MAPR = m;
}

/* CubeMX 配的是浮空输入，悬空时计数器会被噪声打飞，这里强制改上拉 */
static void enc_gpio_pullup(void)
{
    LL_GPIO_InitTypeDef gi = {0};

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

    gi.Mode = LL_GPIO_MODE_INPUT;
    gi.Pull = LL_GPIO_PULL_UP;

    gi.Pin = LL_GPIO_PIN_15;                            /* PA15 = TIM2_CH1 */
    LL_GPIO_Init(GPIOA, &gi);

    gi.Pin = LL_GPIO_PIN_3 | LL_GPIO_PIN_4 | LL_GPIO_PIN_5;   /* PB3=TIM2_CH2 PB4/PB5=TIM3_CH1/CH2 */
    LL_GPIO_Init(GPIOB, &gi);
}

static TIM_TypeDef *enc_tim(enc_id_t id)
{
    return (id == ENC_FREQ) ? TIM2 : TIM3;
}

static void enc_setup(TIM_TypeDef *tim)
{
    /* CubeMX 给的是 X2_TI1(2倍频) + 无滤波，都要改 */
    LL_TIM_SetEncoderMode(tim, LL_TIM_ENCODERMODE_X4_TI12);
    LL_TIM_IC_SetFilter(tim, LL_TIM_CHANNEL_CH1, ENC_FILTER);
    LL_TIM_IC_SetFilter(tim, LL_TIM_CHANNEL_CH2, ENC_FILTER);
    LL_TIM_SetCounter(tim, 0);
    LL_TIM_EnableCounter(tim);
}

void encoder_init(void)
{
    enc_remap();
    enc_gpio_pullup();
    enc_setup(TIM2);
    enc_setup(TIM3);
    s_last[0] = 0;
    s_last[1] = 0;
    s_rem[0]  = 0;
    s_rem[1]  = 0;
}

/* 返回自上次调用以来转过几格。负数=反转。不足一格的零头留到下次 */
int16_t encoder_delta(enc_id_t id)
{
    uint16_t now = (uint16_t)LL_TIM_GetCounter(enc_tim(id));
    int16_t  d   = (int16_t)(now - s_last[id]);
    int32_t  acc;
    int16_t  step;

    s_last[id] = now;

    if (s_inv[id] != 0u) { d = (int16_t)(-d); }         /* 旋钮方向取反 */

    acc  = (int32_t)s_rem[id] + (int32_t)d;
    step = (int16_t)(acc / ENC_CNT_PER_STEP);           /* 负数除法朝 0 截断 */
    s_rem[id] = (int16_t)(acc - (int32_t)step * ENC_CNT_PER_STEP);

    return step;
}

/* -------------------------------------------------------------------------- */

#define BTN_PORT     GPIOA
#define BTN_PIN_KEY  LL_GPIO_PIN_2                      /* 唯一的按键 PA2 */
#define BTN_PIN_IDLE (LL_GPIO_PIN_3 | LL_GPIO_PIN_4)    /* 已空置，留上拉 */

void buttons_init(void)
{
    LL_GPIO_InitTypeDef gi = {0};

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
    gi.Pin  = BTN_PIN_KEY | BTN_PIN_IDLE;
    gi.Mode = LL_GPIO_MODE_INPUT;
    gi.Pull = LL_GPIO_PULL_UP;
    LL_GPIO_Init(BTN_PORT, &gi);
}

/* 只做瞬时电平判断，消抖交给调用方的周期任务 */
bool button_down(void)
{
    return (LL_GPIO_IsInputPinSet(BTN_PORT, BTN_PIN_KEY) == 0);
}
