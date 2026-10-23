#include "ad9850.h"

#include "stm32f1xx_ll_bus.h"

#define DDS_PORT        GPIOA

/* F1 的 LL_GPIO_PIN_x 是复合编码，只能喂 LL_GPIO_Init()，
 * 写 BSRR/BRR 要用下面的裸掩码。
 */
#define DDS_MASK_D7     (1UL << 8)
#define DDS_MASK_CLK    (1UL << 9)
#define DDS_MASK_FQ     (1UL << 10)
#define DDS_MASK_RST    (1UL << 11)
#define DDS_MASK_ALL    (DDS_MASK_D7 | DDS_MASK_CLK | DDS_MASK_FQ | DDS_MASK_RST)

#define DDS_INIT_PINS   (LL_GPIO_PIN_8 | LL_GPIO_PIN_9 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11)

static uint32_t s_ftw      = 0u;
static uint64_t s_freq_cHz = 0u;

static void dds_delay(void)
{
    volatile uint32_t i;
    for (i = 0u; i < 24u; i++) { }
}

/* 先建立数据再抬高 W_CLK，保证 tDS >= 3.5ns */
static void dds_bit(uint8_t b)
{
    DDS_PORT->BSRR = b ? DDS_MASK_D7 : (DDS_MASK_D7 << 16);
    DDS_PORT->BSRR = DDS_MASK_CLK;
    DDS_PORT->BRR  = DDS_MASK_CLK;
}

void ad9850_init(void)
{
    LL_GPIO_InitTypeDef gi = {0};

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);

    gi.Pin        = DDS_INIT_PINS;
    gi.Mode       = LL_GPIO_MODE_OUTPUT;
    gi.Speed      = LL_GPIO_SPEED_FREQ_HIGH;
    gi.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    gi.Pull       = LL_GPIO_PULL_DOWN;   /* F1 输出模式下不起作用，仅为字段完整 */
    LL_GPIO_Init(DDS_PORT, &gi);

    DDS_PORT->BRR = DDS_MASK_ALL;
    ad9850_reset();

    /* 上电先给一个已知频率，方便示波器确认链路 */
    (void)ad9850_set_freq_cHz(100000ULL);        /* 1kHz */
}

/* RESET 脉冲 -> W_CLK 脉冲(把全 0 装入输入寄存器) -> FQ_UD 脉冲(转存到频率寄存器) */
void ad9850_reset(void)
{
    DDS_PORT->BSRR = DDS_MASK_RST;
    dds_delay();
    DDS_PORT->BRR  = DDS_MASK_RST;
    dds_delay();

    DDS_PORT->BSRR = DDS_MASK_CLK;
    dds_delay();
    DDS_PORT->BRR  = DDS_MASK_CLK;
    dds_delay();

    DDS_PORT->BSRR = DDS_MASK_FQ;
    dds_delay();
    DDS_PORT->BRR  = DDS_MASK_FQ;
    dds_delay();

    s_ftw      = 0u;
    s_freq_cHz = 0u;
}

/* FTW = round(f_cHz * 2^32 / (fs * 100))；40MHz 时分子仍在 uint64 范围内 */
uint32_t ad9850_ftw_from_cHz(uint64_t f_cHz)
{
    uint64_t num = f_cHz * AD9850_FTW_NUM + AD9850_FTW_HALF;
    return (uint32_t)(num / AD9850_FTW_DEN);
}

uint32_t ad9850_ftw_from_hz(double f_hz)
{
    if (f_hz < 0.0) {
        f_hz = 0.0;
    }
    return ad9850_ftw_from_cHz((uint64_t)(f_hz * 100.0 + 0.5));
}

/* 40bit 串行帧：32 位频率字 LSB 先出 + 8 位控制字，FQ_UD 上升沿锁存 */
void ad9850_write_raw(uint8_t w0, uint32_t ftw)
{
    uint8_t i;

    for (i = 0u; i < 32u; i++) {
        dds_bit((uint8_t)((ftw >> i) & 1u));
    }
    for (i = 0u; i < 8u; i++) {
        dds_bit((uint8_t)((w0 >> i) & 1u));
    }

    DDS_PORT->BSRR = DDS_MASK_FQ;
    dds_delay();
    DDS_PORT->BRR  = DDS_MASK_FQ;
}

bool ad9850_set_freq_cHz(uint64_t f_cHz)
{
    if (f_cHz < AD9850_FREQ_MIN_cHz || f_cHz > AD9850_FREQ_MAX_cHz) {
        return false;
    }
    s_freq_cHz = f_cHz;
    s_ftw      = ad9850_ftw_from_cHz(f_cHz);
    ad9850_write_raw(AD9850_CTRL_BYTE, s_ftw);
    return true;
}

bool ad9850_set_freq_hz(double f_hz)
{
    if (f_hz < 1.0 || f_hz > 40000000.0) {
        return false;
    }
    return ad9850_set_freq_cHz((uint64_t)(f_hz * 100.0 + 0.5));
}

bool ad9850_set_ftw(uint32_t ftw)
{
    /* 1 个 LSB = fs*100/2^32 厘赫兹；用 double 换算避免 uint64 中间量溢出 */
    s_freq_cHz = (uint64_t)((double)ftw * (12500000000.0 / 4294967296.0) + 0.5);
    s_ftw      = ftw;
    ad9850_write_raw(AD9850_CTRL_BYTE, ftw);
    return true;
}

uint32_t ad9850_get_ftw(void)
{
    return s_ftw;
}

uint64_t ad9850_get_freq_cHz(void)
{
    return s_freq_cHz;
}

double ad9850_get_freq_hz(void)
{
    return (double)s_freq_cHz / 100.0;
}
