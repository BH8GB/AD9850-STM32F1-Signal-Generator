#include "mcp4921.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_gpio.h"

/* 0 = 全硬件（默认）：三个 DMA 挂 TIM4，不用 CPU
 *     CNT=ccr_lo -> DMA1_CH1 写 BRR  拉低 CS
 *     CNT=ccr_hi -> DMA1_CH5 写 BSRR 拉高 CS，锁存刚发完的一帧
 *     更新事件   -> DMA1_CH7 写 SPI1->DR，搬下一个码字
 * 1 = 老方案：TIM4 中断里一拍一点，代码短但很吃 CPU
 */
#define MCP4921_ISR_MODE    0

#define TIM4_CLK_HZ         72000000UL
#define MCP4921_CFG_BASE    0x3000u   /* GA=1 SHDN=1 -> VOUT = VREF*D/4096 */

/* 采样周期 120 拍 @72MHz = 600kHz；SPI 一帧 16 位 @18MHz 约 64 拍，
 * CS 低电平从上一周期的 110 一直盖到本周期的 96，够宽
 */
#define SAW_ARR             119u
#define CS_HI_TICKS         96u      /* CS 拉高（锁存） */
#define CS_LO_TICKS         110u     /* CS 拉低（在周期末尾，给下一帧开路） */

#define SPI_SR_TXE_         (1UL << 1)   /* 发送缓冲空 */
#define SPI_SR_BSY_         (1UL << 7)   /* 正在发送 */

#define CS_MASK             (1UL << 0)   /* PA0 */

static uint16_t s_tab[MCP4921_POINTS_MAX];   /* 表里直接存 16 位命令字 */
static volatile uint16_t s_idx;
static volatile uint16_t s_n;
static volatile uint16_t s_mask;
static volatile uint16_t s_amp;
static volatile uint8_t  s_running;

/* 两个 CS 通道都从这一个格子取值：写 BRR 就是拉低，写 BSRR 就是拉高 */
static uint32_t s_cs_val = 1u;

volatile uint32_t g_saw_isr  = 0u;   /* 老方案的中断计数；全硬件模式下恒 0 */
volatile uint32_t g_saw_late = 0u;
volatile uint32_t g_saw_arr  = 0u;   /* 当前 TIM4 重载值，反推采样率用 */
volatile uint16_t g_saw_pts  = 0u;   /* 当前每周期点数 */
volatile uint16_t g_saw_amp  = 0u;   /* 当前满幅码值 0~4095 */

void mcp4921_write(uint16_t code)
{
    if (code > 4095u) { code = 4095u; }

    while ((SPI1->SR & SPI_SR_BSY_) != 0u) { }
    GPIOA->BSRR = CS_MASK;
    __NOP(); __NOP(); __NOP(); __NOP();
    GPIOA->BRR  = CS_MASK;
    __NOP(); __NOP();
    SPI1->DR = (uint32_t)(MCP4921_CFG_BASE | code);
    while ((SPI1->SR & SPI_SR_BSY_) != 0u) { }
    __NOP(); __NOP(); __NOP(); __NOP();
    GPIOA->BSRR = CS_MASK;
}

/* 填表：第 i 点 = i*amp/n，用定点累加代替逐点除法 */
static void ramp_build(uint16_t n, uint16_t amp)
{
    uint32_t step;
    uint32_t acc = 0u;

    if (n == 0u) { return; }

    step = (((uint32_t)amp) << 16) / (uint32_t)n;
    for (uint16_t i = 0u; i < n; i++) {
        s_tab[i] = (uint16_t)(MCP4921_CFG_BASE | (acc >> 16));
        acc += step;
    }
    g_saw_pts = n;
    g_saw_amp = amp;
}

#if !MCP4921_ISR_MODE
/* 一个 DMA 通道的通用配置：存储器 -> 外设，循环模式，最高优先级 */
static void dma_cfg(DMA_Channel_TypeDef *ch, uint32_t par, uint32_t mar,
                    uint16_t n, uint32_t psize, uint32_t msize, uint32_t minc)
{
    ch->CCR   = 0u;
    ch->CPAR  = par;
    ch->CMAR  = mar;
    ch->CNDTR = n;
    ch->CCR   = DMA_CCR_EN | DMA_CCR_PL | psize | msize | minc
              | DMA_CCR_CIRC | DMA_CCR_DIR;
}
#endif

/* 采样率 = 锯齿频率 x 每周期点数。ARR 同时决定两个 CS 边沿的位置。 */
static void tim4_set_rate(uint32_t sample_hz)
{
    uint32_t psc = 0u;
    uint32_t arr;

    if (sample_hz == 0u) { return; }
    while ((TIM4_CLK_HZ / (psc + 1u)) / sample_hz > 65536u) { psc++; }
    arr = (TIM4_CLK_HZ / (psc + 1u)) / sample_hz;
    if (arr < 104u) { arr = 104u; }      /* 再快 CS 低电平就盖不住一帧了 */

    g_saw_arr = arr;
    LL_TIM_SetPrescaler(TIM4, psc);
    LL_TIM_SetAutoReload(TIM4, arr - 1u);
    LL_TIM_GenerateEvent_UPDATE(TIM4);   /* 新参数立刻生效 */

#if !MCP4921_ISR_MODE
    /* 低频时边沿固定在 96/110；采样率顶上去时按 ARR 收窄，始终留 24 拍 */
    TIM4->CCR3 = (arr > 120u) ? CS_HI_TICKS : (arr - 24u);
    TIM4->CCR1 = (arr > 120u) ? CS_LO_TICKS : (arr - 10u);
#endif
}

#if !MCP4921_ISR_MODE
/* 改点数要关通道后把 CMAR 和 CNDTR 一起重设（F1 的 CMAR 存的是当前地址） */
static void dma_set_count(uint16_t n)
{
    DMA1_Channel7->CCR &= ~DMA_CCR_EN;
    while ((DMA1_Channel7->CCR & DMA_CCR_EN) != 0u) { }

    DMA1_Channel7->CMAR  = (uint32_t)s_tab;
    DMA1_Channel7->CNDTR = n;
    DMA1_Channel7->CCR  |= DMA_CCR_EN;
}
#endif

void mcp4921_init(void)
{
    LL_GPIO_InitTypeDef gi = {0};
    LL_TIM_InitTypeDef  ti = {0};

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);

    gi.Pin        = LL_GPIO_PIN_0;
    gi.Mode       = LL_GPIO_MODE_OUTPUT;
    gi.Speed      = LL_GPIO_SPEED_FREQ_HIGH;
    gi.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    gi.Pull       = LL_GPIO_PULL_UP;
    LL_GPIO_Init(GPIOA, &gi);
    GPIOA->BSRR = CS_MASK;                  /* 空闲 CS 为高 */

    /* SPI1 改 8 位帧为 16 位帧，分频 DIV2->DIV4（18MHz，MCP4921 上限 20MHz） */
    LL_SPI_Disable(SPI1);
    LL_SPI_SetDataWidth(SPI1, LL_SPI_DATAWIDTH_16BIT);
    LL_SPI_SetBaudRatePrescaler(SPI1, LL_SPI_BAUDRATEPRESCALER_DIV4);
    LL_SPI_SetNSSMode(SPI1, LL_SPI_NSS_SOFT);
    LL_SPI_Enable(SPI1);

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
    ti.Prescaler         = 0u;
    ti.CounterMode       = LL_TIM_COUNTERMODE_UP;
    ti.Autoreload        = SAW_ARR;
    ti.ClockDivision     = LL_TIM_CLOCKDIVISION_DIV1;
    ti.RepetitionCounter = 0u;
    LL_TIM_Init(TIM4, &ti);
    LL_TIM_EnableARRPreload(TIM4);

    s_n = 32u; s_mask = 31u; s_amp = 4095u;
    ramp_build(s_n, s_amp);

#if MCP4921_ISR_MODE
    NVIC_SetPriority(TIM4_IRQn, 6);
    NVIC_EnableIRQ(TIM4_IRQn);
    LL_TIM_EnableIT_UPDATE(TIM4);
#else
    /* CH1/CH3 只当比较事件用（CC1E=CC3E=0，不占引脚），到点让 DMA 翻 PA0 */
    TIM4->CCMR1 = (uint16_t)(((TIM4->CCMR1 & (uint16_t)~((3u << 0) | (7u << 4)))) | (2u << 4));
    TIM4->CCMR2 = (uint16_t)(((TIM4->CCMR2 & (uint16_t)~((3u << 0) | (7u << 4)))) | (2u << 4));
    TIM4->CCER &= (uint16_t)~((1u << 0) | (1u << 8));
    TIM4->CCR1  = CS_LO_TICKS;
    TIM4->CCR3  = CS_HI_TICKS;
    TIM4->DIER |= (1u << 9) | (1u << 11);       /* CC1DE, CC3DE */
    TIM4->DIER |= (1u << 8);                    /* UDE：更新事件 -> 数据 DMA */

    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    /* CH1 拉低 CS，CH5 拉高 CS：都是一次 32 位写，循环模式，一个周期一次 */
    dma_cfg(DMA1_Channel1, (uint32_t)&GPIOA->BRR, (uint32_t)&s_cs_val, 1u,
            DMA_CCR_PSIZE_1, DMA_CCR_MSIZE_1, 0u);
    dma_cfg(DMA1_Channel5, (uint32_t)&GPIOA->BSRR, (uint32_t)&s_cs_val, 1u,
            DMA_CCR_PSIZE_1, DMA_CCR_MSIZE_1, 0u);
    /* CH7 搬数据：更新事件一到就往 SPI1->DR 塞一个 16 位命令字 */
    dma_cfg(DMA1_Channel7, (uint32_t)&SPI1->DR, (uint32_t)s_tab, (uint16_t)s_n,
            DMA_CCR_PSIZE_0, DMA_CCR_MSIZE_0, DMA_CCR_MINC);
#endif
}

#if MCP4921_ISR_MODE
/* 老方案：一拍一点全在中断里；CS 上升沿锁存的是上一拍发完的那一帧 */
void TIM4_IRQHandler(void)
{
    TIM4->SR = ~TIM_SR_UIF;

    GPIOA->BSRR = CS_MASK;              /* 抬 CS：锁存上一拍 */
    __NOP(); __NOP(); __NOP();
    GPIOA->BRR  = CS_MASK;              /* 拉低：开这一拍 */

    if ((SPI1->SR & SPI_SR_TXE_) == 0u) { g_saw_late++; }
    SPI1->DR = s_tab[s_idx];

    s_idx = (uint16_t)((s_idx + 1u) & s_mask);
    g_saw_isr++;
}
#endif

bool mcp4921_ramp_start(uint32_t freq_mHz, uint16_t amp_permille)
{
    if (freq_mHz < MCP4921_FREQ_MIN_mHz || freq_mHz > MCP4921_FREQ_MAX_mHz) {
        return false;
    }
    mcp4921_ramp_set_amp(amp_permille);
    mcp4921_ramp_set_freq(freq_mHz);

    s_idx     = 0u;
    s_running = 1u;

#if !MCP4921_ISR_MODE
    dma_set_count(s_n);
    /* 停在周期末尾：使能后第一件事就是完整的一拍，表头从第 0 点开始 */
    LL_TIM_SetCounter(TIM4, SAW_ARR);
#endif
    LL_TIM_EnableCounter(TIM4);
    return true;
}

/* 频率越高每周期点数越少。点数始终取 2 的幂，索引才能用 & 代替取模。 */
void mcp4921_ramp_set_freq(uint32_t freq_mHz)
{
    uint32_t f_hz;
    uint32_t pts;
    uint32_t p;
    uint32_t sample_hz;

    if (freq_mHz == 0u) { return; }
    f_hz = (freq_mHz + 999u) / 1000u;                 /* 向上取整到 Hz */

    pts = MCP4921_SAMPLE_MAX_HZ / f_hz;               /* 不超采样率上限的点数 */
    if (pts > MCP4921_POINTS_MAX) { pts = MCP4921_POINTS_MAX; }
    if (pts < MCP4921_POINTS_MIN) { pts = MCP4921_POINTS_MIN; }
    p = 1u;
    while ((p << 1u) <= pts) { p <<= 1u; }            /* 向下取到 2 的幂 */
    pts = p;

#if !MCP4921_ISR_MODE
    /* 表就是 DMA 的数据源，填的时候先停搬，不然会发出半新半旧的一段 */
    DMA1_Channel7->CCR &= ~DMA_CCR_EN;
    while ((DMA1_Channel7->CCR & DMA_CCR_EN) != 0u) { }
#endif
    if ((uint16_t)pts != s_n) {                       /* 点数没变就不用重填 */
        ramp_build((uint16_t)pts, s_amp);
        s_mask = (uint16_t)(pts - 1u);
        s_n    = (uint16_t)pts;
    }

    sample_hz = (uint32_t)((((uint64_t)freq_mHz * pts) + 500ull) / 1000ull);
    if (sample_hz == 0u) { sample_hz = 1u; }
    tim4_set_rate(sample_hz);

#if !MCP4921_ISR_MODE
    dma_set_count((uint16_t)pts);                     /* 顺便把通道重新打开 */
#endif
}

void mcp4921_ramp_set_amp(uint16_t amp_permille)
{
    uint16_t a;

    if (amp_permille > 1000u) { amp_permille = 1000u; }
    a = (uint16_t)(((uint32_t)amp_permille * 4095u) / 1000u);

    if (a == s_amp) { return; }                       /* 幅度没变就别白填一遍表 */
    s_amp = a;
#if !MCP4921_ISR_MODE
    DMA1_Channel7->CCR &= ~DMA_CCR_EN;
    while ((DMA1_Channel7->CCR & DMA_CCR_EN) != 0u) { }
#endif
    ramp_build(s_n, a);                               /* 幅度变了整张表重填 */
#if !MCP4921_ISR_MODE
    dma_set_count(s_n);
#endif
}

void mcp4921_ramp_stop(void)
{
    s_running = 0u;
    LL_TIM_DisableCounter(TIM4);
#if !MCP4921_ISR_MODE
    DMA1_Channel7->CCR &= ~DMA_CCR_EN;
#endif
    mcp4921_write(0u);
}
