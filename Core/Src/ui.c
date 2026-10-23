#include "ui.h"

#include "ssd1306.h"
#include "ad9850.h"
#include "mcp4725.h"
#include "mcp4921.h"
#include "encoder.h"

/* ------------------------------------------------------- 标定用常量 */

/* 幅度 -> MCP4725 的映射方式
 * 0 = 线性（当前）：固定比例 3.50Vpp <-> 3.000V，各路同一个比例，DAC 直推 AD605 的 VGN
 * 1 = 按 AD605 增益曲线反推 VGN（G(dB)=20*VGN(V)-19，FBK 短接 OUT、VREF=2.5V）
 */
#define AMP_MAP_DB          0

#if (AMP_MAP_DB != 0)
#define ADP605_VIN_SINE_mVpp    1000u   /* 模块 OUT1 正弦约 1Vpp */
#define ADP605_VIN_SQR_mVpp     1000u   /* 模块 OUT3 方波输入幅度（标定用） */
#define ADP605_VGN_MIN_mV       100u    /* 手册可用下限 */
#define ADP605_VGN_MAX_mV       2900u   /* 手册可用上限 */
#endif

/* 各通道的幅度范围（mVpp），旋钮走到头就是这里的上下限 */
#define UI_AMP_MAX_SINE_mVpp    2000u   /* 正弦 0 ~ 2.00 Vpp */
#define UI_AMP_MIN_SINE_mVpp    0u
#define UI_AMP_MAX_SQR_mVpp     1300u   /* 方波 0.10 ~ 1.30 Vpp */
#define UI_AMP_MIN_SQR_mVpp     100u
#define UI_AMP_MAX_SAW_mVpp     3500u   /* 锯齿 0 ~ 3.50 Vpp */
#define UI_AMP_MIN_SAW_mVpp     0u
#define UI_AMP_STEP_mVpp    10u     /* 0.01 Vpp */
#define AMP_DB_REF_mVpp     1000u   /* 0 dB 参考：进 AD605 的 1Vpp（1:1 增益） */
/* 屏上幅度显示什么：0 = 实测的真实输出 Vpp（当前），1 = dB */
#define AMP_SHOW_DB         0
#define MCP4725_DAC_MAX_mV  3000u   /* 比例锚点：3.50 Vpp <-> 3.000 V，各路统一不变 */
#define MCP4725_DAC_REF_mVpp 3500u
#define MCP4921_FULL_mV     5000u   /* MCP4921 满幅 = VREF = 5.000V */

#define UI_FREQ_MIN_cHz     200000ULL        /* 2 kHz   正弦/方波下限 */
#define UI_FREQ_MIN_SAW_cHz 100ULL           /* 1 Hz    锯齿自己的下限 */
#define UI_FREQ_MAX_DDS_cHz 4000000000ULL    /* 40 MHz  正弦/方波 */
#define UI_FREQ_MAX_SAW_cHz 10000000ULL      /* 100.000 kHz 锯齿上限 */

/* 单键 PA2 的三个动作门限，单位毫秒 */
#define UI_BTN_DB_MS        20u    /* 消抖：稳定这么久才算数 */
#define UI_BTN_LONG_MS      600u   /* 按住这么久 = 长按 */
#define UI_BTN_DBL_MS       250u   /* 两次短按间隔小于这个 = 双击 */

/* 频率步进档，单位厘赫兹，编码器每格加减一档 */
static const uint64_t s_step_cHz[] = {
    100000000ULL, 10000000ULL, 1000000ULL, 100000ULL, 10000ULL,
    1000ULL, 100ULL, 10ULL, 1ULL
};
static const char *const s_step_txt[] = {
    "1MHz", "100kHz", "10kHz", "1kHz", "100Hz",
    "10Hz", "1Hz", "0.1Hz", "0.01Hz"
};
#define UI_STEP_COUNT   (sizeof(s_step_cHz) / sizeof(s_step_cHz[0]))
#define UI_STEP_DDS_TOP 3u                   /* 正弦/方波最细一档 = 1kHz */
#define UI_STEP_DEFAULT 3u                   /* 开机默认 1kHz 档 */

static const char *const s_ch_name[UI_CH_COUNT] = { "SINE", "SQR ", "SAW " };

/* ------------------------------------------------------------- 状态 */

static struct {
    uint64_t freq_cHz;
    uint16_t amp_mVpp;
} s_p[UI_CH_COUNT];

static ui_ch_t s_ch      = UI_CH_SINE;
static uint8_t s_step    = UI_STEP_DEFAULT;
static uint8_t s_out_on  = 1u;
static uint8_t s_saw_run = 0u;
static uint8_t s_dirty   = 1u;

/* 本波形能用到的最高档位下标（越小越粗）；正弦/方波最细到 1kHz */
static uint8_t ch_step_top(ui_ch_t c)
{
    return (c == UI_CH_SAW) ? (uint8_t)(UI_STEP_COUNT - 1u) : UI_STEP_DDS_TOP;
}

/* 切波形之后档位索引要收回到本通道的范围内 */
static void step_clamp(ui_ch_t c)
{
    if (s_step > ch_step_top(c)) { s_step = ch_step_top(c); }
}

/* 挂了调试器可以直接读这两个：屏数在涨 = 在刷，失败数一直是 0 = I2C 没丢 */
volatile uint32_t g_ui_frames = 0u;
volatile uint32_t g_ui_fails  = 0u;
volatile uint32_t g_loop      = 0u;

/* 幅度链调试探针：最近 16 次下发的幅度值、单次最大格数、
 * 最后一次写进两颗 MCP4725 的电压、I2C 写失败次数
 */
volatile uint16_t g_amp_hist[16] = {0};
volatile uint8_t  g_amp_hist_i   = 0u;
volatile uint16_t g_amp_dac_mV[3] = {0u, 0u, 0u};
volatile uint32_t g_amp_writes   = 0u;
volatile uint32_t g_amp_fails    = 0u;
volatile int16_t  g_amp_d_last   = 0;
volatile int16_t  g_amp_d_max    = 0;

/* 单颗 DAC 的成败统计：[0]=0x60 正弦 [1]=0x61 方波 */
volatile uint16_t g_dac_ok[2]   = {0u, 0u};
volatile uint16_t g_dac_fail[2] = {0u, 0u};
volatile uint8_t  g_dac_err[2]  = {0u, 0u};
volatile uint8_t  g_dac_present[2] = {1u, 1u};  /* 开机扫到的才有，{1,1} 是没扫到任何芯片时的兜底 */
volatile uint8_t  g_dac_retry      = 0u;        /* 上一轮有写失败，下一轮再写一次 */
volatile uint16_t g_dac_retries    = 0u;        /* 静默重试次数，一直涨说明那颗 DAC 有问题 */

/* 开机一次性自检的结果（只跑一次） */
volatile uint8_t  g_scan_n       = 0u;   /* 总线上应答的器件个数，正常 = 2 */
volatile uint8_t  g_scan_addr[8] = {0};
volatile uint16_t g_rb_sine      = 0u;   /* 0x60 读回的码值（写 3.500V -> 2867） */
volatile uint8_t  g_rb_st_sine   = 0u;
volatile uint8_t  g_rb_err_sine  = 0u;
volatile uint16_t g_rb_sqr       = 0u;   /* 0x61 读回的码值 */
volatile uint8_t  g_rb_st_sqr    = 0u;
volatile uint8_t  g_rb_err_sqr   = 0u;

/* ------------------------------------------------------- 幅度换算 */

/* 总开关关掉时三路一起静音 */
static uint16_t amp_live_mVpp(ui_ch_t c)
{
    return s_out_on ? s_p[c].amp_mVpp : 0u;
}

static uint16_t amp_max_mVpp(ui_ch_t c)
{
    if (c == UI_CH_SINE)   { return (uint16_t)UI_AMP_MAX_SINE_mVpp; }
    if (c == UI_CH_SQUARE) { return (uint16_t)UI_AMP_MAX_SQR_mVpp; }
    return (uint16_t)UI_AMP_MAX_SAW_mVpp;
}

static uint16_t amp_min_mVpp(ui_ch_t c)
{
    if (c == UI_CH_SINE)   { return (uint16_t)UI_AMP_MIN_SINE_mVpp; }
    if (c == UI_CH_SQUARE) { return (uint16_t)UI_AMP_MIN_SQR_mVpp; }
    return (uint16_t)UI_AMP_MIN_SAW_mVpp;
}

#if (AMP_MAP_DB != 0) || (AMP_SHOW_DB != 0)
/* log2(num/den)，返回 Q10 定点（1.0 -> 1024）；纯整数不引 libm，
 * 规整到 [1,2) 后走 32 段折线插值，误差 < 0.003 dB
 */
static const uint16_t s_log2_tab[33] = {
       0,   45,   90,  132,  174,  214,
     254,  292,  330,  366,  402,  436,
     470,  504,  536,  568,  599,  629,
     659,  689,  717,  745,  773,  800,
     827,  853,  879,  904,  929,  953,
     977, 1001, 1024,
};

static int32_t log2_q10(uint32_t num, uint32_t den)
{
    int32_t  e = 0;
    uint64_t x;
    uint32_t mant, k, frac;

    if ((num == 0u) || (den == 0u)) { return 0; }

    x = ((uint64_t)num << 24) / den;                 /* Q24 */
    while (x >= (1ULL << 25)) { x >>= 1; e++; }      /* 规整到 [2^24, 2^25) */
    while (x <  (1ULL << 24)) { x <<= 1; e--; }

    mant = (uint32_t)(x >> 8) - (1u << 16);          /* (m-1) 的 Q16 */
    k    = mant >> 11;                               /* 0 ~ 31 */
    frac = mant & 0x7FFu;                            /* 0 ~ 2047 */

    return (e << 10) + (int32_t)s_log2_tab[k] +
           (((int32_t)(s_log2_tab[k + 1u] - s_log2_tab[k]) * (int32_t)frac) >> 11);
}
#endif  /* AMP_MAP_DB or AMP_SHOW_DB */

/* 屏上 Vpp -> MCP4725 要输出的电压(mV)，直推 AD605 的 VGN */
#if (AMP_MAP_DB != 0)
static uint16_t adp605_vin_mVpp(ui_ch_t c)
{
    return (c == UI_CH_SQUARE) ? (uint16_t)ADP605_VIN_SQR_mVpp
                               : (uint16_t)ADP605_VIN_SINE_mVpp;
}
#endif

static uint16_t amp_dac_mV(ui_ch_t c)
{
    uint32_t vpp = amp_live_mVpp(c);

#if (AMP_MAP_DB != 0)
    int32_t  t;
    int32_t  n;
    int32_t  vgn;

    if (vpp == 0u) { return 0u; }

    t = log2_q10(vpp, adp605_vin_mVpp(c));      /* log2(Vpp/Vin)，Q10 */
    n = t * 1205;
    if (n >= 0) { vgn = 950 + ((n + 2048) >> 12); }
    else        { vgn = 950 - (((-n) + 2048) >> 12); }

    if (vgn < (int32_t)ADP605_VGN_MIN_mV) { vgn = (int32_t)ADP605_VGN_MIN_mV; }
    if (vgn > (int32_t)ADP605_VGN_MAX_mV) { vgn = (int32_t)ADP605_VGN_MAX_mV; }
    return (uint16_t)vgn;
#else
    /* 固定比例 DAC = Vpp*3000/3500：
     * 正弦 0~2000mVpp -> 0~1714mV，方波 100~1300mVpp -> 86~1114mV
     */
    if (vpp >= (uint32_t)MCP4725_DAC_REF_mVpp) { return (uint16_t)MCP4725_DAC_MAX_mV; }
    return (uint16_t)((vpp * (uint32_t)MCP4725_DAC_MAX_mV) / (uint32_t)MCP4725_DAC_REF_mVpp);
#endif
}

/* 锯齿走 MCP4921 数字幅度；满幅是 VREF=5V，屏上 3.50Vpp 只用 70% 码值 */
static uint16_t amp_permille(ui_ch_t c)
{
    return (uint16_t)(((uint32_t)amp_live_mVpp(c) * 1000u) / MCP4921_FULL_mV);
}

static uint64_t ch_freq_max_cHz(ui_ch_t c)
{
    return (c == UI_CH_SAW) ? UI_FREQ_MAX_SAW_cHz : UI_FREQ_MAX_DDS_cHz;
}

/* 正弦/方波最低 2kHz，锯齿允许低到 1Hz */
static uint64_t ch_freq_min_cHz(ui_ch_t c)
{
    return (c == UI_CH_SAW) ? UI_FREQ_MIN_SAW_cHz : UI_FREQ_MIN_cHz;
}

/* --------------------------------------------------------- 下发硬件 */

static uint32_t saw_freq_mHz(void)
{
    /* 1 cHz = 10 mHz */
    return (uint32_t)(s_p[UI_CH_SAW].freq_cHz * 10ULL);
}

/* 写两颗 MCP4725；返回 1 = 有探到的 DAC 没写成功（上层重试），没探到的跳过 */
static uint8_t dac_apply(void)
{
    uint16_t dac_sin = amp_dac_mV(UI_CH_SINE);
    uint16_t dac_sqr = amp_dac_mV(UI_CH_SQUARE);
    uint8_t  bad   = 0u;

    g_amp_dac_mV[UI_CH_SINE]   = dac_sin;
    g_amp_dac_mV[UI_CH_SQUARE] = dac_sqr;
    g_amp_hist[g_amp_hist_i & 0x0Fu] = (uint16_t)s_p[s_ch].amp_mVpp;
    g_amp_hist_i++;
    g_amp_writes++;

    if (g_dac_present[0] != 0u) {
        if (mcp4725_set_mv(MCP4725_ADDR_SINE, dac_sin) == false) {
            g_amp_fails++; g_dac_fail[0]++; g_dac_err[0] = g_mcp_err; bad = 1u;
        } else {
            g_dac_ok[0]++;
        }
    }
    if (g_dac_present[1] != 0u) {
        if (mcp4725_set_mv(MCP4725_ADDR_SQUARE, dac_sqr) == false) {
            g_amp_fails++; g_dac_fail[1]++; g_dac_err[1] = g_mcp_err; bad = 1u;
        } else {
            g_dac_ok[1]++;
        }
    }
    return bad;
}

static uint8_t hw_apply(void)
{
    uint32_t saw_mHz = saw_freq_mHz();
    uint16_t saw_pm  = amp_permille(UI_CH_SAW);
    uint8_t  bad;

    /* 正弦和方波共用一颗 AD9850，只能写一个频率：写当前通道自己的 */
    if (s_ch != UI_CH_SAW) {
        (void)ad9850_set_freq_cHz(s_p[s_ch].freq_cHz);
    }

    bad = dac_apply();

    mcp4921_ramp_set_freq(saw_mHz);
    mcp4921_ramp_set_amp(saw_pm);

    if (saw_pm > 0u) {
        if (s_saw_run == 0u) {
            (void)mcp4921_ramp_start(saw_mHz, saw_pm);
            s_saw_run = 1u;
        }
    } else if (s_saw_run != 0u) {
        mcp4921_ramp_stop();
        s_saw_run = 0u;
    }

    return bad;
}

/* --------------------------------------------------------- 输入处理 */

static void freq_step(int16_t d)
{
    int64_t  v  = (int64_t)s_p[s_ch].freq_cHz + (int64_t)d * (int64_t)s_step_cHz[s_step];
    uint64_t hi = ch_freq_max_cHz(s_ch);
    uint64_t lo = ch_freq_min_cHz(s_ch);

    if (v < (int64_t)lo) { v = (int64_t)lo; }
    if (v > (int64_t)hi) { v = (int64_t)hi; }

    if ((uint64_t)v != s_p[s_ch].freq_cHz) {
        s_p[s_ch].freq_cHz = (uint64_t)v;
        s_dirty = 1u;
    }
}

static void amp_step(int16_t d)
{
    int32_t lo = (int32_t)amp_min_mVpp(s_ch);
    int32_t hi = (int32_t)amp_max_mVpp(s_ch);
    int32_t v  = (int32_t)s_p[s_ch].amp_mVpp + ((int32_t)d * (int32_t)UI_AMP_STEP_mVpp);

    if (v < lo)                       { v = lo; }
    if (v > hi)                       { v = hi; }

    /* 正弦/方波写 0 就是 VGN 到地、AD605 掉电；锯齿写 0 也是真 0 */

    if ((uint16_t)v != s_p[s_ch].amp_mVpp) {
        s_p[s_ch].amp_mVpp = (uint16_t)v;
        s_dirty = 1u;
    }
}

/* ------------------------------------------------------------ 毫秒时基 */

/* ui_task 一轮不一定是 1ms，按键计时用 DWT 数真实毫秒 */
static uint32_t s_ms = 0u;              /* 开机至今的毫秒数 */
volatile uint32_t g_ui_ms = 0u;         /* 挂调试器看这个，确认时基在走 */

static void time_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

static void time_tick(void)
{
    static uint32_t last = 0u;
    static uint32_t rem  = 0u;
    const  uint32_t per  = SystemCoreClock / 1000u;   /* 72MHz -> 72000 */
    uint32_t        cyc  = DWT->CYCCNT;
    uint32_t        d    = cyc - last;                /* 无符号减法自动跨回绕 */
    uint32_t        ms;

    last = cyc;
    rem += d;
    ms   = rem / per;
    rem -= ms * per;

    s_ms   += ms;
    g_ui_ms = s_ms;
}

/* 全机只有 PA2 一个键：短按换步进档，双击总输出 ON/OFF，长按切波形。
 * 短按要等第二拍，会晚 UI_BTN_DBL_MS 生效；长按够时间立刻切，不等松手。
 */
static void btn_task(void)
{
    static uint8_t  raw_last   = 0u;
    static uint32_t t_raw      = 0u;    /* raw 上次变化的时刻 */
    static uint8_t  level      = 0u;    /* 消抖后的电平，1 = 按下 */
    static uint32_t t_press    = 0u;
    static uint32_t t_rel      = 0u;
    static uint8_t  long_fired = 0u;
    static uint8_t  waiting    = 0u;    /* 已收到一次短按，等第二拍 */

    uint8_t raw = button_down() ? 1u : 0u;

    if (raw != raw_last) {
        raw_last = raw;
        t_raw    = s_ms;
    }

    if ((raw_last != level) && ((s_ms - t_raw) >= UI_BTN_DB_MS)) {
        level = raw_last;

        if (level != 0u) {                  /* ---- 按下 ---- */
            t_press    = s_ms;
            long_fired = 0u;
        } else if (long_fired == 0u) {      /* ---- 抬起 ---- */
            if (waiting != 0u) {            /* 第二拍 = 双击 */
                waiting  = 0u;
                s_out_on ^= 1u;
                s_dirty   = 1u;
            } else {
                waiting = 1u;               /* 第一拍，先等第二拍 */
                t_rel   = s_ms;
            }
        }
    }

    /* 长按：够时间立刻切波形，不等松手 */
    if ((level != 0u) && (long_fired == 0u) && ((s_ms - t_press) >= UI_BTN_LONG_MS)) {
        long_fired = 1u;
        waiting    = 0u;                    /* 长按把前面的等待吃掉 */
        s_ch    = (ui_ch_t)(((uint8_t)s_ch + 1u) % (uint8_t)UI_CH_COUNT);
        step_clamp(s_ch);                   /* 从锯齿的细档切回来时收档 */
        s_dirty = 1u;
    }

    /* 等不到第二拍，算一次短按 -> 换步进档 */
    if ((waiting != 0u) && ((s_ms - t_rel) >= UI_BTN_DBL_MS)) {
        waiting = 0u;
        s_step  = (uint8_t)((s_step + 1u) % (ch_step_top(s_ch) + 1u));
        s_dirty = 1u;
    }
}

/* ------------------------------------------------------------- 显示 */

#if (AMP_SHOW_DB != 0)

/* 屏显 dB（AMP_SHOW_DB=1 时才用）：0 dB = AMP_DB_REF_mVpp，单位 0.1dB；
 * 0Vpp（静音）返回 -32000
 */
static int16_t amp_db_x10(uint16_t mVpp)
{
    int32_t t;

    if (mVpp == 0u) { return -32000; }
    if (mVpp > UI_AMP_MAX_SAW_mVpp) { mVpp = UI_AMP_MAX_SAW_mVpp; }

    t = log2_q10(mVpp, AMP_DB_REF_mVpp);        /* log2(Vpp/ref)，Q10 */
    /* 0.1dB 每 Q10 步 = 200*log10(2)/1024，用 241/4096 近似；正负都加半个 LSB */
    return (int16_t)((t * 241 + 2048) >> 12);
}

/* 写成 "+0.0dB" / "-34.0dB"（最多 7 个字符）；0Vpp 写 "-INF" */
static void amp_disp_str(ui_ch_t c, char *out)
{
    int16_t  db = amp_db_x10(s_p[c].amp_mVpp);
    uint16_t a;
    uint8_t  n = 0u;

    if (db <= -32000) {
        out[0] = '-'; out[1] = 'I'; out[2] = 'N'; out[3] = 'F'; out[4] = '\0';
        return;
    }

    a = (uint16_t)((db < 0) ? -db : db);
    out[n++] = (db < 0) ? '-' : '+';
    if (a >= 100u) { out[n++] = (char)('0' + ((a / 100u) % 10u)); }
    out[n++] = (char)('0' + ((a / 10u) % 10u));
    out[n++] = '.';
    out[n++] = (char)('0' + (a % 10u));
    out[n++] = 'd';
    out[n++] = 'B';
    out[n]   = '\0';
}

#else   /* AMP_SHOW_DB == 0：屏上显示实测的真实输出 Vpp */

/* 实测标定表（横轴 = 幅度设定值 mVpp，纵轴 = 真实输出 mVpp）：
 *   正弦 100->230  562->380  891->720  1000->880  1767~2000 钉在 3500（已到顶）
 *   方波 100->1000 320->1200 638->2000 1300->3800
 * 中间线性插值
 */
static const uint16_t s_show_sin_x[7] = {    0u,  100u,  562u,  891u, 1000u, 1767u, 2000u };
static const uint16_t s_show_sin_y[7] = {    0u,  230u,  380u,  720u,  880u, 3500u, 3500u };
static const uint16_t s_show_sqr_x[4] = {  100u,  320u,  638u, 1300u };
static const uint16_t s_show_sqr_y[4] = { 1000u, 1200u, 2000u, 3800u };

static uint16_t tbl_interp(const uint16_t *x, const uint16_t *y, uint8_t n, uint16_t v)
{
    for (uint8_t i = 1u; i < n; i++) {
        if (v <= x[i]) {
            uint32_t span = (uint32_t)(x[i] - x[i - 1u]);
            uint32_t d    = (uint32_t)(v - x[i - 1u]);
            return (uint16_t)((uint32_t)y[i - 1u] +
                              (((uint32_t)(y[i] - y[i - 1u]) * d) / span));
        }
    }
    return y[n - 1u];
}

/* 屏上要显示的幅度(mVpp)：正弦/方波查实测表，锯齿的设定值本来就是真实幅度 */
static uint16_t amp_show_mVpp(ui_ch_t c)
{
    if (c == UI_CH_SINE)   { return tbl_interp(s_show_sin_x, s_show_sin_y, 7u, s_p[c].amp_mVpp); }
    if (c == UI_CH_SQUARE) { return tbl_interp(s_show_sqr_x, s_show_sqr_y, 4u, s_p[c].amp_mVpp); }
    return s_p[c].amp_mVpp;
}

static void amp_disp_str(ui_ch_t c, char *out)
{
    uint16_t mVpp = amp_show_mVpp(c);

    out[0] = (char)('0' + (mVpp / 1000u));
    out[1] = '.';
    out[2] = (char)('0' + ((mVpp / 100u) % 10u));
    out[3] = (char)('0' + ((mVpp / 10u) % 10u));
    out[4] = 'V';
    out[5] = 'p';
    out[6] = 'p';
    out[7] = '\0';
}

#endif  /* AMP_SHOW_DB */

/* 步进档在小数第几位：0=个位 -1=十分位 +1=十位；不在屏上返回 99 */
static int step_exponent(const ssd1306_freq_t *f)
{
    uint64_t v = s_step_cHz[s_step];
    int      e = 0;

    if (v == f->unit_cHz) { return 0; }

    if (v < f->unit_cHz) {
        while ((v < f->unit_cHz) && (e > -18)) { v *= 10ULL; e--; }
    } else {
        while ((v > f->unit_cHz) && (e < 18)) { v /= 10ULL; e++; }
    }
    return (v == f->unit_cHz) ? e : 99;
}

static void ui_render(void)
{
    ssd1306_freq_t f;
    char           abuf[8];
    int            e;
    int            idx;
    int            n;

    ssd1306_clear();

    /* 第 0 行：波形名 | 幅度 | 输出开关 */
    ssd1306_str(0, 0, s_ch_name[s_ch], 1);
    amp_disp_str(s_ch, abuf);
    ssd1306_str(40, 0, abuf, 1);
    ssd1306_str(104, 0, (s_out_on != 0u) ? "ON " : "OFF", 1);

    /* 第 1~2 行：频率大字（最多 6 字符 = 96 像素）+ 小字单位 */
    /* 锯齿固定用 kHz 显示，正弦/方波还是 MHz/kHz/Hz 自动切 */
    if (s_ch == UI_CH_SAW) { ssd1306_freq_format_khz(s_p[s_ch].freq_cHz, &f); }
    else                   { ssd1306_freq_format(s_p[s_ch].freq_cHz, &f); }
    ssd1306_str(0, 16, f.text, 2);

    n = 0;
    while ((f.text[n] != '\0') && (n < 8)) { n++; }
    ssd1306_str((int16_t)(n * 16), 24, f.unit, 1);

    /* 编码器在调哪一位，就把那一位反白 */
    e   = step_exponent(&f);
    idx = (e == 99) ? -1 : ssd1306_freq_digit_pos(&f, e);
    if (idx >= 0) {
        ssd1306_char_inv((int16_t)(idx * 16), 16, f.text[idx], 2);
    }

    /* 第 3 行：当前调频步进档 */
    ssd1306_str(0,  48, "STEP", 1);
    ssd1306_str(40, 48, s_step_txt[s_step], 1);

    if (ssd1306_update()) {
        g_ui_frames++;
    } else {
        g_ui_fails = ssd1306_fail_count();
    }
}

/* --------------------------------------------------------- 对外接口 */

void ui_init(void)
{
    ad9850_init();
    mcp4725_init();
    mcp4725_bus_recover();   /* 上电先把 I2C2 救一遍，免得从机停在半截字节上 */
    mcp4921_init();
    encoder_init();
    buttons_init();
    time_init();

    s_ch      = UI_CH_SINE;
    s_step    = UI_STEP_DEFAULT;
    s_out_on  = 0u;                     /* 开机三路全关，双击 PA2 才打开 */
    s_saw_run = 0u;

    s_p[UI_CH_SINE].freq_cHz   = 100000000ULL;   /* 1.000000 MHz */
    s_p[UI_CH_SINE].amp_mVpp   = AMP_DB_REF_mVpp;   /* 0 dB   = 1.00 Vpp（1:1） */
    s_p[UI_CH_SQUARE].freq_cHz = 100000000ULL;
    s_p[UI_CH_SQUARE].amp_mVpp = UI_AMP_MIN_SQR_mVpp;   /* 最低 = 0.10 Vpp（-20.0 dB） */
    s_p[UI_CH_SAW].freq_cHz    = 10000ULL;       /* 100.00 Hz */
    s_p[UI_CH_SAW].amp_mVpp    = UI_AMP_MAX_SAW_mVpp;   /* 3.50 Vpp */

    (void)ssd1306_init();

    hw_apply();

    /* 开机第一屏可能赶上 I2C 抖动，重刷几次直到整帧送出去 */
    for (uint8_t t = 0u; (t < 3u) && (g_ui_frames == 0u); t++) {
        ui_render();
    }
    s_dirty = 0u;                       /* 开机这屏已经刷过，别让 ui_task 再补一遍 */

    /* 开机一次性 I2C2 自检：看芯片在不在、地址对不对；之后救回总线再重写幅度 */
    g_scan_n = mcp4725_scan(g_scan_addr, 8u);
    /* 探到哪颗写哪颗；一颗都没探到就退回两颗都写的兜底 */
    g_dac_present[0] = (g_scan_n == 0u) ? 1u : 0u;
    g_dac_present[1] = (g_scan_n == 0u) ? 1u : 0u;
    for (uint8_t i = 0u; i < g_scan_n; i++) {
        if (g_scan_addr[i] == MCP4725_ADDR_SINE)   { g_dac_present[0] = 1u; }
        if (g_scan_addr[i] == MCP4725_ADDR_SQUARE) { g_dac_present[1] = 1u; }
    }
    {
        uint16_t c = 0u;
        uint8_t  st = 0u;
        uint8_t  er = 0u;

        (void)mcp4725_read_code_ex(MCP4725_ADDR_SINE, &c, &st, &er);
        g_rb_sine = c; g_rb_st_sine = st; g_rb_err_sine = er;

        c = 0u; st = 0u; er = 0u;
        (void)mcp4725_read_code_ex(MCP4725_ADDR_SQUARE, &c, &st, &er);
        g_rb_sqr = c; g_rb_st_sqr = st; g_rb_err_sqr = er;
    }
    mcp4725_bus_recover();
    hw_apply();

    /* 把开机那段（mute + 自检 + 重写）的计数清掉，方便看用户操作之后的数 */
    g_amp_writes = 0u; g_amp_fails = 0u; g_amp_hist_i = 0u;
    g_amp_d_last = 0;  g_amp_d_max = 0;
    g_dac_ok[0] = 0u; g_dac_ok[1] = 0u;
    g_dac_fail[0] = 0u; g_dac_fail[1] = 0u;
}

void ui_task(void)
{
    int16_t d;

    g_loop++;
    time_tick();

    /* 屏当时没插上/没上电也不怕：探到就整屏重画一次 */
    if (!ssd1306_present() && ssd1306_poll(s_ms)) {
        s_dirty = 1u;
    }

    d = encoder_delta(ENC_FREQ);
    if (d != 0) { freq_step(d); }

    d = encoder_delta(ENC_AMP);
    if (d != 0) {
        g_amp_d_last = d;
        if (d > g_amp_d_max)       { g_amp_d_max = d; }
        if (-(int32_t)d > (int32_t)g_amp_d_max) { g_amp_d_max = (int16_t)(-(int32_t)d); }
        amp_step(d);
    }

    btn_task();

    if (s_dirty != 0u) {
        s_dirty = 0u;
        if (hw_apply() != 0u) { g_dac_retry = 1u; }   /* DAC 没写进去，下一轮补写 */
        ui_render();
    } else if (g_dac_retry != 0u) {
        /* 只补写 DAC，不刷屏；一直失败就一直补 */
        g_dac_retry = 0u;
        g_dac_retries++;
        if (dac_apply() != 0u) { g_dac_retry = 1u; }
    }

    /* 空转一下别把总线占满；编码器走硬件计数，不吃 CPU */
    for (volatile uint32_t i = 0u; i < 2000u; i++) { }
}

ui_ch_t  ui_current(void)         { return s_ch; }
uint64_t ui_freq_cHz(ui_ch_t ch)  { return s_p[ch].freq_cHz; }
uint16_t ui_amp_mVpp(ui_ch_t ch)  { return s_p[ch].amp_mVpp; }
uint8_t  ui_output_on(void)       { return s_out_on; }
