#include "ssd1306.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_gpio.h"

#define I2C_TIMEOUT     200000u

#define SSD1306_ADDR_A  0x3Cu   /* 模块默认 */
#define SSD1306_ADDR_B  0x3Du   /* 变体板 */

#define CTRL_CMD        0x00u   /* 控制字节：后面全是命令 */
#define CTRL_DATA       0x40u   /* 控制字节：后面全是显存数据 */

static uint8_t s_addr = 0u;                        /* 0 = 未探测到 */
static uint8_t s_buf[SSD1306_PAGES][SSD1306_W];    /* 1024 字节显存 */
static uint32_t s_fails = 0u;                      /* I2C 写失败累计次数 */
static bool s_need_recover = false;                /* 外设疑似卡死，下次通讯前先复位 */

/* ------------------------------------------------- PB6/PB7 总线救援工具 */

/* F1 的 I2C 一旦 BUSY 置位就不再产生 START，而且清不掉，只能 SWRST 复位。
 * 下面这套就是复位外设 + 把引脚当 GPIO 手动救总线。
 */
#define I2C_PORT      GPIOB
#define I2C_PIN_SCL   LL_GPIO_PIN_6
#define I2C_PIN_SDA   LL_GPIO_PIN_7

/* 72MHz 下一轮大约几微秒。I2C 没有最低时钟频率要求，慢一点没关系 */
static void i2c_udelay(void)
{
    for (volatile uint32_t n = 0u; n < 72u; n++) { }
}

/* 把两根线临时交给 GPIO（开漏，先释放成高） */
static void i2c_pins_to_gpio(void)
{
    LL_GPIO_InitTypeDef gi = {0};

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);

    gi.Pin        = I2C_PIN_SCL | I2C_PIN_SDA;
    gi.Mode       = LL_GPIO_MODE_OUTPUT;
    gi.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
    gi.Speed      = LL_GPIO_SPEED_FREQ_HIGH;
    LL_GPIO_Init(I2C_PORT, &gi);

    LL_GPIO_SetOutputPin(I2C_PORT, I2C_PIN_SCL | I2C_PIN_SDA);
}

/* 交回 I2C1 */
static void i2c_pins_to_af(void)
{
    LL_GPIO_InitTypeDef gi = {0};

    gi.Pin        = I2C_PIN_SCL | I2C_PIN_SDA;
    gi.Mode       = LL_GPIO_MODE_ALTERNATE;
    gi.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
    gi.Speed      = LL_GPIO_SPEED_FREQ_HIGH;
    LL_GPIO_Init(I2C_PORT, &gi);
}

/* 从机被打断在半截字节上会按住 SDA：敲 9 个 SCL 顶出去再补 STOP */
static void i2c_bus_free(void)
{
    uint32_t i;

    i2c_pins_to_gpio();

    for (i = 0u; i < 9u; i++) {
        LL_GPIO_ResetOutputPin(I2C_PORT, I2C_PIN_SCL);
        i2c_udelay();
        LL_GPIO_SetOutputPin(I2C_PORT, I2C_PIN_SCL);
        i2c_udelay();
    }

    LL_GPIO_ResetOutputPin(I2C_PORT, I2C_PIN_SDA);   /* SCL 高时 SDA 由低变高 = STOP */
    i2c_udelay();
    LL_GPIO_SetOutputPin(I2C_PORT, I2C_PIN_SCL);
    i2c_udelay();
    LL_GPIO_SetOutputPin(I2C_PORT, I2C_PIN_SDA);
    i2c_udelay();
}

/* SWRST 复位 I2C1 外设（BUSY 只有这样才能清掉） */
static void i2c_periph_reset(void)
{
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

    LL_I2C_Disable(I2C1);
    LL_I2C_EnableReset(I2C1);
    for (volatile uint32_t n = 0u; n < 720u; n++) { }
    LL_I2C_DisableReset(I2C1);
}

/* 完整恢复链：外设复位 -> 引脚当 GPIO 救总线 -> 交回 I2C1 -> 重新配置 */
static void i2c_recover(void)
{
    LL_I2C_InitTypeDef ii = {0};

    i2c_periph_reset();
    i2c_bus_free();
    i2c_pins_to_af();

    ii.PeripheralMode  = LL_I2C_MODE_I2C;
    ii.ClockSpeed      = 400000;
    ii.DutyCycle       = LL_I2C_DUTYCYCLE_2;
    ii.OwnAddress1     = 0;
    ii.TypeAcknowledge = LL_I2C_ACK;
    ii.OwnAddrSize     = LL_I2C_OWNADDRESS1_7BIT;
    LL_I2C_Init(I2C1, &ii);
    LL_I2C_Enable(I2C1);

    s_need_recover = false;
}

/* ------------------------------------------------------------- I2C 底座 */

/* 发 start + 从机地址，等 ACK。失败会自己补 stop */
static bool i2c_start(uint8_t addr7)
{
    uint32_t t;

    LL_I2C_GenerateStartCondition(I2C1);
    t = I2C_TIMEOUT;
    while (!LL_I2C_IsActiveFlag_SB(I2C1)) {
        if (--t == 0u) {
            /* START 都发不出去 = 外设 BUSY 卡死，只有 SWRST 能救 */
            s_need_recover = true;
            return false;
        }
    }

    LL_I2C_TransmitData8(I2C1, (uint8_t)(addr7 << 1));
    t = I2C_TIMEOUT;
    while (!LL_I2C_IsActiveFlag_ADDR(I2C1)) {
        if (LL_I2C_IsActiveFlag_AF(I2C1)) {
            LL_I2C_ClearFlag_AF(I2C1);
            LL_I2C_GenerateStopCondition(I2C1);
            return false;
        }
        if (--t == 0u) {
            LL_I2C_GenerateStopCondition(I2C1);
            return false;
        }
    }
    LL_I2C_ClearFlag_ADDR(I2C1);
    return true;
}

static bool i2c_write(uint8_t addr7, const uint8_t *buf, uint16_t len)
{
    uint32_t t;
    uint16_t i;

    if (!i2c_start(addr7)) { return false; }

    for (i = 0u; i < len; i++) {
        t = I2C_TIMEOUT;
        while (!LL_I2C_IsActiveFlag_TXE(I2C1)) {
            if (LL_I2C_IsActiveFlag_AF(I2C1)) {
                LL_I2C_ClearFlag_AF(I2C1);
                LL_I2C_GenerateStopCondition(I2C1);
                return false;
            }
            if (--t == 0u) {
                LL_I2C_GenerateStopCondition(I2C1);
                return false;
            }
        }
        LL_I2C_TransmitData8(I2C1, buf[i]);
    }

    /* 等 BTF 再 STOP，否则最后一个字节会被截断 */
    t = I2C_TIMEOUT;
    while (!LL_I2C_IsActiveFlag_BTF(I2C1)) {
        if (--t == 0u) {
            LL_I2C_GenerateStopCondition(I2C1);
            return false;
        }
    }

    LL_I2C_GenerateStopCondition(I2C1);
    return true;
}

/* 只发地址看有没有 ACK —— 用来探测屏在不在 */
static bool i2c_ping(uint8_t addr7)
{
    if (!i2c_start(addr7)) { return false; }
    LL_I2C_GenerateStopCondition(I2C1);
    return true;
}

static void ssd_cmd(uint8_t cmd)
{
    uint8_t b[2];

    b[0] = CTRL_CMD;
    b[1] = cmd;
    (void)i2c_write(s_addr, b, 2u);
}

/* ----------------------------------------------------------------- 对外 */

bool ssd1306_init(void)
{
    /* 参数紧跟命令，控制字节 0x00 之后每个字节都当命令处理 */
    static const uint8_t seq[] = {
        0xAEu,              /* 关显示 */
        0xD5u, 0x80u,       /* 时钟分频 / 振荡频率 */
        0xA8u, 0x3Fu,       /* 多路复用比 1/64 */
        0xD3u, 0x00u,       /* 显示偏移 0 */
        0x40u,              /* 起始行 0 */
        0x8Du, 0x14u,       /* 电荷泵开 */
        0x20u, 0x02u,       /* 页寻址模式 */
        0xA1u,              /* 段重映射：左右翻转 */
        0xC8u,              /* 扫描方向：上下翻转 */
        0xDAu, 0x12u,       /* COM 引脚配置 */
        0x81u, 0xCFu,       /* 对比度 */
        0xD9u, 0xF1u,       /* 预充电周期 */
        0xDBu, 0x40u,       /* VCOMH 电压 */
        0xA4u,              /* 显示跟随显存 */
        0xA6u,              /* 正常（非反白） */
        0xAFu               /* 开显示 */
    };
    uint32_t i;
    uint8_t  try;

    /* 先走一遍恢复链；整屏 1024 字节，400kHz 下约 25ms */
    i2c_recover();

    s_addr = 0u;
    for (try = 0u; (try < 3u) && (s_addr == 0u); try++) {
        if (i2c_ping(SSD1306_ADDR_A)) {
            s_addr = SSD1306_ADDR_A;
            break;
        }
        if (i2c_ping(SSD1306_ADDR_B)) {
            s_addr = SSD1306_ADDR_B;
            break;
        }
        /* 这一轮没人应答：外设可能又卡住了，整条恢复链重来一次 */
        for (volatile uint32_t d = 0u; d < 72000u; d++) { }
        i2c_recover();
    }

    if (s_addr == 0u) {
        return false;
    }

    for (i = 0u; i < sizeof(seq); i++) {
        ssd_cmd(seq[i]);
    }

    ssd1306_clear();
    return ssd1306_update();
}

bool ssd1306_present(void)
{
    return (s_addr != 0u);
}

/* 运行中重探（0.5s 最多一次）；成功返回 true，调用方应重画整屏 */
static uint32_t s_next_try_ms = 0u;

bool ssd1306_poll(uint32_t now_ms)
{
    if (s_addr != 0u) { return false; }
    if ((uint32_t)(now_ms - s_next_try_ms) < 500u) { return false; }

    s_next_try_ms = now_ms;
    return ssd1306_init();
}

uint32_t ssd1306_fail_count(void)
{
    return s_fails;
}

uint8_t ssd1306_addr(void)
{
    return s_addr;
}

bool ssd1306_update(void)
{
    uint8_t tx[SSD1306_W + 1u];
    uint8_t page;
    uint8_t i;
    bool    ok = true;

    if (s_need_recover) { i2c_recover(); }      /* 上一轮卡死过：先复位外设再发 */

    if (s_addr == 0u) { s_fails++; return false; }

    tx[0] = CTRL_DATA;

    for (page = 0u; page < SSD1306_PAGES; page++) {
        ssd_cmd((uint8_t)(0xB0u + page));   /* 页地址 */
        ssd_cmd(0x00u);                     /* 列地址低 4 位 */
        ssd_cmd(0x10u);                     /* 列地址高 4 位 */

        for (i = 0u; i < SSD1306_W; i++) {
            tx[i + 1u] = s_buf[page][i];
        }
        if (!i2c_write(s_addr, tx, (uint16_t)sizeof(tx))) { ok = false; }
    }
    if (!ok) { s_fails++; s_need_recover = true; }
    return ok;
}

void ssd1306_clear(void)
{
    uint8_t page;
    uint8_t x;

    for (page = 0u; page < SSD1306_PAGES; page++) {
        for (x = 0u; x < SSD1306_W; x++) {
            s_buf[page][x] = 0x00u;
        }
    }
}

/* --------------------------------------------------------------- 画图元 */

void ssd1306_pixel(int16_t x, int16_t y, bool on)
{
    if ((x < 0) || (x >= SSD1306_W) || (y < 0) || (y >= SSD1306_H)) { return; }

    if (on) {
        s_buf[y >> 3][x] |= (uint8_t)(1u << (y & 7));
    } else {
        s_buf[y >> 3][x] &= (uint8_t)~(1u << (y & 7));
    }
}

void ssd1306_hline(int16_t x, int16_t y, int16_t w, bool on)
{
    int16_t i;

    for (i = 0; i < w; i++) {
        ssd1306_pixel((int16_t)(x + i), y, on);
    }
}

void ssd1306_fill(int16_t x, int16_t y, int16_t w, int16_t h, bool on)
{
    int16_t j;

    for (j = 0; j < h; j++) {
        ssd1306_hline(x, (int16_t)(y + j), w, on);
    }
}

void ssd1306_rect(int16_t x, int16_t y, int16_t w, int16_t h, bool on)
{
    ssd1306_hline(x, y, w, on);
    ssd1306_hline(x, (int16_t)(y + h - 1), w, on);

    for (int16_t j = 0; j < h; j++) {
        ssd1306_pixel(x, (int16_t)(y + j), on);
        ssd1306_pixel((int16_t)(x + w - 1), (int16_t)(y + j), on);
    }
}

/* ----------------------------------------------------------------- 文字 */

/* 字模是列主序：每列 2 字节，字节内 MSB 是最上面一行 */
static void draw_glyph(int16_t x, int16_t y, const uint8_t *g, uint8_t scale, bool inv)
{
    uint8_t col;
    uint8_t row;

    if (inv) {
        /* 反白：先把整个字格点亮，下面的笔画再用 !inv 抠成暗的 */
        ssd1306_fill(x, y, (int16_t)(FONT_W * scale), (int16_t)(FONT_H * scale), true);
    }

    for (col = 0u; col < FONT_W; col++) {
        for (row = 0u; row < FONT_H; row++) {
            uint8_t bit = (uint8_t)((g[col * 2u + (row >> 3)] >> (7u - (row & 7u))) & 1u);

            if (bit == 0u) { continue; }

            if (scale <= 1u) {
                ssd1306_pixel((int16_t)(x + col), (int16_t)(y + row), !inv);
            } else {
                ssd1306_fill((int16_t)(x + col * scale),
                             (int16_t)(y + row * scale),
                             (int16_t)scale, (int16_t)scale, !inv);
            }
        }
    }
}

void ssd1306_char(int16_t x, int16_t y, char c, uint8_t scale)
{
    uint8_t idx;

    if ((c < ' ') || (c > '~')) { c = ' '; }
    if (scale == 0u) { scale = 1u; }

    idx = (uint8_t)(c - FONT_ASC_FIRST);
    draw_glyph(x, y, asc2_1608[idx], scale, false);
}

void ssd1306_char_inv(int16_t x, int16_t y, char c, uint8_t scale)
{
    uint8_t idx;

    if ((c < ' ') || (c > '~')) { c = ' '; }
    if (scale == 0u) { scale = 1u; }

    idx = (uint8_t)(c - FONT_ASC_FIRST);
    draw_glyph(x, y, asc2_1608[idx], scale, true);
}

void ssd1306_str(int16_t x, int16_t y, const char *s, uint8_t scale)
{
    if (scale == 0u) { scale = 1u; }

    while (*s != '\0') {
        ssd1306_char(x, y, *s, scale);
        x = (int16_t)(x + (int16_t)FONT_W * (int16_t)scale);
        s++;
    }
}

/* ------------------------------------------------------------ 频率格式化 */

static uint8_t u64_digits(uint64_t v)
{
    uint8_t n = 1u;

    while (v >= 10ull) { v /= 10ull; n++; }
    return n;
}

/* 锯齿专用：固定按 kHz 显示；整数位 1~2 位给 3 位小数，
 * 100~999kHz 让出 1 位小数，文本最多 6 字符
 */
void ssd1306_freq_format_khz(uint64_t f_cHz, ssd1306_freq_t *f)
{
    const uint64_t unit = 100000ULL;        /* 1 kHz */
    uint64_t scaled;
    uint64_t ip;
    uint64_t fp;
    uint32_t pos = 0u;
    uint8_t  n;
    char     tmp[10];

    f->unit_cHz   = unit;
    f->unit       = "kHz";
    f->int_digits = u64_digits((f_cHz + (unit / 2ull)) / unit);
    f->decimals   = (f->int_digits <= 2u) ? 3u : 2u;

    if (f->decimals == 3u) {
        scaled = ((f_cHz * 1000ull) + (unit / 2ull)) / unit;
        ip = scaled / 1000ull; fp = scaled % 1000ull;
    } else {
        scaled = ((f_cHz * 100ull) + (unit / 2ull)) / unit;
        ip = scaled / 100ull;  fp = scaled % 100ull;
    }

    if (ip == 0ull) {
        f->text[pos++] = '0';
    } else {
        n = 0u;
        while ((ip > 0ull) && (n < sizeof(tmp))) {
            tmp[n++] = (char)('0' + (uint8_t)(ip % 10ull));
            ip /= 10ull;
        }
        while (n > 0u) { f->text[pos++] = tmp[--n]; }
    }

    f->text[pos++] = '.';
    if (f->decimals == 3u) {
        f->text[pos++] = (char)('0' + (uint8_t)((fp / 100ull) % 10ull));
        f->text[pos++] = (char)('0' + (uint8_t)((fp / 10ull)  % 10ull));
        f->text[pos++] = (char)('0' + (uint8_t)(fp % 10ull));
    } else {
        f->text[pos++] = (char)('0' + (uint8_t)((fp / 10ull) % 10ull));
        f->text[pos++] = (char)('0' + (uint8_t)(fp % 10ull));
    }
    f->text[pos] = '\0';
}

void ssd1306_freq_format(uint64_t f_cHz, ssd1306_freq_t *f)
{
    uint64_t unit;
    uint64_t scaled;
    uint64_t ip;
    uint64_t fp;
    uint32_t pos = 0u;
    uint8_t  n;
    char     tmp[10];

    if (f_cHz >= 100000000ULL) {                    /* >= 1 MHz */
        unit = 100000000ULL;
    } else if (f_cHz >= 100000ULL) {                /* >= 1 kHz */
        unit = 100000ULL;
    } else {
        unit = 100ULL;
    }

    /* 固定 3 位小数；整数位超过 2 位就把单位往上挪一档，
     * 保证文本最多 6 字符（XX.YYY）
     */
    f->decimals = 3u;
    for (;;) {
        /* 用四舍五入后的整数部分定宽度，避开 999.9999 -> 1000.000 这种进位 */
        f->int_digits = u64_digits((f_cHz + (unit / 2ull)) / unit);
        if ((f->int_digits <= 2u) || (unit >= 100000000ULL)) { break; }
        unit *= 1000ull;
    }

    f->unit_cHz = unit;
    f->unit     = (unit == 100000000ULL) ? "MHz"
                : (unit == 100000ULL)     ? "kHz" : "Hz";

    scaled = ((f_cHz * 1000ull) + (unit / 2ull)) / unit;
    ip     = scaled / 1000ull;
    fp     = scaled % 1000ull;

    if (ip == 0ull) {
        f->text[pos++] = '0';
    } else {
        n = 0u;
        while ((ip > 0ull) && (n < sizeof(tmp))) {
            tmp[n++] = (char)('0' + (uint8_t)(ip % 10ull));
            ip /= 10ull;
        }
        while (n > 0u) { f->text[pos++] = tmp[--n]; }
    }

    f->text[pos++] = '.';
    f->text[pos++] = (char)('0' + (uint8_t)((fp / 100ull) % 10ull));
    f->text[pos++] = (char)('0' + (uint8_t)((fp / 10ull)  % 10ull));
    f->text[pos++] = (char)('0' + (uint8_t)(fp % 10ull));
    f->text[pos] = '\0';
}

int ssd1306_freq_digit_pos(const ssd1306_freq_t *f, int e)
{
    if (e >= 0) {
        if (e >= (int)f->int_digits) { return -1; }
        return (int)f->int_digits - 1 - e;
    }
    if (-e > (int)f->decimals) { return -1; }
    return (int)f->int_digits - e;
}

const char *ssd1306_fmt_freq(uint64_t f_cHz, char *out, uint32_t out_size)
{
    ssd1306_freq_t f;
    uint32_t       i = 0u;

    ssd1306_freq_format(f_cHz, &f);

    while ((f.text[i] != '\0') && ((i + 1u) < out_size)) {
        out[i] = f.text[i];
        i++;
    }
    out[i] = '\0';
    return f.unit;
}
