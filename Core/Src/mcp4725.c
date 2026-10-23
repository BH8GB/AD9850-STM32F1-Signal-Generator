#include "mcp4725.h"

#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_gpio.h"

#define I2C_TIMEOUT     100000u
#define CMD_WRITE_DAC   0x40u   /* 010 = 写 DAC 寄存器，PD=00 正常模式。0x60 会连 EEPROM 一起写 */

#define I2C2_PORT       GPIOB

/* 调试：最近一次 mcp4725_write3 断在哪一步 + 各断点累计次数
 * 1=BUSY 不落 2=START 超时 3=地址没 ACK 4=发数据失败 5=等 BTF 超时
 */
volatile uint8_t  g_mcp_err       = 0u;
volatile uint16_t g_mcp_err_cnt[6] = {0u, 0u, 0u, 0u, 0u, 0u};
volatile uint8_t  g_mcp_last_addr  = 0u;
#define I2C2_PIN_SCL    LL_GPIO_PIN_10   /* PB10 */
#define I2C2_PIN_SDA    LL_GPIO_PIN_11   /* PB11 */

static void i2c2_udelay(void)
{
    for (volatile uint32_t n = 0u; n < 72u; n++) { }   /* 100kHz 量级的半周期 */
}

static void i2c2_pins_to_gpio(void)
{
    LL_GPIO_InitTypeDef gi = {0};

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
    gi.Pin        = I2C2_PIN_SCL | I2C2_PIN_SDA;
    gi.Mode       = LL_GPIO_MODE_OUTPUT;
    gi.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
    gi.Speed      = LL_GPIO_SPEED_FREQ_HIGH;
    LL_GPIO_Init(I2C2_PORT, &gi);
    LL_GPIO_SetOutputPin(I2C2_PORT, I2C2_PIN_SCL | I2C2_PIN_SDA);
}

static void i2c2_pins_to_af(void)
{
    LL_GPIO_InitTypeDef gi = {0};

    gi.Pin        = I2C2_PIN_SCL | I2C2_PIN_SDA;
    gi.Mode       = LL_GPIO_MODE_ALTERNATE;
    gi.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
    gi.Speed      = LL_GPIO_SPEED_FREQ_HIGH;
    LL_GPIO_Init(I2C2_PORT, &gi);
}

/* F1 只在 BUSY=0 时才认新 START；不等就会被吃掉，SB 永不置位 */
static bool i2c2_wait_idle(void)
{
    uint32_t t = I2C_TIMEOUT;

    while (LL_I2C_IsActiveFlag_BUSY(I2C2)) {
        if (--t == 0u) {
            return false;
        }
    }
    return true;
}

bool mcp4725_bus_wedged(void)
{
    return (LL_I2C_IsActiveFlag_BUSY(I2C2) != 0u);
}

/* 出错收尾：清 AF + 补 STOP + 等总线空，否则从机按住 SDA 不放 */
static void i2c2_abort(void)
{
    LL_I2C_ClearFlag_AF(I2C2);
    LL_I2C_GenerateStopCondition(I2C2);
    (void)i2c2_wait_idle();
}

/* 只发 START，等到 SB 置位 */
static uint8_t i2c2_start(void)
{
    uint32_t t = I2C_TIMEOUT;

    LL_I2C_GenerateStartCondition(I2C2);
    while (!LL_I2C_IsActiveFlag_SB(I2C2)) {
        if (--t == 0u) {
            return 1u;
        }
    }
    return 0u;
}

/* 发一个地址字节，等到 ADDR 置位（= 从机 ACK 了） */
static uint8_t i2c2_addr(uint8_t a8)
{
    uint32_t t = I2C_TIMEOUT;

    LL_I2C_TransmitData8(I2C2, a8);
    while (!LL_I2C_IsActiveFlag_ADDR(I2C2)) {
        if (LL_I2C_IsActiveFlag_AF(I2C2)) {
            i2c2_abort();
            return 2u;
        }
        if (--t == 0u) {
            i2c2_abort();
            return 2u;
        }
    }
    LL_I2C_ClearFlag_ADDR(I2C2);
    return 0u;
}

/* 救总线：SWRST 复位 I2C2 -> 引脚当 GPIO 敲 9 个 SCL -> 补 STOP -> 交回 I2C2 */
void mcp4725_bus_recover(void)
{
    LL_I2C_InitTypeDef ii = {0};

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C2);
    LL_I2C_Disable(I2C2);
    LL_I2C_EnableReset(I2C2);
    for (volatile uint32_t n = 0u; n < 720u; n++) { }
    LL_I2C_DisableReset(I2C2);

    i2c2_pins_to_gpio();

    for (uint32_t i = 0u; i < 9u; i++) {
        LL_GPIO_ResetOutputPin(I2C2_PORT, I2C2_PIN_SCL);
        i2c2_udelay();
        LL_GPIO_SetOutputPin(I2C2_PORT, I2C2_PIN_SCL);
        i2c2_udelay();
    }

    LL_GPIO_ResetOutputPin(I2C2_PORT, I2C2_PIN_SDA);   /* SCL 高时 SDA 由低变高 = STOP */
    i2c2_udelay();
    LL_GPIO_SetOutputPin(I2C2_PORT, I2C2_PIN_SCL);
    i2c2_udelay();
    LL_GPIO_SetOutputPin(I2C2_PORT, I2C2_PIN_SDA);
    i2c2_udelay();

    i2c2_pins_to_af();

    ii.PeripheralMode  = LL_I2C_MODE_I2C;
    ii.ClockSpeed      = 100000;
    ii.DutyCycle       = LL_I2C_DUTYCYCLE_2;
    ii.OwnAddress1     = 0;
    ii.TypeAcknowledge = LL_I2C_ACK;
    ii.OwnAddrSize     = LL_I2C_OWNADDRESS1_7BIT;
    LL_I2C_Init(I2C2, &ii);
    LL_I2C_Enable(I2C2);
}

/* 只发地址看有没有 ACK，不发数据 */
static bool i2c2_ping(uint8_t addr7)
{
    if (!i2c2_wait_idle()) { return false; }
    if (i2c2_start() != 0u) { return false; }
    if (i2c2_addr((uint8_t)(addr7 << 1u)) != 0u) { return false; }

    LL_I2C_GenerateStopCondition(I2C2);
    (void)i2c2_wait_idle();
    return true;
}

uint8_t mcp4725_scan(uint8_t *out, uint8_t max)
{
    uint8_t n = 0u;

    for (uint8_t a = 0x08u; a <= 0x77u; a++) {
        if (i2c2_ping(a)) {
            if ((out != 0) && (n < max)) { out[n] = a; }
            n++;
        }
    }
    return n;
}

bool mcp4725_write3(uint8_t addr7, uint8_t b1, uint8_t b2, uint8_t b3)
{
    uint32_t t;
    uint8_t  i;
    uint8_t  buf[3];

    buf[0] = b1; buf[1] = b2; buf[2] = b3;

    g_mcp_last_addr = addr7;
    g_mcp_err       = 0u;

    if (!i2c2_wait_idle()) {          /* 总线卡死：救一把，这次算失败，下次就好了 */
        mcp4725_bus_recover();
        g_mcp_err = 1u; g_mcp_err_cnt[1]++;
        return false;
    }
    if (i2c2_start() != 0u) {
        g_mcp_err = 2u; g_mcp_err_cnt[2]++;
        return false;
    }
    if (i2c2_addr((uint8_t)(addr7 << 1u)) != 0u) {
        g_mcp_err = 3u; g_mcp_err_cnt[3]++;      /* 从机没 ACK 地址 */
        return false;
    }

    for (i = 0u; i < 3u; i++) {
        t = I2C_TIMEOUT;
        while (!LL_I2C_IsActiveFlag_TXE(I2C2)) {
            if (LL_I2C_IsActiveFlag_AF(I2C2)) {
                i2c2_abort();
                g_mcp_err = 4u; g_mcp_err_cnt[4]++;
                return false;
            }
            if (--t == 0u) {
                i2c2_abort();
                g_mcp_err = 4u; g_mcp_err_cnt[4]++;
                return false;
            }
        }
        LL_I2C_TransmitData8(I2C2, buf[i]);
    }

    /* 等 BTF 再 STOP，否则最后一个字节会被截断 */
    t = I2C_TIMEOUT;
    while (!LL_I2C_IsActiveFlag_BTF(I2C2)) {
        if (--t == 0u) {
            i2c2_abort();
            g_mcp_err = 5u; g_mcp_err_cnt[5]++;
            return false;
        }
    }

    LL_I2C_GenerateStopCondition(I2C2);
    (void)i2c2_wait_idle();           /* 等 STOP 真发完，下一条命令才发得出去 */
    return true;
}

void mcp4725_init(void)
{
    /* CubeMX 的 MX_I2C2_Init() 没调 LL_I2C_Enable()，PE 一直是 0，
     * START 发不出去，这里补上使能
     */
    LL_I2C_Enable(I2C2);

    /* 上电先置 0，避免满幅冲击后级 */
    mcp4725_mute(MCP4725_ADDR_SINE);
    mcp4725_mute(MCP4725_ADDR_SQUARE);
}

bool mcp4725_set_code(uint8_t addr7, uint16_t code)
{
    if (code > MCP4725_FULL_SCALE) {
        code = MCP4725_FULL_SCALE;
    }
    return mcp4725_write3(addr7, CMD_WRITE_DAC,
                          (uint8_t)(code >> 4),
                          (uint8_t)((code & 0x0Fu) << 4));
}

bool mcp4725_set_level(uint8_t addr7, uint16_t permille)
{
    uint32_t code;

    if (permille > 1000u) {
        permille = 1000u;
    }
    code = ((uint32_t)permille * MCP4725_1V_CODE) / 1000u;
    return mcp4725_set_code(addr7, (uint16_t)code);
}

bool mcp4725_set_mv(uint8_t addr7, uint16_t mv)
{
    uint32_t code;

    if (mv > MCP4725_VDD_mV) {
        mv = MCP4725_VDD_mV;
    }
    code = (((uint32_t)mv * MCP4725_FULL_SCALE) + (MCP4725_VDD_mV / 2u)) / MCP4725_VDD_mV;
    return mcp4725_set_code(addr7, (uint16_t)code);
}

bool mcp4725_mute(uint8_t addr7)
{
    /* 写 0 = VGN 到地，AD605 整通道掉电，等于静音 */
    return mcp4725_set_code(addr7, 0u);
}

/* 读回 5 字节：状态/PD, DAC_MSB, DAC_LSB, EEPROM_MSB, EEPROM_LSB。
 * 必须先发一次写地址再重复 START 转读，不能直接发起读
 */
bool mcp4725_read_code_ex(uint8_t addr7, uint16_t *code, uint8_t *status, uint8_t *err)
{
    uint32_t t;
    uint8_t  b[5];
    uint8_t  i;
    uint8_t  rc = 0u;

    if (err != 0) { *err = 0u; }

    if (!i2c2_wait_idle()) {          /* 总线卡死：救一把再报错 */
        mcp4725_bus_recover();
        if (err != 0) { *err = 9u; }
        return false;
    }

    /* 读之前 ACK 必须是 1，否则第一个字节就被自己 NACK 掉了 */
    LL_I2C_AcknowledgeNextData(I2C2, LL_I2C_ACK);

    /* 1) START + 写地址 */
    if (i2c2_start() != 0u) { rc = 1u; goto fail; }
    if (i2c2_addr((uint8_t)(addr7 << 1u)) != 0u) { rc = 2u; goto fail; }

    /* 2) 重复 START + 读地址 */
    if (i2c2_start() != 0u) { rc = 3u; goto fail; }
    if (i2c2_addr((uint8_t)((addr7 << 1u) | 1u)) != 0u) { rc = 4u; goto fail; }

    /* 3) 前 4 个字节 ACK，最后一个 NACK（不然从机会一直往下送） */
    for (i = 0u; i < 4u; i++) {
        t = I2C_TIMEOUT;
        while (!LL_I2C_IsActiveFlag_RXNE(I2C2)) {
            if (--t == 0u) { rc = 5u; goto fail; }
        }
        b[i] = LL_I2C_ReceiveData8(I2C2);
    }

    LL_I2C_AcknowledgeNextData(I2C2, LL_I2C_NACK);

    t = I2C_TIMEOUT;
    while (!LL_I2C_IsActiveFlag_RXNE(I2C2)) {
        if (--t == 0u) { rc = 6u; goto fail; }
    }
    LL_I2C_GenerateStopCondition(I2C2);
    b[4] = LL_I2C_ReceiveData8(I2C2);

    (void)i2c2_wait_idle();
    LL_I2C_AcknowledgeNextData(I2C2, LL_I2C_ACK);   /* 还原，别影响后面的写 */

    if (code != 0) {
        *code = (uint16_t)(((uint16_t)b[1] << 4) | (uint16_t)(b[2] >> 4));
    }
    if (status != 0) {
        *status = b[0];
    }
    return true;

fail:
    i2c2_abort();
    LL_I2C_AcknowledgeNextData(I2C2, LL_I2C_ACK);
    if (err != 0) { *err = rc; }
    return false;
}

bool mcp4725_read_code(uint8_t addr7, uint16_t *code, uint8_t *status)
{
    return mcp4725_read_code_ex(addr7, code, status, 0);
}
