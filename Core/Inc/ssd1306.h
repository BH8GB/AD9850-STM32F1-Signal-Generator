#ifndef __SSD1306_H
#define __SSD1306_H

#include "stm32f1xx_ll_i2c.h"
#include "oledfont.h"
#include <stdint.h>
#include <stdbool.h>

/* SSD1306 128x64 OLED，I2C1：PB6=SCL PB7=SDA
 * 7 位地址 0x3C/0x3D 上电自动探测；只写不读，整屏 1024 字节分 8 页推
 */

#define SSD1306_W       128
#define SSD1306_H       64
#define SSD1306_PAGES   (SSD1306_H / 8)

/* 屏地址（探测结果），0 = 屏没应答 */
bool    ssd1306_init(void);
bool    ssd1306_present(void);
uint8_t ssd1306_addr(void);

/* 运行中重试探测（0.5s 最多一次）；返回 true = 刚探到，需要重画整屏 */
bool    ssd1306_poll(uint32_t now_ms);

/* 显存 -> 屏，100kHz 下约 92ms，只在内容变化后调用 */
bool    ssd1306_update(void);
/* 累计 I2C 写失败次数，挂调试器读；一直是 0 说明屏一直收得到 */
uint32_t ssd1306_fail_count(void);

void    ssd1306_clear(void);
void    ssd1306_pixel(int16_t x, int16_t y, bool on);
void    ssd1306_hline(int16_t x, int16_t y, int16_t w, bool on);
void    ssd1306_fill(int16_t x, int16_t y, int16_t w, int16_t h, bool on);
void    ssd1306_rect(int16_t x, int16_t y, int16_t w, int16_t h, bool on);

/* scale = 1 -> 8x16 字符，scale = 2 -> 16x32 */
void    ssd1306_char(int16_t x, int16_t y, char c, uint8_t scale);
/* 反白：整格填满再把笔画抠掉，用来标出“当前在调哪一位” */
void    ssd1306_char_inv(int16_t x, int16_t y, char c, uint8_t scale);
void    ssd1306_str(int16_t x, int16_t y, const char *s, uint8_t scale);

/* 频率格式化结果：文本 = 整数位 + '.' + 小数位，最多 6 个字符 */
typedef struct {
    char        text[16];
    uint8_t     int_digits;   /* 整数位数 */
    uint8_t     decimals;     /* 小数位数 */
    uint64_t    unit_cHz;     /* 单位对应的厘赫兹：Hz=100 kHz=1e5 MHz=1e8 */
    const char *unit;         /* "Hz" / "kHz" / "MHz" */
} ssd1306_freq_t;

void ssd1306_freq_format(uint64_t f_cHz, ssd1306_freq_t *f);

/* 锯齿专用：固定按 kHz 显示，不自动切 Hz/MHz；100kHz 段自动少一位小数 */
void ssd1306_freq_format_khz(uint64_t f_cHz, ssd1306_freq_t *f);

/* 十进制指数 e 落在 text 的下标（0=个位，-1=十分位）；不在文本里返回 -1 */
int  ssd1306_freq_digit_pos(const ssd1306_freq_t *f, int e);

/* 只把文本拷进 out（至少 12 字节），返回单位字符串 */
const char *ssd1306_fmt_freq(uint64_t f_cHz, char *out, uint32_t out_size);

#endif /* __SSD1306_H */
