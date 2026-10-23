#ifndef __OLEDFONT_H
#define __OLEDFONT_H

#include <stdint.h>

/* 8x16 ASCII 字模，索引 = 字符 - 32
 * 每字符 16 字节 = 8 列 x 2 字节，列主序、MSB 在上，正好对上 SSD1306 的页
 */
#define FONT_ASC_FIRST   32
#define FONT_ASC_COUNT   95
#define FONT_W           8
#define FONT_H           16

extern const uint8_t asc2_1608[FONT_ASC_COUNT][16];

#endif /* __OLEDFONT_H */
