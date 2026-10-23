#ifndef __MCP4725_H
#define __MCP4725_H

#include "stm32f1xx_ll_i2c.h"
#include <stdint.h>
#include <stdbool.h>

/* MCP4725 幅度控制，I2C2 (PB10=SCL PB11=SDA)
 * 0x60(A0=GND) -> 正弦那路 AD605 的 VGN
 * 0x61(A0=VDD) -> 方波那路 AD605 的 VGN
 * 没有外部 VREF，基准就是 VDD，输出 0 ~ VDD；换供电改 MCP4725_VDD_mV
 */

#define MCP4725_ADDR_SINE       0x60u
#define MCP4725_ADDR_SQUARE     0x61u

#define MCP4725_FULL_SCALE      4095u
#define MCP4725_VDD_mV          5000u   /* 按 5.000V 算码值 */
#define MCP4725_1V_CODE         ((uint16_t)((1000UL * MCP4725_FULL_SCALE) / MCP4725_VDD_mV))

void mcp4725_init(void);

/* 调试用（定义在 mcp4725.c）：最近一次写入断在哪一步 + 各断点累计次数 */
extern volatile uint8_t  g_mcp_err;
extern volatile uint16_t g_mcp_err_cnt[6];
extern volatile uint8_t  g_mcp_last_addr;
bool mcp4725_set_code(uint8_t addr7, uint16_t code);
bool mcp4725_set_level(uint8_t addr7, uint16_t permille);
/* 按 mV 直接设定输出（0 ~ MCP4725_VDD_mV），标定时用它最直观 */
bool mcp4725_set_mv(uint8_t addr7, uint16_t mv);
bool mcp4725_mute(uint8_t addr7);
bool mcp4725_write3(uint8_t addr7, uint8_t b1, uint8_t b2, uint8_t b3);
/* 读回 DAC 寄存器。status：bit7=RDY bit6=POR bit5=PD1 bit4=PD0（11=掉电）
 * code / status 传 0 表示不关心 */
bool mcp4725_read_code(uint8_t addr7, uint16_t *code, uint8_t *status);

/* 带错误码：0 成功 1 START 超时 2 写地址没 ACK 3 重复 START 超时
 * 4 读地址没 ACK 5/6 读数据超时 9 总线 BUSY 卡死 */
bool mcp4725_read_code_ex(uint8_t addr7, uint16_t *code, uint8_t *status, uint8_t *err);
/* 总线是否卡死（BUSY 一直不落）。卡死时后面所有 START 都不会真的发出去 */
bool mcp4725_bus_wedged(void);

/* 排查用：SWRST 复位 I2C2 + 引脚敲 9 个时钟救总线 + 重新配置 */
void    mcp4725_bus_recover(void);
/* 扫 0x08~0x77，返回应答的个数，地址写进 out[]（最多 max 个） */
uint8_t mcp4725_scan(uint8_t *out, uint8_t max);

#endif
