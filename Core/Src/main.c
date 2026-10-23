/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "ad9850.h"
#include "ssd1306.h"
#include "ui.h"
#include "mcp4725.h"
#include "encoder.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* AD9850 验证频率，单位 0.01Hz：1kHz=100000  1MHz=1e8  5MHz=5e8  40MHz=4e9 */
#define AD9850_TEST_FREQ_cHz   500000000ULL

/* 上电模式（硬件排查用，正常应用填 6）：
 *   0=固定频点  1=引脚走路  2=四线 1Hz 翻转  3=频率阶梯  4=全频段巡检
 *   5=OLED 自检页  6=完整应用  7=MCP4725 单颗验证  8=7 + 读回校验/总线自愈
 */
#define APP_BRINGUP_MODE      6

/* 模式 7：第二个编码器转一格的电压变化量 */
#define MCP4725_TEST_STEP_mV  50u

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* 上电自检结果：挂调试器直接读这两个变量 */
volatile uint8_t  g_oled_addr   = 0u;   /* 探测到的 OLED 7 位地址，0 = 没应答 */
volatile uint32_t g_oled_frames = 0u;   /* 已刷新的屏数，涨得快说明刷新正常 */

/* MCP4725 验证的结果，挂调试器直接读 */
volatile uint16_t g_dac_mv    = 0u;    /* 当前设定的输出，mV */
volatile uint32_t g_dac_fails = 0u;    /* I2C 写失败次数，转半天还是 0 就是正常 */
volatile uint8_t  g_dac_addr  = 0u;    /* 探测到的 MCP4725 7 位地址，0 = 没应答 */
volatile uint16_t g_dac_rb    = 0xFFFFu; /* 从 DAC 读回的码值，0xFFFF = 读失败 */
volatile uint8_t  g_dac_status = 0u;     /* DAC 状态字节：b7=RDY b6=POR b5=PD1 b4=PD0 */
volatile uint8_t  g_dac_rb_err = 0u;     /* 读回断在第几步，0 = 正常 */
volatile uint32_t g_dac_recoveries = 0u; /* 卡总线后自动救回来的次数 */

/* I2C2 扫描结果：count=0 就是一个器件都没应答 */
uint8_t           g_i2c2_found[8];     /* 扫到的 7 位地址 */
volatile uint8_t  g_i2c2_count = 0u;
volatile uint32_t g_i2c2_sr2   = 0u;   /* 扫描前的 I2C2->SR2，bit1=BUSY */
volatile uint32_t g_i2c2_scans = 0u;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* PendSV_IRQn interrupt configuration */
  NVIC_SetPriority(PendSV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));
  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

  /** NOJTAG: JTAG-DP Disabled and SW-DP Enabled
  */
  LL_GPIO_AF_Remap_SWJ_NOJTAG();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* AD9850：OUT1 正弦 + OUT3 方波，同频 */
  ad9850_init();                              /* 先输出 1kHz */

#if APP_BRINGUP_MODE == 1
  /* 走路自检：PA8→D7  PA9→CLK  PA10→FQ  PA11→RST */
  for (;;)
  {
    for (uint32_t i = 0u; i < 4u; i++)
    {
      GPIOA->BRR  = (1UL << 8) | (1UL << 9) | (1UL << 10) | (1UL << 11);
      GPIOA->BSRR = (1UL << (8u + i));
      for (volatile uint32_t d = 0u; d < 12000000u; d++) { }
    }
  }
#elif APP_BRINGUP_MODE == 2
  for (;;)
  {
    GPIOA->BSRR = (1UL << 8) | (1UL << 9) | (1UL << 10) | (1UL << 11);
    for (volatile uint32_t d = 0u; d < 6000000u; d++) { }
    GPIOA->BRR  = (1UL << 8) | (1UL << 9) | (1UL << 10) | (1UL << 11);
    for (volatile uint32_t d = 0u; d < 6000000u; d++) { }
  }
#elif APP_BRINGUP_MODE == 3
  /* 频率阶梯：只改频点，判断数据通路是否活着 */
  for (;;)
  {
    static const uint64_t ladder[] = {
      100000ULL,        /* 1 kHz   */
      1000000ULL,       /* 10 kHz  */
      10000000ULL,      /* 100 kHz */
      100000000ULL,     /* 1 MHz   */
      500000000ULL,     /* 5 MHz   */
      1000000000ULL,    /* 10 MHz  */
    };
    for (uint32_t k = 0u; k < (sizeof(ladder) / sizeof(ladder[0])); k++)
    {
      ad9850_set_freq_cHz(ladder[k]);
      for (volatile uint32_t d = 0u; d < 24000000u; d++) { }
    }
  }
#elif APP_BRINGUP_MODE == 4
  /* 全频段点频巡检，顺序见下面 scan[] 表 */
  for (;;)
  {
    static const uint64_t scan[] = {
      100ULL, 1000ULL, 10000ULL, 100000ULL,           /* 1Hz  10Hz  100Hz  1kHz  */
      1000000ULL, 10000000ULL, 50000000ULL,           /* 10kHz 100kHz 500kHz     */
      100000000ULL, 500000000ULL,                     /* 1MHz  5MHz              */
      1000000000ULL, 1500000000ULL, 2000000000ULL,    /* 10MHz 15MHz 20MHz       */
      2500000000ULL, 3000000000ULL, 3500000000ULL,    /* 25MHz 30MHz 35MHz       */
      4000000000ULL,                                  /* 40MHz                   */
    };

    for (uint32_t k = 0u; k < (sizeof(scan) / sizeof(scan[0])); k++)
    {
      uint32_t f_hz = (uint32_t)(scan[k] / 100ULL);
      uint32_t hold = 40000000u;                                  /* 5s 忙等 */
      uint32_t cyc  = (f_hz != 0u) ? (48000000u / f_hz) : 48000000u; /* 6 个周期 */

      if (cyc > hold) { hold = cyc; }

      ad9850_set_freq_cHz(scan[k]);
      for (volatile uint32_t d = 0u; d < hold; d++) { }
    }
  }
#elif APP_BRINGUP_MODE == 5
  /* OLED 自检页。屏没接也能跑，只是 g_oled_addr 会是 0 */
  {
    char        num[16];
    const char *unit;
    bool        oled_ok = ssd1306_init();

    g_oled_addr = ssd1306_addr();
    ad9850_set_freq_cHz(AD9850_TEST_FREQ_cHz);
    (void)oled_ok;

    for (;;)
    {
      char     tail[4];
      uint32_t c = g_oled_frames % 1000u;

      unit = ssd1306_fmt_freq(ad9850_get_freq_cHz(), num, sizeof(num));

      ssd1306_clear();

      /* 第 0 行：波形名 + 幅度 + 屏状态 */
      ssd1306_str(0,   0, "SINE", 1);
      ssd1306_str(40,  0, "3.50Vpp", 1);
      ssd1306_str(112, 0, ssd1306_present() ? "OK" : "NG", 1);

      /* 第 1~2 行：频率数值，16x32 大字，最多 8 个字符正好铺满 128 像素 */
      ssd1306_str(0, 16, num, 2);

      /* 第 3 行：单位 + 刷新计数（数字在动就说明屏在刷新） */
      tail[0] = (char)('0' + ((c / 100u) % 10u));
      tail[1] = (char)('0' + ((c / 10u) % 10u));
      tail[2] = (char)('0' + (c % 10u));
      tail[3] = '\0';

      ssd1306_str(0,   48, unit, 1);
      ssd1306_str(104, 48, tail, 1);

      (void)ssd1306_update();
      g_oled_frames++;

      for (volatile uint32_t d = 0u; d < 3000000u; d++) { }   /* 约 0.4s */
    }
  }
#elif APP_BRINGUP_MODE == 6
  /* 完整应用：全部交给 ui.c，裸超级循环，不启 FreeRTOS */
  ui_init();
  for (;;)
  {
    ui_task();
  }
#elif APP_BRINGUP_MODE == 7
  /* MCP4725 单颗验证：第二个编码器调 VOUT 0~5V
   * 接线 VDD→5V GND→GND SDA→PB11 SCL→PB10 VOUT→表/示波器(高阻)
   * SDA/SCL 上拉接 5V（PB10/PB11 是 5V 容忍脚；MCP4725 在 5V 下 VIH=3.5V）
   */
  {
    uint16_t mv = 2500u;
    char     num[8];
    char     code_s[12];

    mcp4725_init();
    encoder_init();
    buttons_init();
    (void)ssd1306_init();

    g_oled_addr = ssd1306_addr();

    /* 先救一遍 I2C2 再扫总线，否则 SDA 被按住时什么地址都扫不到 */
    g_i2c2_sr2   = I2C2->SR2;                 /* bit1 = BUSY */
    mcp4725_bus_recover();
    g_i2c2_count = mcp4725_scan(g_i2c2_found, 8u);
    g_i2c2_scans = 1u;

    /* 只挂一颗的时候 A0 接高接低都有可能，两个地址都试一遍 */
    if (mcp4725_set_mv(MCP4725_ADDR_SINE, mv)) {
      g_dac_addr = MCP4725_ADDR_SINE;
    } else if (mcp4725_set_mv(MCP4725_ADDR_SQUARE, mv)) {
      g_dac_addr = MCP4725_ADDR_SQUARE;
    } else {
      g_dac_addr = 0u;
      g_dac_fails++;
    }
    g_dac_mv = mv;

    for (;;)
    {
      int16_t d = encoder_delta(ENC_AMP);

      if (d != 0)
      {
        int32_t v = (int32_t)mv + ((int32_t)d * (int32_t)MCP4725_TEST_STEP_mV);

        if (v < 0)    { v = 0; }
        if (v > 5000) { v = 5000; }

        mv       = (uint16_t)v;
        g_dac_mv = mv;

        if (g_dac_addr != 0u) {
          if (!mcp4725_set_mv(g_dac_addr, mv)) { g_dac_fails++; }
        }
      }

      /* 大字显示 "2.500"，单位 V 用小字贴齐（大字没有 V 的字模） */
      num[0] = (char)('0' + (mv / 1000u));
      num[1] = '.';
      num[2] = (char)('0' + ((mv / 100u) % 10u));
      num[3] = (char)('0' + ((mv / 10u) % 10u));
      num[4] = (char)('0' + (mv % 10u));
      num[5] = '\0';

      {
        uint32_t code = (((uint32_t)mv * MCP4725_FULL_SCALE) + 2500u) / 5000u;

        code_s[0] = 'C'; code_s[1] = 'O'; code_s[2] = 'D'; code_s[3] = 'E';
        code_s[4] = (char)('0' + ((code / 1000u) % 10u));
        code_s[5] = (char)('0' + ((code / 100u) % 10u));
        code_s[6] = (char)('0' + ((code / 10u) % 10u));
        code_s[7] = (char)('0' + (code % 10u));
        code_s[8] = '\0';
      }

      ssd1306_clear();
      if (g_dac_addr != 0u) {
        char rb_s[16];
        uint16_t rb = 0u;
        uint8_t  st = 0u;
        static const char hexd[] = "0123456789ABCDEF";

        if (mcp4725_read_code(g_dac_addr, &rb, &st)) {
          g_dac_rb     = rb;
          g_dac_status = st;
        } else {
          g_dac_rb     = 0xFFFFu;
          g_dac_status = 0xFFu;
        }

        if (g_dac_rb == 0xFFFFu) {
          rb_s[0]='R'; rb_s[1]='B'; rb_s[2]=' '; rb_s[3]='N'; rb_s[4]='G';
          rb_s[5]=' '; rb_s[6]='S'; rb_s[7]='T'; rb_s[8]='F'; rb_s[9]='F';
          rb_s[10]='\0';
        } else {
          rb_s[0]='R'; rb_s[1]='B'; rb_s[2]=' ';
          rb_s[3]=(char)('0' + ((g_dac_rb / 1000u) % 10u));
          rb_s[4]=(char)('0' + ((g_dac_rb / 100u) % 10u));
          rb_s[5]=(char)('0' + ((g_dac_rb / 10u) % 10u));
          rb_s[6]=(char)('0' + (g_dac_rb % 10u));
          rb_s[7]=' '; rb_s[8]='S'; rb_s[9]='T';
          rb_s[10]=hexd[(g_dac_status >> 4) & 0x0Fu];
          rb_s[11]=hexd[g_dac_status & 0x0Fu];
          rb_s[12]='\0';
        }

        ssd1306_str(0,   0, "MCP4725", 1);
        ssd1306_str(96,  0, "OK", 1);
        ssd1306_str(0,  16, num, 2);
        ssd1306_str(80, 24, "V", 1);
        ssd1306_str(0,  48, code_s, 1);
        ssd1306_str(0,  56, rb_s, 1);
      } else {
        /* 没应答就把接线提示打到屏上，附带扫描结果和 BUSY */
        char sc[16];

        sc[0]  = 'S'; sc[1]  = 'C'; sc[2]  = 'A'; sc[3]  = 'N'; sc[4]  = ' ';
        sc[5]  = (char)('0' + (g_i2c2_count % 10u));
        sc[6]  = ' '; sc[7]  = 'B'; sc[8]  = 'U'; sc[9]  = 'S';
        sc[10] = 'Y'; sc[11] = (char)('0' + (uint8_t)((g_i2c2_sr2 >> 1) & 1u));
        sc[12] = ' '; sc[13] = 'N'; sc[14] = (char)('0' + (g_dac_fails % 10u));
        sc[15] = '\0';

        ssd1306_str(0,   0, "NO I2C DEV", 1);
        ssd1306_str(0,  16, "SDA PB11", 1);
        ssd1306_str(0,  24, "SCL PB10", 1);
        ssd1306_str(0,  32, "VDD 5V", 1);
        ssd1306_str(0,  40, "PULLUP 5V", 1);
        ssd1306_str(0,  48, "A0 GND/VCC", 1);
        ssd1306_str(0,  56, sc, 1);
      }

      (void)ssd1306_update();
      g_oled_frames++;

      /* 没找到器件就每隔一会儿重扫一次，边焊边看 */
      if ((g_dac_addr == 0u) && ((g_oled_frames % (uint32_t)400u) == 0u)) {
        mcp4725_bus_recover();
        g_i2c2_count = mcp4725_scan(g_i2c2_found, 8u);
        g_i2c2_sr2   = I2C2->SR2;
        g_i2c2_scans++;

        mv = 2500u;
        if (mcp4725_set_mv(MCP4725_ADDR_SINE, mv)) {
          g_dac_addr = MCP4725_ADDR_SINE;
        } else if (mcp4725_set_mv(MCP4725_ADDR_SQUARE, mv)) {
          g_dac_addr = MCP4725_ADDR_SQUARE;
        }
      }
    }
  }
#elif APP_BRINGUP_MODE == 8
  /* MCP4725 验证 v2：上电在 0V/2.5V 来回扫三趟，之后编码器一格 50mV 调，
   * PA2 重跑自检。屏上 CODE=写入码值 RB=读回码值 ST=状态字
   * E=读回错误步 R=救总线次数 SCN=扫到器件数 BUSY=总线忙位
   */
  {
    uint16_t mv      = 2500u;
    uint8_t  btn_old = 0u;
    char     num[8];
    char     l4[16];
    char     l5[16];
    char     l6[20];
    static const char hexd[] = "0123456789ABCDEF";

    mcp4725_init();
    encoder_init();
    buttons_init();
    (void)ssd1306_init();
    g_oled_addr = ssd1306_addr();

    g_i2c2_sr2   = I2C2->SR2;
    mcp4725_bus_recover();
    g_i2c2_count = mcp4725_scan(g_i2c2_found, 8u);
    g_i2c2_scans = 1u;

    /* A0 接地是 0x60，接 VDD 是 0x61，两个都试一遍 */
    if (mcp4725_set_mv(MCP4725_ADDR_SINE, mv)) {
      g_dac_addr = MCP4725_ADDR_SINE;
    } else if (mcp4725_set_mv(MCP4725_ADDR_SQUARE, mv)) {
      g_dac_addr = MCP4725_ADDR_SQUARE;
    } else {
      g_dac_addr = 0u;
    }

    /* 上电自检扫：0V 和 2.5V 走三趟，万用表/示波器上应该看得见跳 */
    if (g_dac_addr != 0u) {
      for (uint8_t k = 0u; k < 6u; k++) {
        (void)mcp4725_set_mv(g_dac_addr, ((k & 1u) != 0u) ? 2500u : 0u);
        for (volatile uint32_t n = 0u; n < 2000000u; n++) { }   /* 约 0.2s */
      }
      (void)mcp4725_set_mv(g_dac_addr, mv);
    }

    for (;;)
    {
      int16_t d   = encoder_delta(ENC_AMP);
      uint8_t btn = button_down() ? 1u : 0u;

      /* PA2：重跑来回扫，随时能确认 VOUT 到底会不会动 */
      if ((btn == 1u) && (btn_old == 0u) && (g_dac_addr != 0u)) {
        for (uint8_t k = 0u; k < 6u; k++) {
          (void)mcp4725_set_mv(g_dac_addr, ((k & 1u) != 0u) ? 2500u : 0u);
          for (volatile uint32_t n = 0u; n < 2000000u; n++) { }   /* 约 0.2s */
        }
        mv = 2500u;
        (void)mcp4725_set_mv(g_dac_addr, mv);
      }
      btn_old = btn;

      if (d != 0)
      {
        int32_t v = (int32_t)mv + ((int32_t)d * (int32_t)MCP4725_TEST_STEP_mV);

        if (v < 0)    { v = 0; }
        if (v > 5000) { v = 5000; }

        mv       = (uint16_t)v;
        g_dac_mv = mv;

        if (g_dac_addr != 0u) {
          if (mcp4725_bus_wedged()) {          /* 卡住了先救回来再写 */
            mcp4725_bus_recover();
            g_dac_recoveries++;
          }
          if (!mcp4725_set_mv(g_dac_addr, mv)) { g_dac_fails++; }
        }
      }
      g_dac_mv = mv;

      num[0] = (char)('0' + (mv / 1000u));
      num[1] = '.';
      num[2] = (char)('0' + ((mv / 100u) % 10u));
      num[3] = (char)('0' + ((mv / 10u) % 10u));
      num[4] = (char)('0' + (mv % 10u));
      num[5] = '\0';

      ssd1306_clear();

      if (g_dac_addr != 0u) {
        /* 读回不放在这里，交给下面每 25 帧一次的巡检 */
        uint32_t code = (((uint32_t)mv * MCP4725_FULL_SCALE) + (MCP4725_VDD_mV / 2u)) / MCP4725_VDD_mV;

        l4[0]='C'; l4[1]='O'; l4[2]='D'; l4[3]='E';
        l4[4]=(char)('0' + ((code / 1000u) % 10u));
        l4[5]=(char)('0' + ((code / 100u) % 10u));
        l4[6]=(char)('0' + ((code / 10u) % 10u));
        l4[7]=(char)('0' + (code % 10u));
        l4[8]=' '; l4[9]='F';
        l4[10]=(char)('0' + (g_dac_fails % 10u));
        l4[11]='\0';

        l5[0]='R'; l5[1]='B'; l5[2]=' ';
        if (g_dac_rb_err != 0u) {
          l5[3]='-'; l5[4]='-'; l5[5]='-'; l5[6]='-';
        } else {
          l5[3]=(char)('0' + ((g_dac_rb / 1000u) % 10u));
          l5[4]=(char)('0' + ((g_dac_rb / 100u) % 10u));
          l5[5]=(char)('0' + ((g_dac_rb / 10u) % 10u));
          l5[6]=(char)('0' + (g_dac_rb % 10u));
        }
        l5[7]=' '; l5[8]='S'; l5[9]='T';
        l5[10]=hexd[(g_dac_status >> 4) & 0x0Fu];
        l5[11]=hexd[g_dac_status & 0x0Fu];
        l5[12]='\0';

        l6[0]='A';
        l6[1]=hexd[(g_i2c2_found[0] >> 4) & 0x0Fu];
        l6[2]=hexd[g_i2c2_found[0] & 0x0Fu];
        l6[3]=' '; l6[4]='S'; l6[5]='C'; l6[6]='N';
        l6[7]=(char)('0' + (g_i2c2_count % 10u));
        l6[8]=' '; l6[9]='E'; l6[10]=(char)('0' + (g_dac_rb_err % 10u));
        l6[11]=' '; l6[12]='R'; l6[13]=(char)('0' + (g_dac_recoveries % 10u));
        l6[14]=' '; l6[15]='B';
        l6[16]=(char)('0' + (uint8_t)((I2C2->SR2 >> 1) & 1u));
        l6[17]='\0';

        ssd1306_str(0,   0, "MCP4725 V2", 1);
        ssd1306_str(96,  0, "OK", 1);
        ssd1306_str(0,  16, num, 2);
        ssd1306_str(80, 24, "V", 1);
        ssd1306_str(0,  40, l4, 1);
        ssd1306_str(0,  48, l5, 1);
        ssd1306_str(0,  56, l6, 1);
      } else {
        char sc[16];

        sc[0]='N'; sc[1]='O'; sc[2]=' '; sc[3]='I'; sc[4]='2'; sc[5]='C';
        sc[6]=' '; sc[7]='S'; sc[8]='C'; sc[9]='N';
        sc[10]=(char)('0' + (g_i2c2_count % 10u));
        sc[11]='\0';

        ssd1306_str(0,   0, sc, 1);
        ssd1306_str(0,  16, "SDA PB11", 1);
        ssd1306_str(0,  24, "SCL PB10", 1);
        ssd1306_str(0,  32, "VDD 5V", 1);
        ssd1306_str(0,  40, "PULLUP 5V", 1);
        ssd1306_str(0,  48, "A0 GND or VCC", 1);
        ssd1306_str(0,  56, "RETRY 4s", 1);
      }

      (void)ssd1306_update();
      g_oled_frames++;

      /* 每 25 帧（约 2.5s）巡检一次：总线卡了或上次没扫到就先救总线再扫，
       * 扫到谁用谁（A0 悬空时地址会在 0x60/0x61 之间飘），再重写并读回核对 */
      if ((g_oled_frames % (uint32_t)25u) == 0u) {
        if (mcp4725_bus_wedged() || (g_i2c2_count == 0u)) {
          mcp4725_bus_recover();
          g_dac_recoveries++;
        }
        g_i2c2_count = mcp4725_scan(g_i2c2_found, 8u);
        g_i2c2_sr2   = I2C2->SR2;
        g_i2c2_scans++;

        if (g_i2c2_count == 0u) {
          g_dac_addr = 0u;
        } else {
          uint16_t rb = 0u;
          uint8_t  st = 0u;
          uint8_t  er = 0u;

          g_dac_addr = g_i2c2_found[0];
          (void)mcp4725_set_mv(g_dac_addr, mv);
          if (mcp4725_read_code_ex(g_dac_addr, &rb, &st, &er)) {
            g_dac_rb     = rb;
            g_dac_status = st;
            g_dac_rb_err = 0u;
          } else {
            g_dac_rb     = 0xFFFFu;
            g_dac_status = 0u;
            g_dac_rb_err = er;
          }
        }
      }
    }
  }
#else
  ad9850_set_freq_cHz(AD9850_TEST_FREQ_cHz);  /* 验证频点 */
#endif

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(72000000);
  LL_SetSystemCoreClock(72000000);
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /**I2C1 GPIO Configuration
  PB6   ------> I2C1_SCL
  PB7   ------> I2C1_SDA
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */

  /** I2C Initialization
  */
  LL_I2C_DisableOwnAddress2(I2C1);
  LL_I2C_DisableGeneralCall(I2C1);
  LL_I2C_EnableClockStretching(I2C1);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.ClockSpeed = 100000;
  I2C_InitStruct.DutyCycle = LL_I2C_DUTYCYCLE_2;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C1, &I2C_InitStruct);
  LL_I2C_SetOwnAddress2(I2C1, 0);
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /**I2C2 GPIO Configuration
  PB10   ------> I2C2_SCL
  PB11   ------> I2C2_SDA
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10|LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C2);

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */

  /** I2C Initialization
  */
  LL_I2C_DisableOwnAddress2(I2C2);
  LL_I2C_DisableGeneralCall(I2C2);
  LL_I2C_EnableClockStretching(I2C2);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.ClockSpeed = 100000;
  I2C_InitStruct.DutyCycle = LL_I2C_DUTYCYCLE_2;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C2, &I2C_InitStruct);
  LL_I2C_SetOwnAddress2(I2C2, 0);
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /**SPI1 GPIO Configuration
  PA5   ------> SPI1_SCK
  PA7   ------> SPI1_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 10;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /**TIM2 GPIO Configuration
  PA15   ------> TIM2_CH1
  PB3   ------> TIM2_CH2
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  LL_TIM_SetEncoderMode(TIM2, LL_TIM_ENCODERMODE_X2_TI1);
  LL_TIM_IC_SetActiveInput(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
  LL_TIM_IC_SetPrescaler(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
  LL_TIM_IC_SetFilter(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV1);
  LL_TIM_IC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);
  LL_TIM_IC_SetActiveInput(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_DIRECTTI);
  LL_TIM_IC_SetPrescaler(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_ICPSC_DIV1);
  LL_TIM_IC_SetFilter(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_IC_FILTER_FDIV1);
  LL_TIM_IC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_RISING);
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 65535;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM2, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM2);
  LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM2);
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /**TIM3 GPIO Configuration
  PB4   ------> TIM3_CH1
  PB5   ------> TIM3_CH2
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4|LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  LL_TIM_SetEncoderMode(TIM3, LL_TIM_ENCODERMODE_X2_TI1);
  LL_TIM_IC_SetActiveInput(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
  LL_TIM_IC_SetPrescaler(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
  LL_TIM_IC_SetFilter(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV1);
  LL_TIM_IC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);
  LL_TIM_IC_SetActiveInput(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_DIRECTTI);
  LL_TIM_IC_SetPrescaler(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_ICPSC_DIV1);
  LL_TIM_IC_SetFilter(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_IC_FILTER_FDIV1);
  LL_TIM_IC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_RISING);
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 65535;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM3, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM3);
  LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM3);
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, CS_Pin|DDS_D7_Pin|DDS_CLK_Pin|DDS_FQ_Pin
                          |DDS_RST_Pin);

  /**/
  GPIO_InitStruct.Pin = CS_Pin|DDS_D7_Pin|DDS_CLK_Pin|DDS_FQ_Pin
                          |DDS_RST_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = SW_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(SW_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
