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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* DUAL_CORE_BOOT_SYNC_SEQUENCE: Define for dual core boot synchronization    */
/*                             demonstration code based on hardware semaphore */
/* This define is present in both CM7/CM4 projects                            */
/* To comment when developping/debugging on a single core                     */
#define DUAL_CORE_BOOT_SYNC_SEQUENCE

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

DCMI_HandleTypeDef hdcmi;
DMA_HandleTypeDef hdma_dcmi;

I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
#define OV7670_ADDR  0x42    /* 8-bit SCCB write address; HAL adds the R/W bit  */
#define BUF_WORDS    40000   /* 160 KB: 320x240 RGB565 = 153,600 B + margin */

/* Lands in AXI SRAM (0x24xxxxxx) with this project's linker script: DMA1 can
   reach it. DTCM (0x20xxxxxx) would NOT work. D-cache is off, so no cache
   maintenance is needed yet. */
static uint32_t frame[BUF_WORDS] __attribute__((aligned(32)));
static volatile uint8_t frame_done = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_DCMI_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void uart_print(const char *s)
{
  HAL_UART_Transmit(&huart3, (uint8_t *)s, strlen(s), 100);
}

/* SCCB read = write register address, STOP, then a separate read.
   Two transfers on purpose: the OV7670's SCCB does not support a repeated START. */
static HAL_StatusTypeDef ov_read(uint8_t reg, uint8_t *val)
{
  HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(&hi2c2, OV7670_ADDR, &reg, 1, 100);
  if (st != HAL_OK) return st;
  return HAL_I2C_Master_Receive(&hi2c2, OV7670_ADDR, val, 1, 100);
}

static HAL_StatusTypeDef ov_write(uint8_t reg, uint8_t val)
{
  uint8_t b[2] = { reg, val };
  return HAL_I2C_Master_Transmit(&hi2c2, OV7670_ADDR, b, 2, 100);
}

/* HAL calls this from the DCMI interrupt when the frame ends (VSYNC). */
void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *h)
{
  (void)h;
  frame_done = 1;
}

/* Count level changes on PCLK (PA6), HREF (PA4), VSYNC (PB7) for 1 s by polling.
   Polling can't count every PCLK edge, but a non-zero count proves the signal
   toggles on that exact pin. */
static void sig_probe(void)
{
  uint32_t n_p = 0, n_h = 0, n_v = 0;
  uint32_t lp = GPIOA->IDR & (1u << 6);
  uint32_t lh = GPIOA->IDR & (1u << 4);
  uint32_t lv = GPIOB->IDR & (1u << 7);
  uint32_t t0 = HAL_GetTick();
  while (HAL_GetTick() - t0 < 1000) {
    uint32_t a = GPIOA->IDR, b = GPIOB->IDR;
    uint32_t p = a & (1u << 6), h = a & (1u << 4), v = b & (1u << 7);
    if (p != lp) { n_p++; lp = p; }
    if (h != lh) { n_h++; lh = h; }
    if (v != lv) { n_v++; lv = v; }
  }
  char m[120];
  snprintf(m, sizeof m, "edges/1s: PCLK(PA6)=%lu HREF(PA4)=%lu VSYNC(PB7)=%lu\r\n",
           (unsigned long)n_p, (unsigned long)n_h, (unsigned long)n_v);
  uart_print(m);
}

/* Full VGA RGB565, camera scaler OFF (the DCMI does the 2x downscale).
   Writes, then reads every register back so a silent SCCB failure can't hide. */
static void camera_config(void)
{
  char m[96];
  int fails = 0, mism = 0;

  if (ov_write(0x12, 0x80) != HAL_OK) fails++;      /* COM7: reset all regs */
  HAL_Delay(50);

  static const uint8_t cfg[][2] = {
    {0x11, 0x01},   /* CLKRC: internal clock = XCLK/2 -> PCLK ~8 MHz      */
    {0x12, 0x04},   /* COM7: VGA, RGB                                     */
    {0x40, 0xD0},   /* COM15: RGB565, full range                          */
    {0x8C, 0x00},   /* RGB444 off                                         */
    {0x0C, 0x00},   /* COM3: no DCW scaling                               */
    {0x3E, 0x00},   /* COM14: normal PCLK (no divider)                    */
    {0x70, 0x3A},   /* scaling regs at their no-scale values              */
    {0x71, 0x35},   /*   (bit7 = 0 -> NO test pattern, real image)        */
    {0x72, 0x11},
    {0x73, 0xF0},
    {0xA2, 0x02},
  };
  const int n = sizeof cfg / sizeof cfg[0];

  for (int i = 0; i < n; i++) {
    if (ov_write(cfg[i][0], cfg[i][1]) != HAL_OK) fails++;
    HAL_Delay(2);
  }
  for (int i = 0; i < n; i++) {
    uint8_t v = 0;
    ov_read(cfg[i][0], &v);
    if (v != cfg[i][1]) {
      mism++;
      snprintf(m, sizeof m, "  reg 0x%02X wrote 0x%02X read 0x%02X\r\n", cfg[i][0], cfg[i][1], v);
      uart_print(m);
    }
  }
  snprintf(m, sizeof m, "CFG write_fails=%d readback_mismatch=%d\r\n", fails, mism);
  uart_print(m);
  HAL_Delay(300);                                   /* let the camera settle */
}

/* Colour / exposure / white-balance tuning. Values from the Linux ov7670
   driver (ov7670_default_regs + ov7670_fmt_rgb565, J. Corbet, GPL-2.0).
   Clock and window registers are deliberately NOT included: our 320x240
   geometry already works and must not change. */
static void camera_tune(void)
{
  static const uint8_t t[][2] = {
    /* gamma curve */
    {0x7a,0x20},{0x7b,0x10},{0x7c,0x1e},{0x7d,0x35},{0x7e,0x5a},{0x7f,0x69},
    {0x80,0x76},{0x81,0x80},{0x82,0x88},{0x83,0x8f},{0x84,0x96},{0x85,0xa3},
    {0x86,0xaf},{0x87,0xc4},{0x88,0xd7},{0x89,0xe8},
    /* AGC/AEC: disable, set limits, re-enable */
    {0x13,0xe0},{0x00,0x00},{0x10,0x00},{0x0d,0x40},{0x14,0x18},
    {0xa5,0x05},{0xab,0x07},{0x24,0x95},{0x25,0x33},{0x26,0xe3},
    {0x9f,0x78},{0xa0,0x68},{0xa1,0x03},{0xa6,0xd8},{0xa7,0xd8},
    {0xa8,0xf0},{0xa9,0x90},{0xaa,0x94},{0x13,0xe5},
    /* reserved "magic" values from OmniVision */
    {0x0e,0x61},{0x0f,0x4b},{0x16,0x02},{0x1e,0x07},{0x21,0x02},{0x22,0x91},
    {0x29,0x07},{0x33,0x0b},{0x35,0x0b},{0x37,0x1d},{0x38,0x71},{0x39,0x2a},
    {0x3c,0x78},{0x4d,0x40},{0x4e,0x20},{0x69,0x00},{0x74,0x10},   /* 0x6B (PLL x4) removed! */
    {0x8d,0x4f},{0x8e,0x00},{0x8f,0x00},{0x90,0x00},{0x91,0x00},{0x96,0x00},
    {0x9a,0x00},{0xb0,0x84},{0xb1,0x0c},{0xb2,0x0e},{0xb3,0x82},{0xb8,0x0a},
    /* white balance */
    {0x43,0x0a},{0x44,0xf0},{0x45,0x34},{0x46,0x58},{0x47,0x28},{0x48,0x3a},
    {0x59,0x88},{0x5a,0x88},{0x5b,0x44},{0x5c,0x67},{0x5d,0x49},{0x5e,0x0e},
    {0x6c,0x0a},{0x6d,0x55},{0x6e,0x11},{0x6f,0x9f},{0x6a,0x40},{0x01,0x40},
    {0x02,0x60},{0x13,0xe7},            /* AGC + AEC + AWB on */
    /* misc: edge, denoise, banding filter */
    {0x3f,0x00},{0x75,0x05},{0x76,0xe1},{0x4c,0x00},{0x77,0x01},{0x4b,0x09},
    {0xc9,0x60},{0x56,0x40},{0x34,0x11},{0x3b,0x12},{0xa4,0x88},{0x97,0x30},
    {0x98,0x20},{0x99,0x30},{0x9a,0x84},{0x9b,0x29},{0x9c,0x03},{0x9d,0x4c},
    {0x9e,0x3f},{0x78,0x04},
    /* RGB565-specific: gain ceiling, colour matrix, gamma + UV saturation */
    {0x14,0x38},{0x4f,0xb3},{0x50,0xb3},{0x51,0x00},{0x52,0x3d},{0x53,0xa7},
    {0x54,0xe4},{0x58,0x9e},{0x3d,0xc0},{0x41,0x38},
  };
  int fails = 0;
  for (unsigned i = 0; i < sizeof t / sizeof t[0]; i++) {
    if (ov_write(t[i][0], t[i][1]) != HAL_OK) fails++;
    HAL_Delay(1);
  }
  char m[48];
  snprintf(m, sizeof m, "TUNE write_fails=%d\r\n", fails);
  uart_print(m);
  HAL_Delay(500);            /* let auto exposure / white balance settle */
}

/* Capture one frame into a deliberately oversized buffer, measure how many
   words really arrived, then send "INFO ...", "FRAME <n>\n" and n raw bytes. */
static void capture_and_send(void)
{
  char m[128];
  memset(frame, 0, sizeof frame);
  frame_done = 0;
  hdcmi.ErrorCode = HAL_DCMI_ERROR_NONE;   /* don't report errors of an old capture */
  if (HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)frame, BUF_WORDS) != HAL_OK)
    uart_print("INFO Start_DMA FAILED\r\n");

  uint32_t t0 = HAL_GetTick();
  while (!frame_done && (HAL_GetTick() - t0) < 2000) { }   /* ~7 fps: <= 300 ms needed */

  uint32_t words = BUF_WORDS - __HAL_DMA_GET_COUNTER(hdcmi.DMA_Handle);  /* real size */
  HAL_DCMI_Stop(&hdcmi);
  uint32_t n = words * 4;

  snprintf(m, sizeof m, "INFO done=%d words=%lu bytes=%lu dcmi_err=0x%lX\r\n",
           frame_done, (unsigned long)words, (unsigned long)n, (unsigned long)hdcmi.ErrorCode);
  uart_print(m);
  snprintf(m, sizeof m, "FRAME %lu\n", (unsigned long)n);
  uart_print(m);

  uint8_t *p = (uint8_t *)frame;          /* HAL_UART_Transmit takes max 65535 bytes */
  while (n) {
    uint16_t c = (n > 32768) ? 32768 : (uint16_t)n;
    HAL_UART_Transmit(&huart3, p, c, 10000);
    p += c;
    n -= c;
  }
}
/* ---- PCLK frequency measurement -------------------------------------------
   PA6 is borrowed from the DCMI for a few ms and routed to TIM3_CH1 (AF2).
   TIM3 counts rising edges (external clock mode 1) for exactly 1 ms, timed by
   the Cortex-M7 DWT cycle counter. Counted 3 times with different input
   filters: glitches/ringing are counted only by the unfiltered version.     */
static uint32_t count_pclk_1ms(uint32_t filt)
{
  TIM3->CR1   = 0;
  TIM3->SMCR  = 0;
  TIM3->CCER  = 0;                                   /* rising edge          */
  TIM3->CCMR1 = TIM_CCMR1_CC1S_0 | (filt << TIM_CCMR1_IC1F_Pos);
  TIM3->SMCR  = (5u << TIM_SMCR_TS_Pos)              /* trigger = TI1FP1     */
              | (7u << TIM_SMCR_SMS_Pos);            /* external clock mode 1*/
  TIM3->PSC = 0; TIM3->ARR = 0xFFFF;
  TIM3->EGR = TIM_EGR_UG; TIM3->CNT = 0;

  uint32_t cyc = HAL_RCC_GetSysClockFreq() / 1000;   /* CPU cycles in 1 ms   */
  uint32_t t0 = DWT->CYCCNT;
  TIM3->CR1 = TIM_CR1_CEN;
  while (DWT->CYCCNT - t0 < cyc) { }
  TIM3->CR1 = 0;
  return TIM3->CNT;                                  /* edges in 1 ms = kHz  */
}

static void measure_pclk(void)
{
  char m[128];
  HAL_DCMI_DeInit(&hdcmi);                           /* release PA6          */

  __HAL_RCC_TIM3_CLK_ENABLE();
  GPIO_InitTypeDef g = {0};
  g.Pin = GPIO_PIN_6; g.Mode = GPIO_MODE_AF_PP; g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_VERY_HIGH; g.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOA, &g);

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;    /* enable DWT           */
  DWT->LAR = 0xC5ACCE55;                             /* unlock (Cortex-M7)   */
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  uint32_t f0 = count_pclk_1ms(0);   /* no filter: counts glitches too       */
  uint32_t f1 = count_pclk_1ms(1);   /* ignores pulses shorter than ~25 ns   */
  uint32_t f2 = count_pclk_1ms(2);   /* ignores pulses shorter than ~50 ns   */
  snprintf(m, sizeof m, "PCLK kHz: raw=%lu  filt25ns=%lu  filt50ns=%lu\r\n",
           (unsigned long)f0, (unsigned long)f1, (unsigned long)f2);
  uart_print(m);

  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6);
  __HAL_RCC_TIM3_CLK_DISABLE();
  HAL_DCMI_Init(&hdcmi);                             /* PA6 back to DCMI     */
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  int32_t timeout;
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_USART3_UART_Init();
  MX_DCMI_Init();
  /* USER CODE BEGIN 2 */
  char msg[96];
  HAL_Delay(100);                                   /* camera wakes on XCLK */

  snprintf(msg, sizeof msg, "\r\n--- OV7670 DCMI (H755 CM7) ---\r\nframe buffer @ 0x%08lX\r\n",
           (unsigned long)(uintptr_t)frame);
  uart_print(msg);

  uint8_t pid = 0, ver = 0;
  ov_read(0x0A, &pid);
  ov_read(0x0B, &ver);
  snprintf(msg, sizeof msg, "PID=0x%02X VER=0x%02X\r\n", pid, ver);
  uart_print(msg);

  camera_config();
  camera_tune();                                    /* colours / exposure / AWB */

  /* DMA priority -> very high (CubeMX has "Low"): fewer DCMI overruns.
     Allowed only while the stream is disabled - it is, no capture yet. */
  ((DMA_Stream_TypeDef *)hdma_dcmi.Instance)->CR |= DMA_SxCR_PL;

  /* Let the DCMI downscale: keep 2 bytes of every 4 (= every other pixel)
     and every other line -> 640x480 becomes 320x240. */
  hdcmi.Init.ByteSelectMode  = DCMI_BSM_ALTERNATE_2;
  hdcmi.Init.ByteSelectStart = DCMI_OEBS_EVEN;
  hdcmi.Init.LineSelectMode  = DCMI_LSM_ALTERNATE_2;
  hdcmi.Init.LineSelectStart = DCMI_OELS_ODD;
  HAL_DCMI_Init(&hdcmi);
  measure_pclk();                                   /* real PCLK frequency */
  sig_probe();
  uart_print("READY: send 'c' for a frame\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint8_t cmd;
    __HAL_UART_CLEAR_OREFLAG(&huart3);    /* clear a stale overrun so RX keeps working */
    if (HAL_UART_Receive(&huart3, &cmd, 1, 100) == HAL_OK && cmd == 'c')
      capture_and_send();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_4);
}

/**
  * @brief DCMI Initialization Function
  * @param None
  * @retval None
  */
static void MX_DCMI_Init(void)
{

  /* USER CODE BEGIN DCMI_Init 0 */

  /* USER CODE END DCMI_Init 0 */

  /* USER CODE BEGIN DCMI_Init 1 */

  /* USER CODE END DCMI_Init 1 */
  hdcmi.Instance = DCMI;
  hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
  hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;
  hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_HIGH;
  hdcmi.Init.HSPolarity = DCMI_HSPOLARITY_LOW;
  hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
  hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  hdcmi.Init.JPEGMode = DCMI_JPEG_DISABLE;
  hdcmi.Init.ByteSelectMode = DCMI_BSM_ALL;
  hdcmi.Init.ByteSelectStart = DCMI_OEBS_ODD;
  hdcmi.Init.LineSelectMode = DCMI_LSM_ALL;
  hdcmi.Init.LineSelectStart = DCMI_OELS_ODD;
  if (HAL_DCMI_Init(&hdcmi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DCMI_Init 2 */

  /* USER CODE END DCMI_Init 2 */

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

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00909BEB;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
