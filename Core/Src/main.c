/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - RESTORED PMS LOGGING
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ff.h"
#include <stdio.h>
#include <string.h>
#include "ssd1306.h"
#include "ssd1306_fonts.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi2;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV --------------------------------------------------------*/
// --- PMS Sensor Variables ---
#define PMS_UART &huart4
#define DEBUG_UART &huart2

uint8_t pms_buffer[32];
uint16_t pm1_0_env, pm2_5_env, pm10_0_env;

// --- SD Card Variables ---
FATFS fs;             // File system object
FIL fil;              // File object
FRESULT fres;         // Result code
UINT bytesWritten;    // Counter
char logData[100];    // Data buffer
// Timer tracker
uint32_t last_timer_tick = 0;

/* USER CODE END PV ----------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART4_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP -------------------------------------------------------*/
int _write(int file, char *ptr, int len);
uint8_t receive_pms_byte(void);
uint8_t parse_pms_packet(void);
void ScrollErrorText(void);
/* USER CODE END PFP ---------------------------------------------------------*/

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
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART4_UART_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  if (MX_FATFS_Init() != APP_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN 2 */

  // 1. Initialize OLED
  ssd1306_Init();
  ssd1306_Fill(Black);
  ssd1306_SetCursor(10, 10);
  ssd1306_WriteString("WELCOME TO ST32", Font_7x10, White);
  ssd1306_UpdateScreen();

  // 2. Mount SD Card (Just once at startup)
  printf("Mounting SD Card...\r\n");
  fres = f_mount(&fs, "", 1);
  if(fres == FR_OK) {
      printf("SD Card Mounted Successfully.\r\n");
      ssd1306_SetCursor(10, 40);
      ssd1306_WriteString("SD Running", Font_7x10, White);
  } else {
      printf("SD Mount Error: %d\r\n", fres);
      ssd1306_SetCursor(10, 25);
      ssd1306_WriteString("SD Error!", Font_7x10, White);
  }
  ssd1306_UpdateScreen();
  HAL_Delay(1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      uint8_t rx_byte;

      // --- PART 1: READ PMS SENSOR ---

      // Check for Header 0x42
      rx_byte = receive_pms_byte();
      if (rx_byte != 0x42) continue;
      pms_buffer[0] = rx_byte;

      // Check for Header 0x4D
      rx_byte = receive_pms_byte();
      if (rx_byte != 0x4D) continue;
      pms_buffer[1] = rx_byte;

      // Read the rest of the packet
      if (HAL_UART_Receive(PMS_UART, &pms_buffer[2], 30, 1000) != HAL_OK) {
          continue; // Timeout, try again
      }

      // --- PART 2: PROCESS & DISPLAY ---
      if (parse_pms_packet())
      {
          // Print to Serial
          printf("--- New Data ---\r\n");
          printf("PM 1.0: %u\r\n", pm1_0_env);
          printf("PM 2.5: %u\r\n", pm2_5_env);
          printf("PM 10.0: %u\r\n", pm10_0_env);

          // Format: Year-Month-Day Hour:Min:Sec, PM1, PM2.5, PM10

          // Update OLED
          char display_buffer[32];
          ssd1306_Fill(Black);

          sprintf(display_buffer, "PM 1.0: %u", pm1_0_env);
          ssd1306_SetCursor(5, 5);
          ssd1306_WriteString(display_buffer, Font_6x8, White);

          sprintf(display_buffer, "PM 2.5: %u", pm2_5_env);
          ssd1306_SetCursor(5, 15);
          ssd1306_WriteString(display_buffer, Font_6x8, White);

          sprintf(display_buffer, "PM 10 : %u", pm10_0_env);
          ssd1306_SetCursor(5, 25);
          ssd1306_WriteString(display_buffer, Font_6x8, White);

          ssd1306_UpdateScreen();

          // --- PART 3: SAVE TO SD CARD ---

          // Prepare CSV string
          // Open file in APPEND mode
          sprintf(logData, "%u,%u,%u\r\n", pm1_0_env, pm2_5_env, pm10_0_env);
          fres = f_open(&fil, "PMSLOG.CSV", FA_WRITE | FA_OPEN_APPEND);

          if(fres == FR_OK) {
        	  // --- HEADER LOGIC START ---
        	  // Check if file is empty (Size = 0). If so, write the Titles first!
        	                if (f_size(&fil) == 0) {
        	                    char header[] = "PM1.0 (ug/m3), PM2.5 (ug/m3), PM10 (ug/m3)\r\n";
        	                    f_write(&fil, header, strlen(header), &bytesWritten);
        	                }
        	 // --- HEADER LOGIC END ---




              printf("Storing: %s\n", logData); // Debug message
              ssd1306_SetCursor(1, 50);
              ssd1306_WriteString("Storing to SD...", Font_6x8, White);
              f_write(&fil, logData, strlen(logData), &bytesWritten);
              f_close(&fil);
              ssd1306_UpdateScreen();
          } else {
              printf("SD Write Error: %d\r\n", fres);
              ssd1306_SetCursor(1, 50);
              ssd1306_WriteString("Not Storing to Sd card...", Font_6x8, White);
              ssd1306_UpdateScreen();
              HAL_Delay(300);

              // Try to remount if error persists?
              if(fres == FR_NOT_READY) f_mount(&fs, "", 1);
          }
      }

      // Wait before next reading

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* Initialization Functions (GPIO, I2C, SPI, UART) - Standard CubeMX Generated */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00602173;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) Error_Handler();
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) Error_Handler();
}

static void MX_SPI2_Init(void)
{
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK) Error_Handler();
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

static void MX_USART4_UART_Init(void)
{
  huart4.Instance = USART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_5, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

// --- Helper Functions ---

uint8_t receive_pms_byte(void)
{
  uint8_t byte;
  HAL_UART_Receive(PMS_UART, &byte, 1, 1000);
  return byte;
}

uint8_t parse_pms_packet(void)
{
  uint16_t checksum_calc = 0;
  uint16_t checksum_rcvd = 0;

  for (int i = 0; i < 30; i++) {
    checksum_calc += pms_buffer[i];
  }
  checksum_rcvd = (pms_buffer[30] << 8) | pms_buffer[31];

  if (checksum_calc != checksum_rcvd) return 0; // Fail

  pm1_0_env = (pms_buffer[10] << 8) | pms_buffer[11];
  pm2_5_env = (pms_buffer[12] << 8) | pms_buffer[13];
  pm10_0_env = (pms_buffer[14] << 8) | pms_buffer[15];
  return 1; // Success
}

int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(DEBUG_UART, (uint8_t*)ptr, len, HAL_MAX_DELAY);
  return len;
}
/* USER CODE END 4 */

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}
