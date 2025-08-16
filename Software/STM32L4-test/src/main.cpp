// #include "stm32l4xx.h"
// #include <stdio.h>
// #include <string.h>

// RTC_HandleTypeDef hrtc;

// UART_HandleTypeDef huart1;


// /* Private function prototypes -----------------------------------------------*/
// void SystemClock_Config(void);
// static void MX_GPIO_Init(void);
// static void MX_RTC_Init(void);
// static void MX_USART1_UART_Init(void);
// static void Error_Handler(void);
// /* USER CODE BEGIN PFP */
// /* USER CODE END PFP */

// /* Private user code ---------------------------------------------------------*/
// /* USER CODE BEGIN 0 */
// /* USER CODE END 0 */

// void Print_Time() {
//   // RTC_TimeTypeDef sTime;
//   char buf[64];

//   // HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
//   // HAL_RTC_GetDate(&hrtc, NULL, RTC_FORMAT_BIN); // Date not used in this example
//   snprintf(buf, sizeof(buf), "USART1 Test\r\n");
//   HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
// }


// int main(void) {

//   HAL_Init();

//   SystemClock_Config();

//   MX_GPIO_Init();
//   MX_RTC_Init();
//   MX_USART1_UART_Init();

//   while(1) {
//     Print_Time();
//     HAL_Delay(2000); // Delay for 1 second
//   }

// }

// void SystemClock_Config(void) {
//   // TODO: Double check that this can't be made more efficient. APPNOTE on this.
//   RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//   RCC_OscInitTypeDef RCC_OscInitStruct = {0};

//   /* MSI is enabled after System reset, activate PLL with MSI as source */
//   RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
//   RCC_OscInitStruct.MSIState = RCC_MSI_ON;
//   RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
//   RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
//   RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//   RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
//   RCC_OscInitStruct.PLL.PLLM = 1;
//   RCC_OscInitStruct.PLL.PLLN = 40;
//   RCC_OscInitStruct.PLL.PLLR = 2;
//   RCC_OscInitStruct.PLL.PLLQ = 4;
//   if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//   {
//     /* Initialization Error */
//     // TODO: Find a way to reset the chip if this occurs? Due to the structure of this, 
//     // this will PERMANANTLY lock up the die if this happens.
//     while(1);
//   }

  
//   /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
//      clocks dividers */
//   RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
//   RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//   RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//   RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
//   RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
//   if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
//   {
//     /* Initialization Error */
//     // TODO: Find a way to reset the chip if this occurs? Due to the structure of this, 
//     // this will PERMANANTLY lock up the die if this happens.
//     while(1);
//   }
// }

// /**
//   * @brief RTC Initialization Function
//   * @param None
//   * @retval None
//   */
// static void MX_RTC_Init(void)
// {

//   /* USER CODE BEGIN RTC_Init 0 */
//   /* USER CODE END RTC_Init 0 */

//   RTC_TimeTypeDef sTime = {0};
//   RTC_DateTypeDef sDate = {0};

//   /* USER CODE BEGIN RTC_Init 1 */
//   /* USER CODE END RTC_Init 1 */

//   /** Initialize RTC Only
//   */
//   hrtc.Instance = RTC;
//   hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
//   hrtc.Init.AsynchPrediv = 127;
//   hrtc.Init.SynchPrediv = 255;
//   hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
//   hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
//   hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
//   hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
//   hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
//   if (HAL_RTC_Init(&hrtc) != HAL_OK)
//   {
//     Error_Handler();
//   }

//   /* USER CODE BEGIN Check_RTC_BKUP */
//   /* USER CODE END Check_RTC_BKUP */

//   /** Initialize RTC and set the Time and Date
//   */
//   sTime.Hours = 0x01;
//   sTime.Minutes = 0x02;
//   sTime.Seconds = 0x03;
//   sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
//   sTime.StoreOperation = RTC_STOREOPERATION_RESET;
//   if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   sDate.WeekDay = RTC_WEEKDAY_MONDAY;
//   sDate.Month = RTC_MONTH_JUNE;
//   sDate.Date = 0x16;
//   sDate.Year = 0x25;

//   if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   /* USER CODE BEGIN RTC_Init 2 */
//   /* USER CODE END RTC_Init 2 */

// }

// /**
//   * @brief USART1 Initialization Function
//   * @param None
//   * @retval None
//   */
// static void MX_USART1_UART_Init(void)
// {

//   /* USER CODE BEGIN USART1_Init 0 */
//   /* USER CODE END USART1_Init 0 */

//   /* USER CODE BEGIN USART1_Init 1 */
//   /* USER CODE END USART1_Init 1 */
//   huart1.Instance = USART1;
//   huart1.Init.BaudRate = 115200;
//   huart1.Init.WordLength = UART_WORDLENGTH_8B;
//   huart1.Init.StopBits = UART_STOPBITS_1;
//   huart1.Init.Parity = UART_PARITY_NONE;
//   huart1.Init.Mode = UART_MODE_TX_RX;
//   huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//   huart1.Init.OverSampling = UART_OVERSAMPLING_16;
//   huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
//   huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
//   if (HAL_UART_Init(&huart1) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   /* USER CODE BEGIN USART1_Init 2 */
//   /* USER CODE END USART1_Init 2 */

// }

// /**
//   * @brief GPIO Initialization Function
//   * @param None
//   * @retval None
//   */
// static void MX_GPIO_Init(void)
// {
// /* USER CODE BEGIN MX_GPIO_Init_1 */
// /* USER CODE END MX_GPIO_Init_1 */

//   /* GPIO Ports Clock Enable */
//   __HAL_RCC_GPIOC_CLK_ENABLE();
//   __HAL_RCC_GPIOA_CLK_ENABLE();

// /* USER CODE BEGIN MX_GPIO_Init_2 */
// /* USER CODE END MX_GPIO_Init_2 */
// }

// /* USER CODE BEGIN 4 */
// /* USER CODE END 4 */

// /**
//   * @brief  This function is executed in case of error occurrence.
//   * @retval None
//   */
// void Error_Handler(void)
// {
//   /* USER CODE BEGIN Error_Handler_Debug */
//   /* USER CODE END Error_Handler_Debug */
// }


#include <Arduino.h>

void setup() {
  // Initialize Serial1 with a baud rate of 115200
  Serial.begin(115200);
  
  // Print a message to Serial1
  Serial.println("Hello from Serial1!");
}
void loop() {
  Serial.println("Serial1 is running...");
  // Add a small delay to avoid flooding the output
  delay(1000);
}