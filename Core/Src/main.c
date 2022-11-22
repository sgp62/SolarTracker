/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "nmea.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define FIND_AND_NUL(s, p, c) ( \
   (p) = strchr(s, c), \
   *(p) = '\0', \
   ++(p), \
   (p))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart1;

osThreadId uartTaskHandle;
osThreadId spiTaskHandle;
osSemaphoreId SPI_semHandle;
osSemaphoreId UART_semHandle;
/* USER CODE BEGIN PV */

// FM25V02A FRAM SPI Commands
const uint8_t READ = 0b00000011;
const uint8_t WRITE = 0b00000010;
const uint8_t WREN = 0b00000110;
const uint8_t RDSR = 0b00000101;
const uint8_t WRSR = 0b00000001;
const uint8_t FSTRD = 0b00001011;
const uint8_t SLEEP = 0b10111001;
const uint8_t RDID = 0b10011111;


__IO uint32_t VirtualUserButtonStatus = 0;  /* set to 1 after User set a button  */

/* Buffer used for transmission */
uint8_t aTxBuffer[] = " **** UART_TwoBoards_ComPolling ****  **** UART_TwoBoards_ComPolling ****  **** UART_TwoBoards_ComPolling **** ";

/* Buffer used for reception */
uint8_t aRxBuffer[PROCESS_RX_BUFFER_SIZE];

SerialBuffer SerialBufferReceived;

/*
Notes :
- The GPS module returns data in the form of NMEA responses
- For example, we used the GGA response
- $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
For parsing, use the GPS Parser function to send the following form of arguments:
The name of the NMEA response that you want to parse, the position of the data that you need.
As a response - you will get a separate buffer with the requested data
*/

/* Private function prototypes -----------------------------------------------*/
//static void Error_Handler(void);
static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI3_Init(void);
void uartTaskFunc(void const * argument);
void spiTaskFunc(void const * argument);

/* USER CODE BEGIN PFP */
float GpsToDecimalDegrees(char* nmeaPos, char quadrant);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
QueueHandle_t xQueueSerialDataReceived;
/**
 * Convert NMEA absolute position to decimal degrees
 * "ddmm.mmmm" or "dddmm.mmmm" really is D+M/60,
 * then negated if quadrant is 'W' or 'S'
 */
float GpsToDecimalDegrees(char* nmeaPos, char quadrant)
{
  float v= 0;
  if(strlen(nmeaPos)>5)
  {
    char integerPart[3+1];
    int digitCount= (nmeaPos[4]=='.' ? 2 : 3);
    memcpy(integerPart, nmeaPos, digitCount);
    integerPart[digitCount]= 0;
    nmeaPos+= digitCount;
    v= atoi(integerPart) + atof(nmeaPos)/60.;
    if(quadrant=='W' || quadrant=='S')
      v= -v;
  }
  return v;
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

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
//  BSP_LED_Init(LED3);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
  //Enable Uart Interrupts
  HAL_NVIC_SetPriority(USART_GPS_IRQn, 7, 6);
  HAL_NVIC_EnableIRQ(USART_GPS_IRQn);
  USART_GPS->CR1 |= USART_CR1_RXNEIE; // Enable Interrupt

  //}
	/* Turn on LED3 if test passes then enter infinite loop */
//	BSP_LED_On(LED3);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of SPI_sem */
  osSemaphoreDef(SPI_sem);
  SPI_semHandle = osSemaphoreCreate(osSemaphore(SPI_sem), 1);

  /* definition and creation of UART_sem */
  osSemaphoreDef(UART_sem);
  UART_semHandle = osSemaphoreCreate(osSemaphore(UART_sem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  //Post UART sem for initial GPS read
  //xSemaphoreGive(UART_semHandle);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  xQueueSerialDataReceived = xQueueCreate( 2, sizeof( SerialBuffer) );
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of uartTask */
  osThreadDef(uartTask, uartTaskFunc, osPriorityNormal, 0, 512);
  uartTaskHandle = osThreadCreate(osThread(uartTask), NULL);

  /* definition and creation of spiTask */
  osThreadDef(spiTask, spiTaskFunc, osPriorityNormal, 0, 512);
  spiTaskHandle = osThreadCreate(osThread(spiTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */
  SPI3->CR1 |= SPI_CR1_SSM;
  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void USART_GPS_IRQHandler(void) // Sync and Queue NMEA Sentences
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	static char rx_buffer[LINEMAX + 1]; // Local holding buffer to build line, w/NUL
	static int rx_index = 0;
	if (USART_GPS->ISR & USART_ISR_ORE) // Overrun Error
		USART_GPS->ICR = USART_ICR_ORECF;
	if (USART_GPS->ISR & USART_ISR_NE) // Noise Error
		USART_GPS->ICR = USART_ICR_NCF;
	if (USART_GPS->ISR & USART_ISR_FE) // Framing Error
		USART_GPS->ICR = USART_ICR_FECF;
	if (USART_GPS->ISR & USART_ISR_RXNE) // Received character?
	{
		char rx = (char)(USART_GPS->RDR & 0xFF);
		if ((rx == '\r') || (rx == '\n')) // Is this an end-of-line condition, either will suffice?
		{
			if (rx_index != 0) // Line has some content?
			{
				rx_buffer[rx_index++] = 0; // Add NUL if required down stream
				//QueueBuffer(rx_buffer, rx_index); // Copy to queue from live dynamic receive buffer
				xQueueSendFromISR(xQueueSerialDataReceived,(void *)&rx_buffer,&xHigherPriorityTaskWoken);
				rx_index = 0; // Reset content pointer
				got_nmea = 1;
			}
		}
		else
		{
			if ((rx == '$') || (rx_index == LINEMAX)) // If resync or overflows pull back to start
				rx_index = 0;
			rx_buffer[rx_index++] = rx; // Copy to buffer and increment
		}
	}
}

/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
    Error_Handler();
}

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_12)
  {
    VirtualUserButtonStatus = 1;
  }
}

/**
  * @brief  Compares two buffers.
  * @param  pBuffer1, pBuffer2: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval 0  : pBuffer1 identical to pBuffer2
  *         >0 : pBuffer1 differs from pBuffer2
  */
static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    if ((*pBuffer1) != *pBuffer2)
    {
      return BufferLength;
    }
    pBuffer1++;
    pBuffer2++;
  }

  return 0;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_uartTaskFunc */
/**
  * @brief  Function implementing the uartTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_uartTaskFunc */
void uartTaskFunc(void const * argument)
{
  /* USER CODE BEGIN 5 */
	static uint8_t valid_count = 0;
	float latitude, longitude;

	char* message_id, *time, *data_valid, *raw_latitude, *raw_longitude, *latdir, *longdir;

	/* Infinite loop */
	for(;;)
	{
	  if(uxQueueMessagesWaitingFromISR(xQueueSerialDataReceived)>0)
	  {
		  if(valid_count == 0) {
			  //osSemaphoreAcquire(UART_semHandle, osWaitForever); //Grab semaphore for new message
			  xSemaphoreTake(UART_semHandle, portMAX_DELAY);
		  }

		  xQueueReceive(xQueueSerialDataReceived,&(SerialBufferReceived),1);
		  valid_count++;
		  //Fill and check header
		  for(int c = 0; c < 6; c++){
			  nmea_header[c] = SerialBufferReceived.Buffer[c];
		  }
		  if(!strcmp(nmea_header, "$GPRMC")){
			  if(SerialBufferReceived.Buffer[18] == 'A'){
				  //Got a fix0000
				  message_id = SerialBufferReceived.Buffer;
				  time = FIND_AND_NUL(message_id, time, ',');
				  data_valid = FIND_AND_NUL(time, data_valid, ',');
				  raw_latitude = FIND_AND_NUL(data_valid, raw_latitude, ',');
				  latdir = FIND_AND_NUL(raw_latitude, latdir, ',');
				  raw_longitude = FIND_AND_NUL(latdir, raw_longitude, ',');
				  longdir = FIND_AND_NUL(raw_longitude, longdir, ',');

				  latitude = GpsToDecimalDegrees(raw_latitude, latdir);
				  longitude = GpsToDecimalDegrees(raw_longitude, longdir);

				  if(valid_count >= 47){ //Length of NMEA message
					  valid_count = 0;
					  //Post SPI write semaphore when received full valid message
					  //osSemaphoreRelease(SPI_semHandle);
					  xSemaphoreGive(SPI_semHandle);
				  }
			  }
		  }
		  got_nmea=0;
	  }
	//osDelay(1);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_spiTaskFunc */
/**
* @brief Function implementing the spiTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_spiTaskFunc */
void spiTaskFunc(void const * argument)
{
  /* USER CODE BEGIN spiTaskFunc */
	HAL_StatusTypeDef response = HAL_ERROR;
  /* Infinite loop */
  for(;;)
  {
	  //osStatus stat = osSemaphoreAcquire(SPI_semHandle, osWaitForever); //Wait for nmea sem to be posted
	  xSemaphoreTake(SPI_semHandle, portMAX_DELAY);
	  osDelay(1);
	  //Send over SPI to FRAM
	  //SPI Initialization **************************
	  //Write CS Pin high
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	  // Enable write enable latch (allow write operations)
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi3, (uint8_t *)&WREN, 1, 100);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	  // Test bytes to write to EEPROM
	  spi_mout_buf[0] = 0xAB;
	  spi_mout_buf[1] = 0xCD;
	  spi_mout_buf[2] = 0xEF;

	  // Set starting address
	  spi_addr = 0x00;

	  // Write 3 bytes starting at given address
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi3, (uint8_t *)&WRITE, 1, 100);
	  HAL_SPI_Transmit(&hspi3, (uint8_t *)&spi_addr, 2, 100);
	  HAL_SPI_Transmit(&hspi3, (uint8_t *)spi_mout_buf, 3, 100);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	  //IO Driver for output pin enable

	  // Clear buffer
	  spi_mout_buf[0] = 0;
	  spi_mout_buf[1] = 0;
	  spi_mout_buf[2] = 0;

	  // Wait until WIP bit is cleared
	   spi_wip = 1;
	   while (spi_wip)
	   {
		 // Read status register
		 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		 HAL_SPI_Transmit(&hspi3, (uint8_t *)&RDSR, 1, 100);
		 response = HAL_SPI_Receive(&hspi3, (uint8_t *)spi_mout_buf, 1, 100);
		 if (response == HAL_OK) {
		  printf("Status Reg: %02x \r\n", spi_mout_buf[0]);
		 } else {
		  printf("Got error response as %d\r\n", response);
		 }
		 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

		 // Mask out WIP bit
		 spi_wip = spi_mout_buf[0] & 0b00000001;
	   }

	   // Read the 3 bytes back
	   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	   HAL_SPI_Transmit(&hspi3, (uint8_t *)&READ, 1, 5);
	   HAL_SPI_Transmit(&hspi3, (uint8_t *)&spi_addr, 2, 5);
	   HAL_SPI_Receive(&hspi3, (uint8_t *)spi_mout_buf, 3, 5);
	   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	   // Read status register
	   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	   HAL_SPI_Transmit(&hspi3, (uint8_t *)&RDSR, 1, 100);
	   HAL_SPI_Receive(&hspi3, (uint8_t *)spi_mout_buf, 1, 100);
	   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	  //osSemaphoreRelease(UART_semHandle); //Tell UART to gather more data
	  xSemaphoreGive(UART_semHandle);

  }
  /* USER CODE END spiTaskFunc */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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

#ifdef  USE_FULL_ASSERT
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
