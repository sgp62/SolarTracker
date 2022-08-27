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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nmea.h"
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
 UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* UART handler declaration */
UART_HandleTypeDef UartHandle;
__IO uint32_t VirtualUserButtonStatus = 0;  /* set to 1 after User set a button  */

/* Buffer used for transmission */
uint8_t aTxBuffer[] = " **** UART_TwoBoards_ComPolling ****  **** UART_TwoBoards_ComPolling ****  **** UART_TwoBoards_ComPolling **** ";

/* Buffer used for reception */
uint8_t aRxBuffer[PROCESS_RX_BUFFER_SIZE];

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
nmea_t gps;
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
  MX_USART1_UART_Init(); //Possibly initiating USART twice???
  /* USER CODE BEGIN 2 */
  /*##-1- Configure the UART peripheral ######################################*/
	/* Put the USART peripheral in the Asynchronous mode (UART Mode) */
//	/* UART configured as follows:
//		- Word Length = 8 Bits
//		- Stop Bit = One Stop bit
//		- Parity = None
//		- BaudRate = 9600 baud
//		- Hardware flow control disabled (RTS and CTS signals) */
//	UartHandle.Instance        = USART1;
//
//	UartHandle.Init.BaudRate     = 9600;
//	UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
//	UartHandle.Init.StopBits     = UART_STOPBITS_1;
//	UartHandle.Init.Parity       = UART_PARITY_NONE;
//	UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
//	UartHandle.Init.Mode         = UART_MODE_TX_RX;
//	UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

//	if(HAL_UART_DeInit(&huart1) != HAL_OK)
//	{
//	  Error_Handler();
//	}
//	if(HAL_UART_Init(&huart1) != HAL_OK)
//	{
//	  Error_Handler();
//	}

//	//NMEA library example
//	uint8_t got_one = 0;
//	nmea_init(&gps, USART1, 1024);
//	while(~got_one)
//	{
//		uint8_t time_h;
//		printf("Starting Receive\n");
//		nmea_loop(&gps);
//	    if (nmea_available(&gps))
//	    {
//	      nmea_gnss_time_h(&gps, &time_h);
//	      nmea_available_reset(&gps);
//	      got_one = 1;
//	    }
//	}


	for(int i = 0; i < RXBUFFERSIZE; i++){
		printf("%d",aRxBuffer[i]);
	}
	int end_flag = 0;
	int uart_counter = 0;
	while(~end_flag){
		//Receive one byte at a time until we reach carriage return (end of NMEA message)
		if(HAL_UART_Receive(&huart1, (uint8_t *)(aRxBuffer+uart_counter), 1, 0x1FFFFFF) != HAL_OK)
		{
		Error_Handler();
		}
		if(aRxBuffer[uart_counter++] == 13){//Found carriage return, exit
			end_flag = 1;
		}
	}

//	/*##-2- Put UART peripheral in reception process ###########################*/
//	if(HAL_UART_Receive(&UartHandle, (uint8_t *)aRxBuffer, RXBUFFERSIZE, 0x1FFFFFF) != HAL_OK)
//	{
//	Error_Handler();
//	}
//	printf("Done Receive\n");
//
//	for(int i = 0; i < RXBUFFERSIZE; i++){
//		printf("%d",aRxBuffer[i]);
//	}


//	/*##-3- Start the transmission process #####################################*/
//	/* While the UART in reception process, user can transmit data through
//	 "aTxBuffer" buffer */
//	if(HAL_UART_Transmit(&UartHandle, (uint8_t*)aTxBuffer, TXBUFFERSIZE, 5000)!= HAL_OK)
//	{
//	Error_Handler();
//	}
//
//	/*##-4- Compare the sent and received buffers ##############################*/
//	if(Buffercmp((uint8_t*)aTxBuffer,(uint8_t*)aRxBuffer,RXBUFFERSIZE))
//	{
//	Error_Handler();
//	}

	/* Turn on LED3 if test passes then enter infinite loop */
//	BSP_LED_On(LED3);
  /* USER CODE END 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

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
