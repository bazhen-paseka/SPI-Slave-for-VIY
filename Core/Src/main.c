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
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

	#include "stdio.h"
	#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

	enum {
		TRANSFER_WAIT,
		TRANSFER_COMPLETE,
		TRANSFER_ERROR
	};

	#define	BUFFERSIZE	8

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

	SPI_HandleTypeDef SpiHandle;
	char DataChar[0xFF];
	uint8_t aTxBuffer[BUFFERSIZE] = "SPI-DMA" ;
	uint8_t aRxBuffer[BUFFERSIZE] = "0123456" ;
	__IO uint32_t wTransferState = TRANSFER_WAIT;
	int cnt_i=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

	uint16_t BufferCmp(uint8_t *pBuffer1, uint8_t *pBuffer2, uint16_t BufferLength);

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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

	sprintf(DataChar,"\r\n\r\n\tSPI+DMA SLAVE for VIY.UA\r\n" );
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;

	#define DATE_as_int_str 	(__DATE__)
	#define TIME_as_int_str 	(__TIME__)
	sprintf(DataChar,"\tBuild: %s. Time: %s." , DATE_as_int_str , TIME_as_int_str ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;

	sprintf(DataChar,"\r\n\tfor debug: UART1 115200/8-N-1\r\n" ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;

	snprintf(DataChar, BUFFERSIZE + 7 , "1Tx: %s\r\n", aTxBuffer ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;

	snprintf(DataChar, BUFFERSIZE + 7 , "1Rx: %s\r\n", aRxBuffer ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;

/*##-1- Configure the SPI peripheral #######################################*/
	SpiHandle.Instance               = SPI2;
	SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
	SpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
	SpiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
	SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
	SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
	SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;
	SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
	SpiHandle.Init.CRCPolynomial     = 10;
	SpiHandle.Init.NSS               = SPI_NSS_SOFT;
	SpiHandle.Init.Mode 			 = SPI_MODE_SLAVE;

	sprintf(DataChar,"SLAVE\r\n" ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;

	if(HAL_SPI_Init(&SpiHandle) != HAL_OK)  {
		sprintf(DataChar,"SPI_Init - FAIL\r\n" ) ;
		HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;
	} else {
		sprintf(DataChar,"SPI_Init - Ok\r\n" ) ;
		HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;
	}
	/* SPI block is enabled prior calling SPI transmit/receive functions, in order to get CLK signal properly pulled down.
	 Otherwise, SPI CLK signal is not clean on this board and leads to errors during transfer */
	__HAL_SPI_ENABLE(&SpiHandle);

	sprintf(DataChar,"SPI_TransmitReceive_DMA Start... " ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;
	if(HAL_SPI_TransmitReceive_DMA(&SpiHandle, (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, BUFFERSIZE) != HAL_OK) {
//	if(HAL_SPI_TransmitReceive_DMA(&hspi2, (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, BUFFERSIZE) != HAL_OK) {
		sprintf(DataChar," - FAIL\r\n" ) ;
		HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;
	} else {
		sprintf(DataChar," - Ok.\r\n" ) ;
		HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;
	}

	cnt_i = 0;
	while (wTransferState == TRANSFER_WAIT) {
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		sprintf(DataChar,"  TRANSFER_WAIT.. %d\r", cnt_i++ ) ;
		HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;
		HAL_Delay(100);
	}

	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, SET);
	sprintf(DataChar,"\r\nTRANSFER_COMPLETED\r\n" ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;

	snprintf(DataChar, BUFFERSIZE + 7 , "2Tx: %s\r\n", aTxBuffer ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;

	sprintf(DataChar,"2Rx: " ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;
	snprintf(DataChar, BUFFERSIZE + 1 , "%s", aRxBuffer ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;
	sprintf(DataChar,"\r\n" ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;

	uint16_t buffer_cmp_res = 0;

	switch(wTransferState) {
		case TRANSFER_COMPLETE :
			buffer_cmp_res = BufferCmp((uint8_t*)aTxBuffer, (uint8_t*)aRxBuffer, BUFFERSIZE);
			sprintf(DataChar,"buffer_cmp_res= %d\r\n", buffer_cmp_res ) ;
			HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;

		  if(buffer_cmp_res)  {
				sprintf(DataChar,"Buffer cmp - Wrong.\r\n") ;
				HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;
		  } else {
				sprintf(DataChar,"Buffer cmp - Successfully.\r\n") ;
				HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;
		  }
		break;
		default: {} break;
	}

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, RESET);
	sprintf(DataChar,"Cplt-TRANSFER_COMPLETE\r\n" ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;
	wTransferState = TRANSFER_COMPLETE;
}
//-------------------------------------------------------

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
	sprintf(DataChar,"!!!-TRANSFER_ERROR-!!!\r\n" ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;
	wTransferState = TRANSFER_ERROR;
}
//-------------------------------------------------------

uint16_t BufferCmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength) {
	while (BufferLength--) {
		if((*pBuffer1) != *pBuffer2) {
			return BufferLength;
		}
			pBuffer1++;
			pBuffer2++;
	}
	return 0;
}
//-------------------------------------------------------

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
