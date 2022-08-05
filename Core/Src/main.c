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
		TRANSFER_WAIT		= 0,
		TRANSFER_COMPLETE	= 1,
		TRANSFER_ERROR	 	= 2
	};

	#define	BUFFERSIZE	8

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

	char DataChar[0xFF];
	uint8_t aTxBuffer[BUFFERSIZE] = "SPI-DMA7" ;
	uint8_t aRxBuffer[BUFFERSIZE] = "01234567" ;
	__IO uint32_t wTransferState = TRANSFER_WAIT;
	int cnt_i=0;
	volatile uint8_t nss_flag = 0;
	extern DMA_HandleTypeDef hdma_spi2_rx;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

	uint16_t BufferCmp(uint8_t *pBuffer1, uint8_t *pBuffer2, uint16_t BufferLength);
	void PrintSPI2 (void);
	void ResetSPI2 (void) ;

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

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	sprintf(DataChar, "\r\n1Tx: " ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;
	snprintf(DataChar, BUFFERSIZE + 1 , "%s", (char*)aTxBuffer ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;
	sprintf(DataChar, "\r\n" ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;

	sprintf(DataChar, "1Rx: " ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;
	snprintf(DataChar, BUFFERSIZE + 1 , "%s", (char*)aRxBuffer ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;
	sprintf(DataChar, "\r\n" ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;

	wTransferState = TRANSFER_WAIT;
	nss_flag = 0;

	sprintf(DataChar,"SPI_TransmitReceive_DMA" ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;
	if(HAL_SPI_TransmitReceive_DMA(&hspi2, (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, BUFFERSIZE) != HAL_OK) {
		sprintf(DataChar," - FAIL\r\n" ) ;
		HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;
	} else {
		sprintf(DataChar," - Ok.\r\n" ) ;
		HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;
	}

	cnt_i = 0;
//	while (wTransferState == TRANSFER_WAIT) {
//		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//		sprintf(DataChar,"  TRANSFER_WAIT.. %d\r", cnt_i++ ) ;
//		HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;
//		HAL_Delay(100);
//	}

	while (nss_flag == 0) {
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		sprintf(DataChar,"  NSS not ready.. %d\r", cnt_i++ ) ;
		HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;
		HAL_Delay(100);
	}

//	ResetSPI2();

	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, SET);
//	sprintf(DataChar,"TRANSFER_COMPLETED\r\n" ) ;
//	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;

	sprintf(DataChar,"NSS Ready.\r\n" ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;

	sprintf(DataChar,"2Tx: " ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;
	snprintf(DataChar, BUFFERSIZE + 1 , "%s", (char*)aTxBuffer ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;
	sprintf(DataChar,"\r\n" ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;

	sprintf(DataChar,"2Rx: " ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;
	snprintf(DataChar, BUFFERSIZE + 1 , "%s", (char*)aRxBuffer ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;
	sprintf(DataChar,"\r\n" ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;

	PrintSPI2();

	uint16_t buffer_cmp_res = 0;

	switch(wTransferState) {
		case TRANSFER_COMPLETE :
			buffer_cmp_res = BufferCmp((uint8_t*)aTxBuffer, (uint8_t*)aRxBuffer, BUFFERSIZE);
			sprintf(DataChar,"buffer_cmp_res= %d\r\n", buffer_cmp_res ) ;
			HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;
			uint8_t byte_cnt = __HAL_DMA_GET_COUNTER(&hdma_spi2_rx);
			sprintf(DataChar,"DMA_GET_COUNTER= %d\r\n", byte_cnt ) ;
			HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;

		  if(buffer_cmp_res)  {
				sprintf(DataChar,"Buffer cmp - Wrong.\r\n") ;
				HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;
		  } else {
				sprintf(DataChar,"Buffer cmp - Success.\r\n") ;
				HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;
		  }
		break;
		default: {} break;
	}
//	HAL_Delay(10000);
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
	sprintf(DataChar,"\r\nCplt: TRANS_CMPLT\r\n" ) ;
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
	BufferLength++;
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

void PrintSPI2 (void) {
	__IO uint32_t 		myErrorCode		= hspi2.ErrorCode;
	//SPI_InitTypeDef		myInit 			= hspi2.Init;	//	struct SPI hardware
	//SPI_TypeDef* 		myInstance		= hspi2.Instance;	//Екземпляр
	HAL_LockTypeDef		myLock			= hspi2.Lock;	//	0-1-2-3
//	__SPI_HandleTypeDef*	myRxISR 	= hspi2.RxISR;
//	uint32_t myRxISR = hspi2.RxISR ;
//	uint32_t myTxISR = hspi2.TxISR ;


	uint16_t 			myRxXferCount	= hspi2.RxXferCount;
	uint16_t			myRxXferSize	= hspi2.RxXferSize;

	uint16_t 			myTxXferCount	= hspi2.TxXferCount;
	uint16_t			myTxXferSize	= hspi2.TxXferSize;

	uint32_t 			myInit_DataSize	= hspi2.Init.DataSize;

//	sprintf(DataChar,"\r\nmyTxISR \t%lu\r\n", myTxISR ) ;
//	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;
//	sprintf(DataChar,"myRxISR \t%lu\r\n", myRxISR ) ;
//	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;

	sprintf(DataChar,"myErrorCode \t%lu\r\n", myErrorCode ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;
	sprintf(DataChar,"myLock \t\t%u\r\n", myLock ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;
	sprintf(DataChar,"myRxXferCount \t%u\r\n", myRxXferCount ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;
	sprintf(DataChar,"myRxXferSize \t%u\r\n", myRxXferSize ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;
	sprintf(DataChar,"myTxXferCount \t%u\r\n", myTxXferCount ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;
	sprintf(DataChar,"myTxXferSize \t%u\r\n", myTxXferSize ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;
	sprintf(DataChar,"myInit_DataSize %lu\r\n", myInit_DataSize ) ;
	HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;

}
//-------------------------------------------------------

void ResetSPI2 (void) {
 hspi2.RxXferCount = 0 ;
 hspi2.RxXferSize  = 0 ;
 hspi2.TxXferCount = 0 ;
 hspi2.TxXferSize  = 0 ;
sprintf(DataChar,"\t-> Cleare SPI2 <-\r\n") ;
HAL_UART_Transmit( &huart1, (uint8_t *)DataChar , strlen(DataChar) , 100 ) ;

	//CLEAR_BIT(hspi->Instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);
	//hspi->State = HAL_SPI_STATE_READY;
	  HAL_DMA_IRQHandler(&hdma_spi2_rx);
//	  HAL_DMA_IRQHandler(&hdma_spi2_tx);
	  HAL_SPI_IRQHandler(&hspi2);

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
