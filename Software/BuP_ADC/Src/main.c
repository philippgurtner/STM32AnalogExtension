/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint32_t ReadADC(uint8_t Channel);
uint32_t InitADC(void);


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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  InitADC();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  HAL_GPIO_WritePin(ADC_CS_GPIO_Port,ADC_CS_Pin,0);
	  HAL_Delay(100);

	  HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	  //HAL_Delay(500);

	  //if(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin))


	  /*if(HAL_GPIO_ReadPin(ADC_DRDY_GPIO_Port, ADC_DRDY_Pin)==0)
	  {

		  uint8_t buf[3]={0,0,0};
		  uint8_t rxbuf[5];

		  buf[0]= 0x12;	//RDATA => Read Data
		  HAL_SPI_Transmit(&hspi1,buf,1,10);

		  HAL_SPI_Receive(&hspi1, rxbuf,3, 10); //Read Datarate Register --> Ohne CRC und Status



	  }
	  */

	  ReadADC(1);
	  HAL_Delay(100);
	  HAL_GPIO_WritePin(ADC_CS_GPIO_Port,ADC_CS_Pin,1);

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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

uint32_t InitADC(void)
{
	uint8_t buf[3] = {0,0,0};
	uint8_t rxbuf[2]={0,0};

	HAL_Delay(2.2);
	HAL_GPIO_WritePin(ADC_RESET_GPIO_Port,ADC_RESET_Pin,1); // Set reset
	HAL_GPIO_WritePin(ADC_CS_GPIO_Port,ADC_CS_Pin,1);
	HAL_Delay(10);


	HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin,0);
	HAL_Delay(.5);

	buf[0]=0x21;	//Read Statusbyte
	buf[1]=0x00;	//Lenght 1

	HAL_SPI_Transmit(&hspi1,buf,2,10);
	HAL_SPI_Receive(&hspi1, rxbuf,1, 10);

	HAL_Delay(.5);
	HAL_GPIO_WritePin(ADC_CS_GPIO_Port,ADC_CS_Pin,1);
	HAL_Delay(.5);

	HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, ~(buf[0] & 0x40)); //Read Status, ist ADC Komunikationsbereit


	HAL_GPIO_WritePin(ADC_CS_GPIO_Port,ADC_CS_Pin,0);
	HAL_Delay(.5);

/*
	//Set Mode:
	////////////////////////////////////
	//Read Datarate Register (04h)
	buf[0]=0x24;
	buf[1]=0x00;
	HAL_SPI_Transmit(&hspi1,buf,2,10);
	HAL_Delay(1);
	HAL_SPI_Receive(&hspi1, rxbuf,1, 1);


	// Set Modified Datarateregister (04h)
	buf[0]=	0x44;	// Write Register DATARATE
	buf[1]= 0x00;	//Write 1 Register
	buf[2]=	rxbuf[0]|0x20; // Set Modebit
	HAL_SPI_Transmit(&hspi1,buf,3,10);


	// DEBUG
	//Read Datarate Register (04h) für Debugzwecke ob richtig geantowrted wird.
	buf[0]=0x24;
	buf[1]=0x00;
	HAL_SPI_Transmit(&hspi1,buf,2,10);
	HAL_Delay(1);
	HAL_SPI_Receive(&hspi1, rxbuf,1, 1);


	*/


	/////////////// Select INPUT
	buf[0] = 0x42; 		// Select INPMUX Register
	buf[1] = 0x00;		// Write 1 Registers
	buf[2]=0x13;
	HAL_SPI_Transmit(&hspi1,buf,3,10);



	/////////////// Read Back selected Input
	buf[0]=0x22;	//Read Statusbyte
	buf[1]=0x00;	//Lenght 1
	HAL_SPI_Transmit(&hspi1,buf,2,10);
	HAL_SPI_Receive(&hspi1, rxbuf,1, 10);


	buf[0]= 0x08;	//Start Convertion via Command
	HAL_SPI_Transmit(&hspi1,buf,1,10);


	HAL_Delay(1);
	HAL_GPIO_WritePin(ADC_CS_GPIO_Port,ADC_CS_Pin,1);

	return 0;
}

uint32_t ReadADC(uint8_t Channel) 	// Positive ADC input, Negativer ist AINCOM
{
	uint8_t buf[3]={0,0,0};
	uint8_t rxbuf[5];

	if (Channel>5)	// DS124S06 hat nur 5 Kanäle
		return 0;

	HAL_GPIO_WritePin(ADC_CS_GPIO_Port,ADC_CS_Pin,0);


	HAL_Delay(.5);
	HAL_GPIO_WritePin(ADC_CS_GPIO_Port,ADC_CS_Pin,1);
	HAL_Delay(.5);
	HAL_GPIO_WritePin(ADC_CS_GPIO_Port,ADC_CS_Pin,0);
	HAL_Delay(.5);


	// Delay
	HAL_Delay(.5);
	HAL_GPIO_WritePin(ADC_CS_GPIO_Port,ADC_CS_Pin,1);
	HAL_Delay(.5);

	while( HAL_GPIO_ReadPin(ADC_DRDY_GPIO_Port, ADC_DRDY_Pin) );

	HAL_GPIO_WritePin(ADC_CS_GPIO_Port,ADC_CS_Pin,0);
	HAL_Delay(1);

	// Eigentlich Warten bis Fertig Konvertiert !!!!!!!!!
	////////////////////////////////////
	/////////////////////////////////
	////////////////////////////////
	//////////////////////////////
	///////////////////////////
	////////////////////////
	/////////////////////
	///////////////////
	////////////////
	/////////////
	///////////
	////////
	//////
	////
	///
	//


	buf[0]= 0x12;	//RDATA => Read Data
	HAL_SPI_Transmit(&hspi1,buf,1,10);

	HAL_SPI_Receive(&hspi1, rxbuf,3, 10); //Read Datarate Register --> Ohne CRC und Status

	HAL_Delay(.5);
	HAL_GPIO_WritePin(ADC_CS_GPIO_Port,ADC_CS_Pin,1);

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
