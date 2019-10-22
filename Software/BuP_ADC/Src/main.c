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
uint32_t InitADC(void);
uint32_t ReadADCChannel(uint8_t Channel);
void writeADC(uint8_t startadress, uint8_t length, uint8_t *data);
void ReadADC(uint8_t startadress, uint8_t length, uint8_t *rxdata);
void ADCwrite_1register(uint8_t startadress, uint8_t data);
void start(void);

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
	  HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);


	  ReadADCChannel(1);
	  HAL_Delay(500);


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
	uint8_t txbuf[18] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	uint8_t rxbuf[18] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	HAL_Delay(2.2);
	HAL_GPIO_WritePin(ADC_RESET_GPIO_Port,ADC_RESET_Pin,1); // Set reset
	HAL_GPIO_WritePin(ADC_CS_GPIO_Port,ADC_CS_Pin,1);
	HAL_Delay(10);


	ReadADC(0x21, 1, rxbuf); 	// Read Statusbyte to rxbuf
	
	//Set Mode:

	//ReadADC(0x04,1,rxbuf);	//Read Datarate Register (04h)
	//ADCwrite_1register(0x02,rxbuf[0]|0x20); // Set Modified Datarateregister (04h)
	//ReadADC(0x04, 1, rxbuf); 	// Read Statusbyte to rxbuf


	// Select INPUT (INPMUX- Register)
	ADCwrite_1register(0x02,0x13);
	ReadADC(0x02, 1, rxbuf); 	// Read Statusbyte to rxbuf

	start();

	return 0;
}


void ADCwrite_1register(uint8_t startadress, uint8_t data)
{
	uint8_t txdata[1] = {data};
	writeADC(startadress, 1, txdata);
}


void start(void)
{
	HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin,0);
	HAL_Delay(.5);

	uint8_t txbuf[1]= {0x08};				//Start Convertion via Command
	HAL_SPI_Transmit(&hspi1,txbuf,1,10);

	HAL_Delay(.5);
	HAL_GPIO_WritePin(ADC_CS_GPIO_Port,ADC_CS_Pin,1);
}


void writeADC(uint8_t startadress, uint8_t length, uint8_t *data)
{
	if(length==0 && length >0x11)	// sonst werden keine Daten gesendet, 0x11= Max register anzahl (vgl. Table 25. Datenblatt) 
	{	
		return 0;
	}
	
	uint8_t txdata[length+2];
	txdata[0]= startadress | 0x40; 	// set write command
	txdata[1]= length -1; 	// 0x00 is 1 byte
	
	for(uint8_t i=0; i<length; i++)	// Dateiinhalt in txdata plazieren
	{
		txdata[i+2]= *(data+i);
	}
	
	
	HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin,0);
	HAL_Delay(.5);

	HAL_SPI_Transmit(&hspi1,txdata,length+2,10);
	
	HAL_Delay(.5);
	HAL_GPIO_WritePin(ADC_CS_GPIO_Port,ADC_CS_Pin,1);
}

void ReadADC(uint8_t startadress, uint8_t length, uint8_t *rxdata) 	// Speichert rückgabe in rxdata
{
	uint8_t txdata[2] = {startadress |0x20 , length-1};	// create array for spi transmit
	
	HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin,0);
	HAL_Delay(.5);

	HAL_SPI_Transmit(&hspi1,txdata, length+1,	  10);
	HAL_SPI_Receive(&hspi1, rxdata, length, 10);

	HAL_Delay(.5);
	HAL_GPIO_WritePin(ADC_CS_GPIO_Port,ADC_CS_Pin,1);
}





uint32_t ReadADCChannel(uint8_t Channel) 	// Positive ADC input, Negativer ist AINCOM
{
	uint8_t txbuf[3]={0,0,0};
	uint8_t rxbuf[5];

	if (Channel>5)	// DS124S06 hat nur 5 Kanäle
		return 0;

	HAL_GPIO_WritePin(ADC_CS_GPIO_Port,ADC_CS_Pin,0);


	while( HAL_GPIO_ReadPin(ADC_DRDY_GPIO_Port, ADC_DRDY_Pin) );


	// Eigentlich Warten bis Fertig Konvertiert !!!!!!!!!


	txbuf[0]= 0x12;	//RDATA => Read Data
	HAL_SPI_Transmit(&hspi1,txbuf,1,10);

	HAL_SPI_Receive(&hspi1, rxbuf,3,10); //Read Datarate Register --> Ohne CRC und Status

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
