/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ff.h"
extern const uint8_t rawAudio[123200];
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
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */
uint16_t adc_value;
/*------Zmienne potrzebne do glosnika----------*/
double sample=0;
double volume=0;
uint16_t value;
double V=2.95;


/*------------zmienne do ... --------------*/
volatile int x=0;
volatile int y=0;
volatile int z=100;
volatile int selection=-1;
/*-------------Zmienne potrzebne do zapisu i odczytu na kacie pamieci-------------------------------*/
char buffer[256]; //bufor odczytu i zapisu
static FATFS FatFs; //uchwyt do urządzenia FatFs (dysku, karty SD...)
FRESULT fresult; //do przechowywania wyniku operacji na bibliotece FatFs
FIL file; //uchwyt do otwartego pliku
WORD bytes_written; //liczba zapisanych byte
WORD bytes_read; //liczba odczytanych byte
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI1_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM5_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/*---------------------Zapis na karte SD-----------------------*/
void writeSD()
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, 0);
	fresult = f_mount(&FatFs, "", 0);
	fresult = f_open(&file, "write.txt", FA_OPEN_ALWAYS | FA_WRITE);
	int len = sprintf( buffer, "Hello PTM!\r\n");
	fresult = f_write(&file, buffer, len, &bytes_written);
	fresult = f_close (&file);
}

/*---------------------Odczyt na karcie SD-----------------------*/
void readSD()
{
	fresult = f_mount(&FatFs, "", 0);
	fresult = f_open(&file, "read.txt", FA_READ);
	fresult = f_read(&file, buffer, 16, &bytes_read);
	fresult = f_close(&file);

}



/*--------------------Odczyt z mikrofonu------------------*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef*htim)
{
	if(htim->Instance== TIM4)
	{
		HAL_ADC_Start(&hadc1);
			  if(HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
			  {
				  adc_value = HAL_ADC_GetValue(&hadc1);
				  x = (2.95/(double)4096) * adc_value;
			  }
	}
}

/*--------------------Odwarzanie z glosniczka------------------*/
void HAL_TIM_PeriodElapsedCallback1(TIM_HandleTypeDef*htim)
{
	if(htim->Instance== TIM5)
	{
		if(sample!=123200-1/*utwor.size()*/)
		{
			HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,rawAudio[(int)sample]);
			//HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,rawAudio[sample]*volume);
			sample++;
		}
		else {sample=0;}

	}
}
/*--------------------Ustalanie glosnosci------------------*/
void set_volume()
{

	V=2.95;
	  HAL_ADC_Start(&hadc1);
	  if(HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
	  {
		  value = HAL_ADC_GetValue(&hadc1);
		  volume=(V/(double)4096)*value*10;


	  }

}

/*-------------------Konfiguracja diody RGB1----------------------------*/
void rgb1_set(uint8_t red, uint8_t green, uint8_t blue)
{
	htim3.Instance->CCR1=red*2000;
	htim3.Instance->CCR2=green*2000;
	htim3.Instance->CCR3=blue*2000;
}

/*-------------------Konfiguracja diody RGB2----------------------------*/

void rgb2_set(uint8_t red)
{
	htim2.Instance->CCR1=red*2000;
}

/*-------Ustalenie jasnosci diody  RGB2 zaleznie od głosnosci lub czestotliwosci
 *  odbieranego dźwięku------*/
//do zrobienia
void rgb2_set_intensity()
{
rgb2_set(255);
}

/*-----Proba ustawienia janosci diody--------------*/
void HAL_TIM_PeriodElapsedCallback_DiodeRGB2(TIM_HandleTypeDef*htim)
{
	if(htim->Instance== TIM2)
		{
		rgb2_set(255);
		y=100;
switch(x)
	{

	case 0:
		TIM2->CCR1=2100;
		x++;
		break;
	case 1:
		TIM2->CCR1=2100-2*y;
		x++;
		break;
	case 2:
		TIM2->CCR1=2100-4*y;
			x++;
			break;
	case 3:
		TIM2->CCR1=2100-6*y;
			x++;
			break;
	case 4:
		TIM2->CCR1=2100-8*y;
			x++;
			break;
	case 5:
		TIM2->CCR1=2100-10*y;
			x=0;
			break;
			}
		}
}

/*----------Czytanie z przyciskow---------------*/
void read_bottoms()
{

	  	 if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0)==GPIO_PIN_RESET)
	  	 	  	 			  	  	{selection=0;}
	  	 if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1)==GPIO_PIN_RESET)
	  	 	  	  	 			  	{selection=1;}
	  	 if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2)==GPIO_PIN_RESET)
	  		  	  	  	  	 		{selection=2;}
	  	 if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3)==GPIO_PIN_RESET)
	  		  	  	 	  	  	 	{selection=3;}
	  	 if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4)==GPIO_PIN_RESET)
	  	  	  	  	  	 			{selection=4;}
	  	 if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5)==GPIO_PIN_RESET)
	  	  	  	 	  	  	 		{selection=5;}
	  	 if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_6)==GPIO_PIN_RESET)
	  	  	  		  	  	  	  	 {selection=6;}
	  	 if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_7)==GPIO_PIN_RESET)
	  	  	  		  	  	 	  	 {selection=7;}

	  switch(selection)
	  	  {				case 0: {
	  		  	  	  	  	  	  	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, SET);
	  		  	  	  	  	  	  	  if ((sample<10)&&(sample>0))
	  		  	  	  	  	  	  	  	  {
	  		  	  	  	  	  	  		  	  sample=0;
	  		  	  	  	  	  	  	  	  }
	  		  	  	  	  	  	  	  else if (sample==0)
	  		  	  	  	  	  	  	  	  {
	  		  	  	  	  	  	  		  	  //poprzedni utwor
									  	  }
	  		  	  	  	  	  	  	  else
	  		  	  	  	  	  	  	  	  {
	  		  	  	  	  	  	  		  sample=sample-10;
	  		  	  	  	  	  	  	  	  }

	  	  	  	  	  	  	  	  	  break;
	  	  	  	  	  	  	  	}
	  	  				case 1: {
	  	  							  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, RESET);
	  	  							  sample=sample+0;
	  	  							  rgb1_set(255, 255, 0); //pomaranczowy
	  	  							  break;
	  	  						}
	  	  				case 2: {
	  	  							  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, SET);
	  	  							  sample++;
	  	  							  rgb1_set(0, 255, 0); //zielony
	  	  							  break;
	  	  						}
	  	  				case 3: {
	  	  							 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, RESET);
	  	  							 if (sample+10>123200/*utwor.size()*/)
	  	  							 {
	  	  								 sample=0;
	  	  							 }
	  	  							 else
	  	  							 {
	  	  								sample=sample+10;
	  	  							 }

	  	  							break;
	  	  						}
	  	  	  	  	  	case 4: { HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, SET);
	  	  	  	  	  			rgb1_set(255, 0, 0);//czerwony
	  	  	  	  	  			break;
	  	  	  	  	  			}
	  	  	  	  	  	case 5: { HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, RESET);
	  	  	  	  	  			rgb1_set(0, 0, 255);//niebieski
	  	  	  	  	  			break;
	  	  	  	  	  			}
	  					case 6: { HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, SET); break;}
	  					case 7: { HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, RESET); break;}
	  					default: {HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, RESET);
	  								break;}

	  	  }

	  /*-----------selection---------------*/
	  /*wybor i przypadek oznacza numer na przyciskach

	    0 - cofniecie o 10 chwil
	    1 - zatrzymanie
	    2 - odtworzenie
	    3 - przesuniecie o 10 chwil
	    4 - nagrywanie i zatrzymanie nagrywania
	    5 - menu do wyboru utworu
	    6 - wybranie utworu w tyl
	    7 - wybranie utworu w przod

	   */


}

void petla()
	{
		//HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,2048);
			 	 writeSD();
			 	 readSD();


			 	  read_bottoms();
	}
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_SPI1_Init();
  MX_DAC_Init();
  MX_TIM5_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim5);
  //HAL_DAC_Start(&hdac,DAC_CHANNEL_1);
  //HAL_ADC_Start(&hadc1);
  //HAL_ADC_Start(&hadc2);

  writeSD();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 set_volume();
	 petla();


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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization 
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config 
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 20999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 49999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 49;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 104;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE4 PE5 
                           PE6 PE7 PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 
                           PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
