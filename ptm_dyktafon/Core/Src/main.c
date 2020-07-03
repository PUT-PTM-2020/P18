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
#include "STM_MY_LCD16X2.h"
#include "file_manager.h"
#include "recorder.h"

#define CHUNK_SIZE 256
//extern const uint8_t rawAudio[123200];
int16_t data_chunk[CHUNK_SIZE];
extern volatile int data_iterator;
extern const uint32_t SAMPLE_RATE;
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
ADC_HandleTypeDef hadc2;

DAC_HandleTypeDef hdac;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */
uint16_t adc_value;
/*------Zmienne potrzebne do glosnika----------*/
uint32_t sample=0;
double volume=0;
volatile uint8_t diode_state =0;
uint16_t value;
double V=2.95;
volatile int playing = 0;
uint32_t file_size = 0;

/*-----------zmienne potrzebne do nagrywania-------------*/
volatile int recording = 0;
volatile int save_error=0;

/*------------zmienne do ... --------------*/
volatile int16_t x=0;
volatile int y=0;
volatile int z=100;
volatile int selection=-1;
volatile int last_selection=-1;
/*-------------Zmienne potrzebne do zapisu i odczytu na kacie pamieci-------------------------------*/
char buffer[256]; //bufor odczytu i zapisu
static FATFS FatFs; //uchwyt do urządzenia FatFs (dysku, karty SD...)
FRESULT fresult; //do przechowywania wyniku operacji na bibliotece FatFs
static FIL file; //uchwyt do otwartego pliku
char * file_name="0.wav";
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
static void MX_TIM4_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */
/*--------------------Otwarcie pliku do odtwarzania------------------------*/
void OpenFileToRead(char *file_name)
{
	FRESULT fr1;
	fr1=f_open(&file, file_name, FA_READ);
	sample=44;
	file_size=f_size(&file);
	f_lseek(&file, 44);

}
void CloseFileToRead(char *file_name)
{
	f_close(&file);
}





/*---------------------Zapis na karte SD-----------------------*/
void writeSD()
{
	//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, 0);
	//fresult = f_mount(&FatFs, "", 0);
	fresult = f_open(&file, "plik1234.wav", FA_OPEN_ALWAYS | FA_CREATE_ALWAYS | FA_WRITE); //nazwa pliku moze miec maksymalnie 12 znakow z rozszerzeniem
	int len = sprintf( buffer, "Hello PTM!\r\n");
	fresult = f_write(&file, buffer, len, &bytes_written);
	fresult = f_close (&file);
}

/*---------------------Odczyt na karcie SD-----------------------*/
void readSD()
{
	//fresult = f_mount(&FatFs, "", 0);
	fresult = f_open(&file, "WRITE1.txt", FA_READ);
	fresult = f_read(&file, buffer, 16, &bytes_read);
	fresult = f_close(&file);

}
/*------------------------Odczyt jednego fragmentu danych------------*/
int ReadChunk(char* file_path, uint32_t sample)
{
	UINT br;
	f_read(&file, data_chunk, CHUNK_SIZE*2, &br);
	if (CHUNK_SIZE != br) return 1;
	return 0;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef*htim) //2,5,4 timer wykorzystany 3 do diody1
{
	/*--------------------Odczyt z mikrofonu------------------*/
	if(htim->Instance== TIM4)
	{
		if (recording==1)
		{
		HAL_ADC_Start(&hadc1);
			  if(HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
			  {
				  adc_value = HAL_ADC_GetValue(&hadc1);
				  //x = /*(int16_t)*/((2.95/(double)4096) * adc_value);
				  x=adc_value;
				  data_chunk[data_iterator]  = x;
				  data_iterator++;
				  if (data_iterator >= CHUNK_SIZE - 1)
					  {
					  SaveChunk(file_name, data_chunk);
					  data_iterator = 0;
					  // tu jeszcze można policzyć średnią z tego fragmentu danych i na podstawie tego ocenić głośność, i ustawić zmienną sterującą diodą2
					  }
			  }
		}
	}

	/*--------------------Odwarzanie z glosniczka------------------*/
	if(htim->Instance== TIM5)
		{
			if(playing)
			{
				if(sample <= file_size)
				{
					HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, /*(int16_t)*/data_chunk[data_iterator]*volume);
					data_iterator++;
					if (data_iterator >= CHUNK_SIZE - 1)
					{
						sample += 2*CHUNK_SIZE;
						ReadChunk(file_name, sample);
						data_iterator = 0;
					 }
				}
				else
				{
					sample = 0;
					playing = 0;
				}

				//if(sample!=123200-1/*utwor.size()*/)
				//{
				//	HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,rawAudio[(int)sample]/**volume*/);
				//	//HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,rawAudio[sample]*volume);
				//	sample++;
				//}
				//else {sample=0;}
			}
		}

	/*-----Proba ustawienia janosci diody--------------*/
	else if(htim->Instance== TIM2)
			{
					rgb2_set(255);
					TIM2->CCR1=2100-diode_state*200;
					if (diode_state < 5) diode_state++;
					else diode_state = 0;
			}
}

/*--------------------Ustalanie glosnosci------------------*/
void set_volume()
{

	V=2.95;
	  HAL_ADC_Start(&hadc2);
	  if(HAL_ADC_PollForConversion(&hadc2, 10) == HAL_OK)
	  {
		  value = HAL_ADC_GetValue(&hadc2);
		  volume = 0.5 + value/(double)4096;

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
	if (x<0)
	{
		rgb2_set(155);
	}
	else if ((x>=0)&&(x<256))
	{
		rgb2_set(x);
	}
	else
	{
		rgb2_set(255);
	}
}

void select_button(int selection)
{
	if (recording)
	{
		switch(selection)
		{
		//pauza nagrywania
		case 0:
			{
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, RESET);
				rgb1_set(0, 0, 255);//niebieski
				LCD1602_2ndLine();
				LCD1602_print("pause recording");
				recording=0;
				break;
			}
		//zatrzymanie i zapisanie nagrywania
		case 1:
			{
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, RESET);
				rgb1_set(0, 0, 255);//niebieski
				LCD1602_2ndLine();
				LCD1602_print("stop recording");
				recording=0;
				CloseFile(file_name);
			//	AddWaveHeader(file_name);
				break;
			}
		default: {break;}
		}
	}
	else if (playing)
	{
		switch(selection)
		{
		//10 chwil do tyłu
		case 0:
			{
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, SET);
				  if (sample < 10 * SAMPLE_RATE)
					  {
						  sample = 0;
					  }
				  else if (sample==0)
					  {
						  //poprzedni utwor
					  }
				  else
					  {
					  sample -= 10 * SAMPLE_RATE;
					  LCD1602_2ndLine();
					  LCD1602_print("-10");
					  }
				break;
			}
		//10 chwil do przodu
		case 1:
			{
				 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, RESET);
				 if (sample + 10 * SAMPLE_RATE > file_size)
				 {
					 sample=0;
					 playing = 0;
				 }
				 else
				 {
					sample += 10 * SAMPLE_RATE;
					LCD1602_2ndLine();
					LCD1602_print("+10");
				 }
				 break;
			}
		//zatrzymaj
		case 2:
			{
				//podczas zatrzymania
				playing=0;
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, RESET);
				LCD1602_2ndLine();
				LCD1602_print("stop");
				rgb1_set(255, 255, 0); //pomaranczowy
				CloseFileToRead(file_name);
				break;
			}
		//zakończ odtwarzanie
		case 3:
			{
				playing=0;
				break;
			}
		default: {break;}
		}
	}
	else
	{
	switch(selection)
		{
		//0 utwór do tyłu
			case 0:
			{
				if (atoi(file_name)>1)
				{
					file_name = PreviousFile(file_name);
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, SET);
				}
						break;
			}
			//utwór do przodu
			case 1:
			{
			file_name = NextFile(file_name);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, RESET);
			break;
			}
			//odtwórz aktualny utwór
			case 2:
			{
				  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, SET);
				  sample=0;
				  OpenFileToRead(file_name);
				  playing=1;
				  LCD1602_2ndLine();
				  LCD1602_print("start");
				  rgb1_set(0, 255, 0); //zielony
				  break;
			}
			//nagraj nowy utwór
			case 3:
			{
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, SET);
				LCD1602_2ndLine();
				LCD1602_print("recording");
				rgb1_set(255, 0, 0);//czerwony
				file_name = GetNextFileName();
				save_error=AddWaveHeader(file_name); // dodaje  nagłówek
				OpenFile(file_name);
				recording = 1;
				break;
			}
			//kontynuacja odwarzania
			case 4:
			{
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, SET);
				playing=1;
				LCD1602_2ndLine();
				LCD1602_print("start");
				rgb1_set(0, 255, 0); //zielony
				break;

			}
			case 5:
			{
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, SET);
				rgb1_set(255, 0, 0);//czerwony
				LCD1602_2ndLine();
				LCD1602_print("recording");
				recording=1;
				break;
			}
			default: {break;}
		}

	}


}


	 /* switch(selection)
		  	  {
					 case 0: {
						  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, SET);
						  if (sample < 10 * SAMPLE_RATE)
							  {
								  sample = 0;
							  }
						  else if (sample==0)
							  {
								  //poprzedni utwor
							  }
						  else
							  {
							  sample -= 10 * SAMPLE_RATE;
							  LCD1602_2ndLine();
							  LCD1602_print("-10");
							  }

						  break;
							}
					case 1: {
						  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, RESET);
						  sample=sample+0;
						  LCD1602_2ndLine();
						  LCD1602_print("stop");
						  rgb1_set(255, 255, 0); //pomaranczowy
						  break;
							}
					case 2: {
						  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, SET);
						  sample++;
						  LCD1602_2ndLine();
						  LCD1602_print("start");
						  rgb1_set(0, 255, 0); //zielony
						  break;
							}
					case 3: {
						 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, RESET);
						 if (sample + 10 * SAMPLE_RATE > file_size)
						 {
							 sample=0;
							 playing = 0;
						 }
						 else
						 {
							sample += 10 * SAMPLE_RATE;
							LCD1602_2ndLine();
							LCD1602_print("+10");
						 }
						break;
							}
					case 4: {
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, SET);
						LCD1602_2ndLine();
						LCD1602_print("recording");
						rgb1_set(255, 0, 0);//czerwony
						if (recording)
							{
							recording = 0;
							AddWaveHeader(file_name); // nadpisuje nagłówek
							}
						else
						{
							file_name = GetNextFileName();
							save_error=AddWaveHeader(file_name); // dodaje  nagłówek
							recording = 1;
						}
						break;
						}
					case 5: {
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, RESET);
						rgb1_set(0, 0, 255);//niebieski
						LCD1602_2ndLine();
						LCD1602_print("stop recording");
						break;
					}
					case 6: {
						if (atoi(file_name)>0)
						{
							file_name = PreviousFile(file_name);
							fresult = f_close (&file);
							fresult = f_open(&file, file_name, FA_READ);
							HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, SET);
						}
						break;
					}
					case 7: {
						file_name = NextFile(file_name);
						fresult = f_open(&file, file_name, FA_READ);
						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, RESET);
						break;
					}
					default: {HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, RESET);
								break;}

		  	  }
*/
		  /*-----------selection---------------*/
		  /*wybor i przypadek oznacza numer na przyciskach

		    0 - cofniecie o 10 chwil
		    1 - zatrzymanie
		    2 - odtworzenie
		    3 - przesuniecie o 10 chwil
		    4 - nagrywanie
		    5 - zatrzymanie nagrywania
		    6 - wybranie utworu w tyl
		    7 - wybranie utworu w przod

		   */
//}


/*----------Czytanie z przyciskow---------------*/
void read_buttons()
{
		rgb2_set_intensity();
	  	 if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7)==GPIO_PIN_RESET)
	  	 	  	 			  	  	{selection=0;}
	  	 if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)==GPIO_PIN_RESET)
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
	  	 if (selection == last_selection)
	  	 {
	  		 select_button(selection);
	  	 }
	  	 last_selection = selection;
}

void petla()
	{
		//HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,2048);
			 	//writeSD(); //dziala
			 	//readSD(); // dziala
				HAL_Delay(100);
			 	set_volume();
				//LCD1602_1stLine();
		 	 	LCD1602_Begin8BIT(RS_GPIO_Port, RS_Pin, E_Pin, D0_GPIO_Port, D0_Pin, D1_Pin, D2_Pin, D3_Pin, D4_GPIO_Port, D4_Pin, D5_Pin, D6_Pin, D7_Pin);
				//LCD1602_1stLine();
				LCD1602_print("sprawdzam");
				//rgb2_set(255);
			 	read_buttons();
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
  MX_TIM4_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  HAL_DAC_Start(&hdac,DAC_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim5);
  HAL_TIM_Base_Start_IT(&htim4);


  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc2);
  fresult = f_mount(&FatFs, "", 0);
//readSD();
//writeSD();

  	//LCD1602_Begin8BIT(RS_GPIO_Port, RS_Pin, E_Pin, D0_GPIO_Port, D0_Pin, D1_Pin, D2_Pin, D3_Pin, D4_GPIO_Port, D4_Pin, D5_Pin, D6_Pin, D7_Pin);
  	//LCD1602_print("sprzawdzam");
  //rgb2_set(255);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  /** DAC channel OUT2 config 
  */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 49;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 209;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  HAL_GPIO_WritePin(GPIOB, D0_Pin|D1_Pin|D2_Pin|D3_Pin 
                          |GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, D4_Pin|D5_Pin|D6_Pin|D7_Pin 
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, RS_Pin|E_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE4 PE5 
                           PE6 PE7 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : D0_Pin D1_Pin D2_Pin D3_Pin 
                           PB8 */
  GPIO_InitStruct.Pin = D0_Pin|D1_Pin|D2_Pin|D3_Pin 
                          |GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : D4_Pin D5_Pin D6_Pin D7_Pin 
                           PD12 PD13 PD14 PD15 
                           PD7 */
  GPIO_InitStruct.Pin = D4_Pin|D5_Pin|D6_Pin|D7_Pin 
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB7 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RS_Pin E_Pin */
  GPIO_InitStruct.Pin = RS_Pin|E_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

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
