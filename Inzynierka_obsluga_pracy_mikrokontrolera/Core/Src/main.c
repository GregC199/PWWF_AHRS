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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//ZYROSKOP
#define GYR_REG1_A 0x20
#define GYR_USTAWIENIA 0x0F
#define GYR_MULTIREAD 0x40
#define GYR_READ 0x80
#define GYR_SCZYTANIE_POCZATEK 0x28
#define GYR_WSZYSTKIE_OSIE (GYR_READ | GYR_MULTIREAD | GYR_SCZYTANIE_POCZATEK)

//AKCELEROMETR
#define ACC_USTAWIENIA 0x57
#define ACC_ADRES (0x19 << 1)
#define ACC_CTRL_REG1_A 0x20
#define ACC_CTRL_REG4_A 0x23
#define ACC_SCZYTANIE_POCZATEK 0x28
#define ACC_MULTI_READ 0x80
#define ACC_WSZYSTKIE_OSIE (ACC_MULTI_READ | ACC_SCZYTANIE_POCZATEK)
#define ACC_SET_4G 0x10

//MAGNETOMETR
#define MAG_ADRES 0x3C
#define MAG_CRB_REG 0x01
#define MAG_CRA_REG 0x00
#define MAG_MR_REG 0x02
#define MAG_ZAKRES 0x80
#define MAG_USTAWIENIA 0x00
#define MAG_SCZYTANIE_POCZATEK 0x03
#define MAG_WSZYSTKIE_OSIE (ACC_MULTI_READ | MAG_SCZYTANIE_POCZATEK)
#define MAG_CZESTOTLIWOSC 0x18

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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile int flaga = 0; //zmienna pomocnicza do obslugi przerwan z timera 11

void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef* htim)
{
/*
 * Reimplementacja HAL_TIM_PeriodElapsedCallback w celu uchwycenia
 * przerwania pochodzacego od timera nr 11
 */
	if(htim == &htim11){
		flaga = 1;
	}
}

void Setup_L3GD20(SPI_HandleTypeDef* spi){
/*
 * Komunikacje SPI z L3GD20 rozpoczynamy poprzez ustawienie
 * pinu GYR_SS w stan niski. Nastepnie przesylamy do zyroskopu
 * informacje na temat jego trybu pracy oraz zmieniamy zakres pomiarowy
 * na +-250 stopni. Na koniec zamykamy komunikacje poprzez ustawienie
 * pinu GYR_SS w stan wysoki.
 */
	uint8_t tmp;

	//Rozpoczecie komunikacji spi z zyroskopem
	HAL_GPIO_WritePin(GYR_SS_GPIO_Port, GYR_SS_Pin, GPIO_PIN_RESET);

	//Uruchomienie zyroskopu
	tmp = GYR_REG1_A;
	HAL_SPI_Transmit(spi, &tmp, 1, 100);

	//Zmiana zakresu pomiarow na +-250 stopni
	tmp = GYR_USTAWIENIA;
	HAL_SPI_Transmit(spi, &tmp, 1, 100);

	HAL_GPIO_WritePin(GYR_SS_GPIO_Port, GYR_SS_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
}

void Setup_AKCELEROMETR()
{
/*
 * Uruchomienie akcelerometru w trzech osiach, zmiana czestotliwosci jego dzialania na 100Hz,
 * zmiana zakresu pomiarowego z +-2g na +-4g - jednostek przyspieszenia ziemskiego
 */
	uint8_t tmp = ACC_USTAWIENIA;

	//AKCELEROMETR - aktywacja, 100Hz, osie XYZ
	HAL_I2C_Mem_Write(&hi2c1, ACC_ADRES, ACC_CTRL_REG1_A, 1, &tmp, 1, 100);

	//AKCELEROMETR - zmiana zakresu pomiarowego z +-2g na +-4g
	tmp = ACC_SET_4G;
	HAL_I2C_Mem_Write(&hi2c1, ACC_ADRES, ACC_CTRL_REG4_A, 1, &tmp, 1, 100);

}
void Setup_MAGNETOMETR(){
/*
 * Uruchomienie magnetometru w trybie ciaglej pracy,
 * zmiana czestotliwosci pracy na 75Hz, zmiana zakresu pomiarowego
 * na +-4G - gaus
 */

	uint8_t tmp= MAG_USTAWIENIA;

	//MAGNETOMETR - aktywacja - continous conversion mode
	HAL_I2C_Mem_Write(&hi2c1, MAG_ADRES, MAG_MR_REG, 1, &tmp, 1, 100);

	//MAGNETOMETR - czestotliwosc
	tmp = MAG_CZESTOTLIWOSC;
	HAL_I2C_Mem_Write(&hi2c1, MAG_ADRES, MAG_CRA_REG, 1, &tmp, 1, 100);

	//MAGNETOMETR - zakres pomiarowy +-4
	tmp = MAG_ZAKRES;
	HAL_I2C_Mem_Write(&hi2c1, MAG_ADRES, MAG_CRB_REG, 1, &tmp, 1, 100);
}

void Setup_LSM303DLHC(){
/*
 * Funkcja uruchamiajaca konfiguracje akcelerometru,
 * a nastepnie magnetometru
 */
	HAL_Delay(100);
	Setup_AKCELEROMETR();
	HAL_Delay(100);
	Setup_MAGNETOMETR();
	HAL_Delay(100);

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM11_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  //AKCELEROMETR i MAGNETOMETR - setup
  Setup_LSM303DLHC();


  //ZYROSKOP - setup
  Setup_L3GD20(&hspi1);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  //TIM11 start - timer 50Hz
  HAL_TIM_Base_Start_IT(&htim11);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
