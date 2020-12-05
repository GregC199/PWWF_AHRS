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
#include <stdio.h>
#include <limits.h>
#include <math.h>
#include <string.h>
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
int8_t Wiadomosc[200]; // Zmienna pomocnicza sluzaca do przesylania informacji z mikrokontrolera do PC
int16_t Rozmiar; // Zmienna pomocnicza przechowujaca rozmiar przesylanej informacji
uint8_t Dane[6]; // Tablica pomocnicza do obslugi pobierania danych z urzadzen

float X_a = 0; // Zawiera przyspieszenie w osi OX w jednostce g - przyspieszenia ziemskiego
float Y_a = 0; // Zawiera przyspieszenie w osi OY w jednostce g - przyspieszenia ziemskiego
float Z_a = 0; // Zawiera przyspieszenie w osi OZ w jednostce g - przyspieszenia ziemskiego

float X_mem = 0; // Poprzednia wartość przyspieszenia w osi OX
float Y_mem = 0; // Poprzednia wartość przyspieszenia w osi OY
float Z_mem = 0; // Poprzednia wartość przyspieszenia w osi OZ

float X_roznica = 0; // Przechowywuje roznice przyspieszenia w osi OX
float Y_roznica = 0; // Przechowywuje roznice przyspieszenia w osi OY
float Z_roznica = 0; // Przechowywuje roznice przyspieszenia w osi OZ

float X_m = 0; // Zawiera pole magnetyczne w osi OX w jednostce G - gauss
float Y_m = 0; // Zawiera pole magnetyczne w osi OY w jednostce G - gauss
float Z_m = 0; // Zawiera pole magnetyczne w osi OZ w jednostce G - gauss

float X_g = 0; // Zawiera predkosc katowa w osi OX w stopniach
float Y_g = 0; // Zawiera predkosc katowa w osi OY w stopniach
float Z_g = 0; // Zawiera predkosc katowa w osi OZ w stopniach

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
 * pinu GYR_SS w stan niski. Potem przesylamy do zyroskopu
 * informacje na temat rejestru, ktory chcemy edytowac, a nastepnie na temat
 * jego trybu pracy, z zakresem pomiarowy
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

void L3GD20_MultiRead(SPI_HandleTypeDef* spi){
/*
 * Odczytanie danych z wszystkich trzech osi przy pomocy protokolu
 * komunikacyjnego SPI omowionego w ramach funkcji Setup_L3GD20.
 */
	uint8_t tmp;

	//Rozpoczecie komunikacji spi z zyroskopem
	HAL_GPIO_WritePin(GYR_SS_GPIO_Port, GYR_SS_Pin, GPIO_PIN_RESET);

	//Sczytanie wartosci wskazan trzech osi na raz (6 bajtow = 3x(mlodszy i starszy bajt))
	tmp = GYR_WSZYSTKIE_OSIE; //Adres wraz z multireadem
	HAL_SPI_Transmit(spi, &tmp, 1, 100);
	HAL_SPI_Receive(spi, Dane, 6, 100); //Odebranie danych z zyroskopu

	//Zakonczenie komunikacji spi z zyroskopem
	HAL_GPIO_WritePin(GYR_SS_GPIO_Port, GYR_SS_Pin, GPIO_PIN_SET);
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
	  /*
	   * Calosc programu wykonuje sie wtedy gdy nasz TIM11 wywolal przerwanie
	   * oraz zostal wcisniety przycisk B1.
	   * Podwojne wywolanie sprawdzenia wcisniecia przycisku w celu filtracji drgan przycisku
	   */
	   if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_SET) {

		   if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_SET) {

				   if (flaga == 1){
					   //Wywolanie odczytania danych z wszystkich trzech osi dla akcelerometru
					   HAL_I2C_Mem_Read(&hi2c1, ACC_ADRES, ACC_WSZYSTKIE_OSIE, 1, Dane, 6, 100);

					   //Zespolenie bajtu starszego i mlodszego, przetworzenie sczytanych danych
					   X_a = ((float)((int16_t)((Dane[1] << 8) | Dane[0])) * 4.0) / (float) INT16_MAX;
					   Y_a = ((float)((int16_t)((Dane[3] << 8) | Dane[2])) * 4.0) / (float) INT16_MAX;
					   Z_a = ((float)((int16_t)((Dane[5] << 8) | Dane[4])) * 4.0) / (float) INT16_MAX;

					   //Wywolanie odczytania danych z wszystkich trzech osi dla magnetometru
					   HAL_I2C_Mem_Read(&hi2c1, MAG_ADRES, MAG_WSZYSTKIE_OSIE, 1, Dane, 6, 100);

					   //Zespolenie bajtu starszego i mlodszego, przetworzenie sczytanych danych
					   X_m = ((float)((int16_t)((Dane[0] << 8) | Dane[1])) * 4.0)/(float) INT16_MAX;
					   Y_m = ((float)((int16_t)((Dane[2] << 8) | Dane[3])) * 4.0)/(float) INT16_MAX;
					   Z_m = ((float)((int16_t)((Dane[4] << 8) | Dane[5])) * 4.0)/(float) INT16_MAX;


					   //Wywolanie funkcji sczytujacej dane dla wszystkich osi dla zyroskopu
					   L3GD20_MultiRead(&hspi1);

					   //Zespolenie bajtu starszego i mlodszego, przetworzenie sczytanych danych
					   X_g = ((float)((int16_t)((Dane[1] << 8) | Dane[0])) * 250.0)/(float) INT16_MAX;
					   Y_g = ((float)((int16_t)((Dane[3] << 8) | Dane[2])) * 250.0)/(float) INT16_MAX;
					   Z_g = ((float)((int16_t)((Dane[5] << 8) | Dane[4])) * 250.0)/(float) INT16_MAX;

					   /*
						* Zapalenie diod w celu sprawdzenia poprawnosci dzialania programu, kolejno:
						* LD3 - dla osi OX
						* LD4 - dla osi OY
						* LD5 - dla osi OZ
						*/
					   X_roznica = fabs(X_mem - X_a);
					   Y_roznica = fabs(Y_mem - Y_a);
					   Z_roznica = fabs(Z_mem - Z_a);

					   if(X_roznica > 0.1)HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
					   if(Y_roznica > 0.1)HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
					   if(Z_roznica > 0.1)HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);

					   //Odczytanie rozmiaru wiadomosci oraz jej zapis do zmiennej wiadomosci
					   Rozmiar = sprintf((char *)Wiadomosc, "%f %f %f \n", X_g,Y_g,Z_g);

					   //Przeslanie wiadomosci poprzez UART2
					   HAL_UART_Transmit(&huart2, (uint8_t*) Wiadomosc,  Rozmiar, 100);

					   //Odczytanie rozmiaru wiadomosci oraz jej zapis do zmiennej wiadomosci
					   Rozmiar = sprintf((char *)Wiadomosc, "%f %f %f \n", X_a,Y_a,Z_a);

					   //Przeslanie wiadomosci poprzez UART2
					   HAL_UART_Transmit(&huart2, (uint8_t*) Wiadomosc,  Rozmiar, 100);

					   //Odczytanie rozmiaru wiadomosci oraz jej zapis do zmiennej wiadomosci
					   Rozmiar = sprintf((char *)Wiadomosc, "%f %f %f \n", X_m,Y_m,Z_m);

					   //Przeslanie wiadomosci poprzez UART2
					   HAL_UART_Transmit(&huart2, (uint8_t*) Wiadomosc,  Rozmiar, 100);

					   /*
					    * Zapis pomirow w pamieci dla nastepnego powtorzenia petli programu
					    * oraz zresetowanie flagi poprzez wyzerowanie jej wartosci
					    */
					   X_mem = X_a;
					   Y_mem = Y_a;
					   Z_mem = Z_a;

					   flaga = 0;



				   }
		   	   }
		   }
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
