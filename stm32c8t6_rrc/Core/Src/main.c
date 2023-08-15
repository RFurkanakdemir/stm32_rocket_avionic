/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//standart c libraries
#include "stdint.h"
#include "stdio.h"
#include "math.h"
#include "string.h"

//Sensor libraries
#include "lwgps.h"

#include "lora_lib.h"
#include "E32Lora.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
lwgps_t gps; //gps struct
//rrc_t rrc

dataPaket_t     paket;
Variables_t     variable = { .telemTimer = 0 , .firstinit = 1  , .u8_buffer = { 0 } , .u8_counter = 0};
dataStruct_t    data     ;
GcsPaket_t      gcsStructPaket;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CURRENT_NODE ROCKET_NODE_2	//node information

#define MESSAGE_LENGTH  sizeof(message)
#define RECV_LENTH		sizeof(recv)
#define DATA_PKT_LENGTH sizeof( dataPaket_t )

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
uint8_t gpsrx_buffer[256];
uint8_t gpsrx_index = 0;
uint8_t gpsrx_data = 0;

uint8_t lora_buffer[256];
uint8_t lora_index = 0;
uint8_t lora_data = 0;

uint8_t rrc_buffer[256];
uint8_t rrc_index = 0;
uint8_t rrc_data=0;


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)	//interrupt ile tetiklenerek gps verilerini alır
{
	if(huart == &huart1) //gpsin bağlı olduğu uart kanalı
	{
		if(gpsrx_data != '\n' && gpsrx_index < sizeof(gpsrx_buffer)) {
			gpsrx_buffer[gpsrx_index++] = gpsrx_data; //buffera data aktarılır
		} else {
			lwgps_process(&gps, gpsrx_buffer, gpsrx_index+1);//data parser
			gpsrx_index = 0;
			gpsrx_data = 0;//gelen data tekrar sıfıra eşitlenir
		}
		HAL_UART_Receive_IT(&huart1, &gpsrx_data, 1);
	}

	if(huart == &huart3) //RRCnin bağlı olduğu uart kanalı
		{
			if(rrc_data != '\r' && rrc_index < sizeof(rrc_buffer)) {
				rrc_buffer[rrc_index++] = rrc_data; //buffera data aktarılır
			} else {
				//lwgps_process(&rrc, rrc_buffer, rrc_index+1);//data parser
				rrc_index = 0;
				rrc_data = 0;//gelen data tekrar sıfıra eşitlenir
				for(uint8_t i=0 ; i<50 ; i++){

					rrc_buffer[i]=0;
				}
			}
			HAL_UART_Receive_IT(&huart3, &rrc_data, 1);
		}





}

uint8_t message[ 3 ]={0xc1, 0xc1, 0xc1 };
uint8_t recv[ 6 ]={0xff,0xff,0xff,0xff,0xff,0xff};

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
uint8_t lrdly;
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  lwgps_init(&gps);
  //lora lib init
  E32_Init(M0_GPIO_Port, M0_Pin, M1_GPIO_Port, M1_Pin, AUX_GPIO_Port, AUX_Pin, &huart2);
  E32_SetMode(SLEEP_MODE);
  HAL_Delay(50);

  // Lora'nın , var olan configini okuyoruz.*********************
  uint8_t status = HAL_UART_Transmit(&huart2, message, MESSAGE_LENGTH , 2000);
  status = HAL_UART_Receive(&huart2,recv, RECV_LENTH,2000);//config oku
  HAL_Delay(50);

  //activate lora normal mod
  HAL_GPIO_WritePin(M0_GPIO_Port ,M0_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(M1_GPIO_Port, M1_Pin, GPIO_PIN_RESET);
  HAL_Delay(200);

  HAL_UART_Receive_IT(&huart1, &gpsrx_data, 1);//uart1den gelen interruptları almak için


  HAL_UART_Receive_IT(&huart3, &rrc_data, 1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if ( variable.firstinit )
	  	  	{
	  	  		initDataPaket( &paket,  CURRENT_NODE );
	  	  		variable.firstinit  = 0;
	  	  		memset( &data , 0  , sizeof( dataStruct_t ) );
	  	  		variable.telemTimer = HAL_GetTick();


	  	  	}


	  	  data.roket_gps_enlem= gps.latitude;
	  	  data.roket_gps_boylam = gps.longitude;
	  /*
	  	  data.roket_gps_enlem = 39.927170 ;
	  	  data.roket_gps_boylam = 32.835861;
	  */
	  	  data.roket_gps_irtifa = gps.altitude;
	  	  data.acc_x+= 1;
	  	  data.acc_y+=2;
	  	  data.acc_z+=3;
	  	  data.angle+=4;
	  	  data.gyro_x+=5;
	  	  data.gyro_y+=6;
	  	  data.gyro_z+=7;
	  	  data.irtifa = gps.altitude;
	  	  data.status=+2;


	  if ( HAL_GetTick() - (variable.telemTimer)  >= 3000 )
	  	{

		  veriPaketle( &paket,  &data);
		  verileriYolla( paket.u8_array , DATA_PKT_LENGTH );
		  lrdly++;
		  variable.telemTimer = HAL_GetTick();

	  	}





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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
