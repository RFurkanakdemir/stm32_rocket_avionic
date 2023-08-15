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
  ******************************************************************************
  gps ok

  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
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
#include "bmi160_wrapper.h"
#include <bmp3_wrapper.h>

#include "lora_lib.h"
#include "E32Lora.h"

//Filter libraries
#include "micros.h"

#include "quaternion.h"
#include "FusionAHRS.h"
#include "LowPassFilter.h"
#include "NotchFilter.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
lwgps_t gps; //gps struct
BMI160_t imu_t; //bmı160 struct
BMP390_t bmp_t; //bmp390 struct

dataPaket_t     paket;
Variables_t     variable = { .telemTimer = 0 , .firstinit = TRUE  , .u8_buffer = { 0 } , .u8_counter = 0};
dataStruct_t    data     ;
GcsPaket_t      gcsStructPaket;


Quaternion_t quaternion_t;//quaternion filter struct

FusionBias fusionBiasIMU1;
FusionAhrs fusionAhrsIMU1;
FusionAHRS_t AHRS_IMU1;

NotchFilter_t NF_gyro_x, NF_gyro_y, NF_gyro_z;
LPFTwoPole_t LPF_accel_x, LPF_accel_y, LPF_accel_z, LPF_gyro_x, LPF_gyro_y, LPF_gyro_z;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//for low pass filter
#define 	SAMPLE_FREQ_HZ 		200.0f	// Data sample frequency in Hz
#define 	LPF_ACCEL_CTOFF_HZ 	260.0f	// LPF Cut-Off or accelerometer
#define 	LPF_GYRO_CTOFF_HZ 	256.0f	// LPF Cut-Off or gyro
#define 	NF_GYRO_CFREQ_HZ	74.0f	// NF center frequency for Gyro
#define 	NF_GYRO_NWDTH_HZ	5.0f	// NF notch-width frequency for Gyro



#define CURRENT_NODE ROCKET_NODE	//node information

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
/*
	if(huart==&huart3){

		HAL_UART_Receive(&huart3, &lora_data, 1, 10);
		lora_buffer[lora_index]=lora_data;
		lora_index++;
		HAL_UART_Receive_IT(&huart3, &lora_data, 1);


	}*/
}//gps interruot

uint8_t message[ 3 ]={0xc1, 0xc1, 0xc1 };
uint8_t recv[ 6 ]={0xff,0xff,0xff,0xff,0xff,0xff};

//for filter
float sample_time_sec_f32 = 1.0f / SAMPLE_FREQ_HZ;
float sample_time_us_f32 = (1.0f / SAMPLE_FREQ_HZ) * 1000000.0f;

float accelLowPassFiltered_f32[3], gyroLowPassFiltered_f32[3], gyroNotchFiltered_f32[3];

float eulerAngles_f32[3];

uint64_t timer_u64 = 0;
uint64_t lastTime_u64 = 0;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
 	uint8_t is_fly = 0;
 	uint8_t buz_delay=0;
 	uint8_t status=1;
 	uint8_t first=0;
 	uint8_t second=0;
 	uint8_t status_lora = 0;
 	uint8_t buz_init = 0;
 	uint8_t error=0;
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
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */


  initFusionAHRS(&fusionBiasIMU1, &fusionAhrsIMU1, &AHRS_IMU1, sample_time_sec_f32);

  lwgps_init(&gps);


  while (BMI160_init(&imu_t) != 0 );
  while (BMP390_init(&bmp_t) != 0 );
  HAL_Delay(50);

  //lora lib init
  E32_Init(M0_GPIO_Port, M0_Pin, M1_GPIO_Port, M1_Pin, AUX_GPIO_Port, AUX_Pin, &huart3);
  E32_SetMode(SLEEP_MODE);
  HAL_Delay(50);

  // Lora'nın , var olan configini okuyoruz.*********************
  status_lora = HAL_UART_Transmit(&huart3, message, MESSAGE_LENGTH , 2000);
  status_lora = HAL_UART_Receive(&huart3,recv, RECV_LENTH,2000);//config oku
  HAL_Delay(50);

  //activate lora normal mod
  HAL_GPIO_WritePin(M0_GPIO_Port ,M0_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(M1_GPIO_Port, M1_Pin, GPIO_PIN_RESET);
  HAL_Delay(200);

  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);


  HAL_UART_Receive_IT(&huart1, &gpsrx_data, 1);//uart1den gelen interruptları almak için

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //buzzer reset for x sec
	  if(buz_delay <254){
		  buz_delay++;
	  }

	  if(buz_delay>250 && buz_init ==0){
		  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
		  buz_init=1;
	  }

	  // sensors okey?
	  if (  imu_t.INIT_OK_i8 != TRUE && bmp_t.INIT_OK_i8 != TRUE)
	  	 {
	  	  	//x*******Read Data******
	  	  	error=bmi160ReadAccelGyro(&imu_t);	//gyro verilerini oku (i2c1 kullanır)
	  	  	error=BMP390_get_data(&bmp_t);//basınç oku (i2c2 kullanır)

	  	  	getFusionAHRS_6DoF(&fusionBiasIMU1, &fusionAhrsIMU1, &AHRS_IMU1, imu_t.BMI160_Accel_f32[0], imu_t.BMI160_Accel_f32[1],
		imu_t.BMI160_Accel_f32[2],imu_t.BMI160_Gyro_f32[0],imu_t.BMI160_Gyro_f32[1],imu_t.BMI160_Gyro_f32[2]);

	  	 }
//fly trigger
	  if(bmp_t.altitude>700){
	  	  	  is_fly= 1;
	  	 }

//birincil patlama
	  if ((AHRS_IMU1.PITCH<-55 || AHRS_IMU1.PITCH > 55 || AHRS_IMU1.ROLL<-50 || AHRS_IMU1.ROLL > 50) && is_fly==1){

	  		HAL_GPIO_WritePin(birincil_GPIO_Port,birincil_Pin,GPIO_PIN_SET);
	  		first=1;
	  	}

//ikincil patlama
	  	if(bmp_t.altitude<600 && is_fly == 1 && first ==1){

	  		HAL_GPIO_WritePin(ikincil_GPIO_Port, ikincil_Pin, GPIO_PIN_SET);
	  		second=1;

	  	}



	  if ( variable.firstinit )
	  	{
	  		initDataPaket( &paket,  CURRENT_NODE );
	  		variable.firstinit  = FALSE;
	  		memset( &data , 0  , sizeof( dataStruct_t ) );
	  		variable.telemTimer = HAL_GetTick();
	  	}

	  if(first==1){
		  status=2;
	  }
	  if(second==1){
		  status = 3;
	  }
	  if(first==1 && second==1){
		  status=4;
	  }

	  data.roket_gps_enlem= gps.latitude;
	  data.roket_gps_boylam = gps.longitude;
/*
	  data.roket_gps_enlem = 39.927170 ;
	  data.roket_gps_boylam = 32.835861;
*/
	  data.roket_gps_irtifa = gps.altitude;
	  data.acc_x= imu_t.BMI160_Accel_f32[0];
	  data.acc_y=imu_t.BMI160_Accel_f32[1];
	  data.acc_z=imu_t.BMI160_Accel_f32[2];
	  data.angle=AHRS_IMU1.PITCH;
	  data.gyro_x=imu_t.BMI160_Gyro_f32[0];
	  data.gyro_y=imu_t.BMI160_Gyro_f32[1];
	  data.gyro_z=AHRS_IMU1.PITCH+5;
	  data.irtifa = bmp_t.altitude;
	  data.status=status;



	  if ( HAL_GetTick() - (variable.telemTimer)  >= 800 )
	 	{

		  veriPaketle( &paket,  &data);
	 	  verileriYolla( paket.u8_array , DATA_PKT_LENGTH );
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
