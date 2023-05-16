/*
 * common_porting.c
 *
 *  Created on: Mar 30, 2022
 *      Author: Ibrahim Ozdemir
 *      GitHub: ibrahimcahit
 */

#include "common_porting.h"

extern I2C_HandleTypeDef hi2c2;
#define I2C_HANDLE	(hi2c2)
#define I2CTIMEOUT 1000

uint8_t GTXBuffer[256], GRXBuffer[1024];


void DelayUs(uint32_t Delay)
{
	uint32_t i;

	while(Delay--)
	{
		for(i = 0; i < 84; i++)
		{
			;
		}
	}
}

void bmi160_delay_us(uint32_t period)
{
	uint32_t i;

	while(period--)
	{
		for(i = 0; i < 84; i++)
		{
			;
		}
	}
}

/*******************************************************************************
* Function Name  : I2C_Read
* Description    : Read data from I2C device registers
* Input          : I2C2 device_address, register address, data, data lenght
* Output         : None
* Return         : None
*******************************************************************************/
int8_t SensorAPI_I2Cx_Read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	int8_t tmp;
	tmp=HAL_I2C_Master_Transmit(&I2C_HANDLE, BMI160_ADDR, &reg_addr, 1, I2CTIMEOUT);
	tmp=HAL_I2C_Master_Receive(&I2C_HANDLE, BMI160_ADDR, data, len, I2CTIMEOUT);
	return tmp;
}

/*******************************************************************************
* Function Name  : I2C_Write
* Description    : Write data into I2C device registers
* Input          : I2C2 device_address, register address, data, data lenght
* Output         : None
* Return         : None
*******************************************************************************/
int8_t SensorAPI_I2Cx_Write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	GTXBuffer[0] = reg_addr;
	memcpy(&GTXBuffer[1], data, len);
	int8_t tmp;
	tmp=HAL_I2C_Master_Transmit(&I2C_HANDLE, BMI160_ADDR, GTXBuffer, len+1, I2CTIMEOUT);
	return tmp;
}
