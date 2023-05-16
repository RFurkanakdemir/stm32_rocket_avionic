/*
 * common_porting.h
 *
 *  Created on: Mar 30, 2022
 *      Author: Ibrahim Ozdemir
 *      GitHub: ibrahimcahit
 */

#ifndef COMMON_PORTING_H_
#define COMMON_PORTING_H_

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include "stm32f1xx_hal.h"
#include "main.h"



#define BMI160_ADDR 0x69<<1
#define BUS_TIMEOUT 1000



void DelayUs(uint32_t Delay);
void bmi160_delay_us(uint32_t period);


int8_t SensorAPI_I2Cx_Read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t SensorAPI_I2Cx_Write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);

#endif /* COMMON_PORTING_H_ */

