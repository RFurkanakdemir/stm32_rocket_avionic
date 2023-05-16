/*
 * common.h
 *
 *  Created on: Dec 28, 2022
 *      Author: rfrkn
 */

#ifndef INC_BMP3_COMMON_H_
#define INC_BMP3_COMMON_H_



#endif /* INC_BMP3_COMMON_H_ */



#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include <stm32f1xx_hal.h>
#include "main.h"
#include <bmp3.h>





void bmp3_delay_us(uint32_t period, void *intf_ptr);


BMP3_INTF_RET_TYPE SensorAPI_I2Cx_Read_bmp(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
BMP3_INTF_RET_TYPE SensorAPI_I2Cx_Write_bmp(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
