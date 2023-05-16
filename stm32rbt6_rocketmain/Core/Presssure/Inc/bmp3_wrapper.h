/*
 * wrapper.h
 *
 *  Created on: Dec 28, 2022
 *      Author: rfrkn
 */

#ifndef INC_BMP3_WRAPPER_H_
#define INC_BMP3_WRAPPER_H_



#endif /* INC_BMP3_WRAPPER_H_ */

#include <bmp3_common.h>
#include <bmp3.h>
#include <bmp3_defs.h>


typedef struct {

	double temperature;
	double pressure;
	float altitude;

	int8_t INIT_OK_i8;
	int8_t Status_OK;
	uint8_t op_mode;
	uint8_t is_press_en;
	uint8_t is_temp_en;
	uint8_t is_press_os;
	uint8_t is_temp_os;
	uint8_t is_odr;
	int8_t get_pin_conf_OK;


} BMP390_t;


int8_t BMP390_init(BMP390_t *DataStruct);
int8_t BMP390_get_pin_settings(BMP390_t *DataStruct);
int8_t BMP390_get_pin_status(BMP390_t *DataStruct);
int8_t BMP390_get_data(BMP390_t *DataStruct);


