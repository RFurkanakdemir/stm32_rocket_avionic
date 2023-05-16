/*
 * wrapper.c
 *
 *  Created on: Dec 28, 2022
 *      Author: rfrkn
 */
#include <bmp3_wrapper.h>
#include <math.h>
struct bmp3_settings settings;
struct bmp3_data bmp390_data;
struct bmp3_dev dev;
struct bmp3_status status;


double sum=0,mean=0;
uint8_t i=0;

int8_t BMP390_init(BMP390_t *DataStruct){
	uint16_t settings_sel;
	int8_t snc;

	uint8_t dev_adress = BMP3_ADDR_I2C_SEC;
	dev.intf_ptr =&dev_adress;	//++

	dev.chip_id = BMP390_CHIP_ID;
	dev.delay_us =bmp3_delay_us;
	dev.intf = BMP3_I2C_INTF; //+
	dev.read = SensorAPI_I2Cx_Read_bmp;
	dev.write = SensorAPI_I2Cx_Write_bmp;

	snc = bmp3_init(&dev);
	HAL_Delay(500);
	if (snc==BMP3_OK)
	{


		settings.int_settings.drdy_en = BMP3_ENABLE;
		settings.press_en = BMP3_ENABLE;
		settings.temp_en = BMP3_ENABLE;

		settings.odr_filter.press_os = BMP3_OVERSAMPLING_2X;
		settings.odr_filter.temp_os = BMP3_OVERSAMPLING_2X;
		settings.odr_filter.odr = BMP3_ODR_100_HZ;
		HAL_Delay(200);
		settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_ODR |
		                   BMP3_SEL_DRDY_EN;
		snc = bmp3_set_sensor_settings(settings_sel, &settings, &dev);
		HAL_Delay(500);

		if (snc==BMP3_OK)
		{
			settings.op_mode = BMP3_MODE_NORMAL;
			snc= bmp3_set_op_mode(&settings, &dev);
			HAL_Delay(5000);

		}
		else
		{
			snc=0x23;//error

		}
	}
	else
	{
		snc=0x21;//error
	}

	DataStruct->INIT_OK_i8 = snc;//if 0 bmp3 ok
	return snc;
}//for function

int8_t BMP390_get_pin_settings(BMP390_t *DataStruct){
	int8_t snc;
	uint8_t ope_mode=2;
	snc=bmp3_get_sensor_settings(&settings, &dev);
	if (BMP3_OK==snc){
		DataStruct->is_press_en=settings.press_en;
		DataStruct->is_temp_en = settings.temp_en;
		DataStruct->is_press_os=settings.odr_filter.press_os;
		DataStruct->is_temp_os=settings.odr_filter.temp_os ;
		DataStruct->is_odr=settings.odr_filter.odr;
		snc=bmp3_get_op_mode(&ope_mode, &dev);
		if (BMP3_OK==snc){

			DataStruct->op_mode=ope_mode;
		}

		else{
			snc=0x26; //error
		}
	}
	else {

		snc = 0x25;//error
	}
	DataStruct->get_pin_conf_OK=snc;
	return snc;
}

int8_t BMP390_get_pin_status(BMP390_t *DataStruct){
	int8_t snc;
	snc = bmp3_get_status(&status, &dev);

	DataStruct->Status_OK=snc;
	return snc;
}


int8_t BMP390_get_data(BMP390_t *DataStruct){
	double pg,h,x,y,z;
	float irt;
	int8_t snc;

	snc=bmp3_get_sensor_data(BMP3_PRESS_TEMP, &bmp390_data, &dev);
	HAL_Delay(5);
	if(i<10){
		//calculate altitude
		pg=bmp390_data.pressure/101325;
		x= ((-8.31432)*(-0.0065))/(9.80665*0.0289644);
		y=((pow(pg,x))-1);
		z= (bmp390_data.temperature+273)/(-0.0065);
		h=z*y;
		
		sum=sum+h;
		i++;
		
		if(i==10){
			mean=sum/10;
			i++;

		}
	}
	//tip dönüşümü hatası gelebilir.
	if(i>=10){
		pg=bmp390_data.pressure/101325;
		x= ((-8.31432)*(-0.0065))/(9.80665*0.0289644);
		y=((pow(pg,x))-1);
		z= (bmp390_data.temperature+273)/(-0.0065);
		h=z*y;
		irt = h-mean;
		DataStruct->altitude=irt;

	}
	
	DataStruct->temperature=bmp390_data.temperature;
	DataStruct->pressure=bmp390_data.pressure;

	return snc;
}

