#include <bmp3_common.h>



#define BMP390_ADDR 0x77<<1
extern I2C_HandleTypeDef hi2c1;
#define I2C_HANDLE	(hi2c1)
#define I2CTIMEOUT 1000



uint8_t GLTXBuffer[256], GLRXBuffer[1024];

void bmp3_delay_us(uint32_t period, void *intf_ptr)
{
   HAL_Delay(period);
}




/*******************************************************************************
* Function Name  : I2C_Read
* Description    : Read data from I2C device registers
* Input          : I2C2 device_address, register address, data, data lenght
* Output         : None
* Return         : None
*******************************************************************************/

BMP3_INTF_RET_TYPE SensorAPI_I2Cx_Read_bmp(uint8_t reg_addr, uint8_t *read_data, uint32_t len, void *intf_ptr)//(uint8_t reg_addr, uint8_t *read_data, uint32_t len, void *intf_ptr)
{
	uint8_t snc;
	snc=HAL_I2C_Master_Transmit(&I2C_HANDLE,BMP390_ADDR , &reg_addr, 1, I2CTIMEOUT);
	snc=HAL_I2C_Master_Receive(&I2C_HANDLE, BMP390_ADDR, read_data, len, I2CTIMEOUT);
    return snc;
}

/*******************************************************************************
* Function Name  : I2C_Write
* Description    : Write data into I2C device registers
* Input          : I2C2 device_address, register address, data, data lenght
* Output         : None
* Return         : None
*******************************************************************************/

BMP3_INTF_RET_TYPE SensorAPI_I2Cx_Write_bmp(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
	GLTXBuffer[0] = reg_addr;
	memcpy(&GLTXBuffer[1], reg_data, len);

	HAL_I2C_Master_Transmit(&I2C_HANDLE, BMP390_ADDR, GLTXBuffer, len+1, I2CTIMEOUT);
	return BMP3_INTF_RET_SUCCESS;
}











