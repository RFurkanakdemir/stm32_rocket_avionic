/*
 * E32Lora.cpp
 *
 *  Created on: Jan 1, 2019
 *      Author: kevin
 */

#include "main.h"
#include "E32Lora.h"
#include <stdio.h>
#include <string.h>
#include "stm32f1xx.h"

static UART_HandleTypeDef *_huart;
static GPIO_TypeDef* _m0Port;
static uint16_t _m0Pin;
static GPIO_TypeDef* _m1Port;
static uint16_t _m1Pin;
static GPIO_TypeDef* _auxPort;
static uint16_t _auxPin;
static uint32_t _baudRateList[] = {1200,2400,4800,9600,19200,38400,57600,115200};
static uint8_t _targetChannel  ;
static uint16_t _targetAddress ;
static uint8_t _currentConfig[] = {0xc0,0xff,0xff,0x1,0x4,0x44};
volatile uint8_t _dataAvailable = 0;
uint8_t _disableAuxIrq = 0;

static E32_STATUS E32_WaitForAux(uint8_t state)
{

	uint16_t count = 0;
	while(HAL_GPIO_ReadPin(_auxPort, _auxPin) != state)
	{
		if (count++ > 2500)
			return E32_TIMEOUT;

		HAL_Delay(1);
	}

	_dataAvailable = 0;
	return E32_OK;
}

static E32_STATUS E32_ConfigResponse(uint8_t *response, uint8_t responseLength)
{
	E32_STATUS error;
	if ((error = E32_WaitForAux(0)) != E32_OK)
		return error;

	//HAL_GPIO_TogglePin(D2_GPIO_Port, D2_Pin);
	HAL_StatusTypeDef status = HAL_UART_Receive(_huart, response, responseLength, 2000);
	if (status == HAL_TIMEOUT)
		return E32_TIMEOUT;
	else if (status != HAL_OK)
		return status;

	return E32_OK;
}

static E32_STATUS E32_ConfigRequest(uint8_t *request, uint8_t requestLength,
		uint8_t *response, uint8_t responseLength)
{
	E32_STATUS error;

	uint8_t status = HAL_UART_Transmit(_huart, request, requestLength, 2000);

	if (status == HAL_TIMEOUT)
		return E32_TIMEOUT;
	else if (status != HAL_OK)
		return status;

	if(response != NULL)
		if ((error = E32_ConfigResponse(response, responseLength)) != E32_OK)
			return error;

	return E32_OK;
}

static uint32_t E32_GetBaud() {
	return _baudRateList[(_currentConfig[3] & 0x38) >> 3];
}


E32_STATUS E32_Init(GPIO_TypeDef* portM0, uint16_t pinM0, GPIO_TypeDef* portM1, uint16_t pinM1,
		GPIO_TypeDef* portAux, uint16_t pinAux, UART_HandleTypeDef *h)
{
	_huart = h;

	_m0Port = portM0;
	_m0Pin = pinM0;

	_m1Port = portM1;
	_m1Pin = pinM1;

	_auxPort = portAux;
	_auxPin = pinAux;

	return E32_SetMode(NORMAL_MODE);
}

E32_STATUS E32_SetConfig(uint8_t *config) {
	E32_STATUS error;

	uint8_t origMode = E32_GetMode();
	if ((error = E32_SetMode(SLEEP_MODE)) != E32_OK)
		return error;

	if((error=E32_ConfigRequest(config,6,NULL,0))!=E32_OK) {
		return error;
	}

	if ((error = E32_SetMode(origMode)) != E32_OK)
		return error;

	return error;

}
E32_STATUS E32_SetMode(uint8_t mode)
{

	uint8_t prevMode = E32_GetMode();
	if (mode == prevMode)
			return E32_OK;

	_disableAuxIrq = 1;
	if (E32_WaitForAux(1) != E32_OK) {
		return E32_ERROR;
	}

	HAL_GPIO_WritePin(_m0Port, _m0Pin, (mode & 1));
	HAL_GPIO_WritePin(_m1Port, _m1Pin, (mode & 2));

	// Got to delay to catch the falling edge and then wait for rise again
	HAL_Delay(2);
	if (E32_WaitForAux(1) != E32_OK) {
		return E32_ERROR;
	}

	if (mode == SLEEP_MODE)
		_huart->Init.BaudRate = 9600;
	else if (_currentConfig[0] != 0xff)
		_huart->Init.BaudRate = E32_GetBaud();

	HAL_UART_Init(_huart);

	//Wake up needs a 200ms delay before things start to work
	if(prevMode == SLEEP_MODE) {
		HAL_Delay(250);
		_disableAuxIrq=0;
	}
	else if(mode==CONFIG_MODE) {
		uint8_t message[]={0xc1, 0xc1, 0xc1 };
		E32_STATUS error = E32_ConfigRequest(message, 3, _currentConfig, 6);
		if(error != E32_OK)
			return error;
		_disableAuxIrq = 1;
	}
	else {
		HAL_Delay(50);
		_disableAuxIrq = 0;
	}

	return E32_OK;
}

uint8_t E32_GetMode()
{
	return (HAL_GPIO_ReadPin(_m1Port, _m1Pin) << 1) |
			(HAL_GPIO_ReadPin(_m0Port, _m0Pin));
}

E32_STATUS E32_GetConfig(uint8_t *configBuffer)
{
	uint8_t message[]={0xc1, 0xc1, 0xc1 };
	E32_STATUS error;

	uint8_t origMode = E32_GetMode();
	if ((error = E32_SetMode(SLEEP_MODE)) != E32_OK)
		return error;
	error = E32_ConfigRequest(message, 3, configBuffer, 6);
	memcpy(_currentConfig,configBuffer,6);
	if ((error = E32_SetMode(origMode)) != E32_OK)
		return error;

	return E32_OK;
}

E32_STATUS E32_GetVersion(uint8_t *versionBuffer)
{
	uint8_t message[]={0xc3, 0xc3, 0xc3 };
	E32_STATUS error;

	uint8_t origMode = E32_GetMode();
	if ((error = E32_SetMode(SLEEP_MODE)) != E32_OK)
		return error;

	error = E32_ConfigRequest(message, 3, versionBuffer, 6);

	if ((error = E32_SetMode(origMode)) != E32_OK)
		return error;

	return E32_OK;
}

E32_STATUS E32_Reset()
{
	uint8_t message[] = {0xc4, 0xc4, 0xc4};
	E32_STATUS error;

	if ((error = E32_SetMode(SLEEP_MODE)) != E32_OK)
		return error;

	if((error = E32_ConfigRequest(message,3,NULL,0)) != E32_OK)
		return error;

	if ((error=E32_WaitForAux(0)) != E32_OK)
		return error;

	if ((error=E32_WaitForAux(1)) != E32_OK)
		return error;

	if((error = E32_SetMode(NORMAL_MODE)) != E32_OK)
		return error;

	return E32_OK;
}

E32_STATUS E32_SaveParams()
{
	E32_STATUS error;

	if (E32_GetMode() != SLEEP_MODE)
		return E32_INVALID_MODE;

	_currentConfig[0] = '\xc0';

	if((error=E32_ConfigRequest(_currentConfig,6,NULL,0))!=E32_OK)
		return error;

	return E32_OK;
}

E32_STATUS E32_SetParams()
{
	E32_STATUS error;

	if (E32_GetMode() != SLEEP_MODE)
		return E32_INVALID_MODE;

	_currentConfig[0] = '\xc2';

	if((error=E32_ConfigRequest(_currentConfig,6,NULL,0))!=E32_OK)
		return error;

	return E32_OK;
}

E32_STATUS E32_SetAddress(uint16_t addr) {

	if (E32_GetMode() != SLEEP_MODE)
		return E32_INVALID_MODE;

	_currentConfig[1] = (addr & 0xff00) >> 8;
	_currentConfig[2] = addr & 0xff;

	return E32_OK;
}

E32_STATUS E32_SetParity(enum uartParity parity) {

	if (E32_GetMode() != SLEEP_MODE)
		return E32_INVALID_MODE;

	_currentConfig[3] = (_currentConfig[3] & 0x3f) | parity << 6;

	return E32_OK;

}

E32_STATUS E32_SetUartBaud(enum uartBaud baud)
{

	if (E32_GetMode() != SLEEP_MODE) {
		_huart->Init.BaudRate = _baudRateList[baud];
		HAL_UART_Init(_huart);
		return E32_OK;
	}

	_currentConfig[3] = (_currentConfig[3] & 0xc7) | baud << 3;

	return E32_OK;
}

E32_STATUS E32_SetAirRate(enum airRate rate)
{
	if (E32_GetMode() != SLEEP_MODE)
		return E32_INVALID_MODE;

	_currentConfig[3] = (_currentConfig[3] & 0xF8) | rate;

	return E32_OK;
}

E32_STATUS E32_SetChannel(uint8_t channel)
{
	if (E32_GetMode() != SLEEP_MODE)
		return E32_INVALID_MODE;

	_currentConfig[4] = channel & 0x1F;

	return E32_OK;
}

E32_STATUS E32_SetTransmissionMode(enum txMode mode)
{
	if (E32_GetMode() != SLEEP_MODE)
		return E32_INVALID_MODE;

	_currentConfig[5] = (_currentConfig[5] & 0x7F) | mode << 7;


	return E32_OK;
}

E32_STATUS E32_SetIOMode(enum ioMode mode)
{
	if (E32_GetMode() != SLEEP_MODE)
		return E32_INVALID_MODE;

	_currentConfig[5] = (_currentConfig[5] & 0xBF) | mode << 6;


	return E32_OK;
}

E32_STATUS E32_SetWakeTime(enum wakeupTime wake) {

	if (E32_GetMode() != SLEEP_MODE)
		return E32_INVALID_MODE;

	_currentConfig[5] = (_currentConfig[5] & 0xC7) | wake << 3;


	return E32_OK;
}

E32_STATUS E32_SetFECSwitch(enum fecSwitch fec) {
	if (E32_GetMode() != SLEEP_MODE)
		return E32_INVALID_MODE;

	_currentConfig[5] = (_currentConfig[5] & 0xFB) | fec << 2;


	return E32_OK;
}

E32_STATUS E32_SetTXPower(enum txPower power) {
	if (E32_GetMode() != SLEEP_MODE)
		return E32_INVALID_MODE;

	_currentConfig[5] = (_currentConfig[5] & 0xFC) | power;

	return E32_OK;
}

E32_STATUS E32_SetTargetChannel(uint8_t channel) {
	_targetChannel = channel & 0x1F;

	return E32_OK;
}

E32_STATUS E32_SetTargetAddress(uint8_t addr) {
	_targetAddress = addr;

	return E32_OK;
}

E32_STATUS E32_Transmit(uint8_t *message, uint16_t length) {
	if (E32_GetMode() == SLEEP_MODE)
		return E32_INVALID_MODE;

	if (length > 512)
		return E32_MESSAGE_TOO_LONG;

	if (_currentConfig[5] & 0x80) {
		if(length > 509)
			return E32_MESSAGE_TOO_LONG;
		uint8_t header[3 + length];
		header[0] = (_targetAddress & 0xFF) >> 8;
		header[1] = _targetAddress &0xff;
		header[2] = _targetChannel;
		memcpy(&header[3],message,length);
		if(HAL_UART_Transmit(_huart, header, length+3, 2000) != HAL_OK)
			return E32_ERROR;
	}
	else
		if(HAL_UART_Transmit(_huart, message, length, 2000) != HAL_OK)
			return E32_ERROR;

	return E32_OK;
}

uint16_t E32_ReceiveData(uint8_t *buffer, uint16_t bufferLength) {
	if (E32_GetMode() == SLEEP_MODE)
		return E32_INVALID_MODE;

	_dataAvailable = 0;
	uint16_t idx = -1;
	uint8_t status;

	while((status=HAL_UART_Receive(_huart, &buffer[++idx], 1, 100))==HAL_OK) {
		if (idx==bufferLength)
			break;
	}

	return idx - 1;
}

uint8_t E32_DataAvailable(void) {
	return _dataAvailable;
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	if ((GPIO_Pin == AUX_Pin) & (!_disableAuxIrq)) {
		_dataAvailable=1;
	}
}
