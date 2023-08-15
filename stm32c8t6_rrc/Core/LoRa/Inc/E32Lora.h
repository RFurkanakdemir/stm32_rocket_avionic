/*
 * E32Lora.h
 *
 *  Created on: Jan 1, 2019
 *      Author: kevin
 */

#ifndef INC_E32LORA_H_
#define INC_E32LORA_H_
#include "main.h"

#define NORMAL_MODE 0
#define WAKEUP_MODE 2
#define POWERSAVING_MODE 1
#define SLEEP_MODE 3
#define CONFIG_MODE 7

#define E32_OK 0
#define E32_TIMEOUT 1
#define E32_ERROR 100
#define E32_MESSAGE_TOO_LONG 101
#define E32_INVALID_MODE 102

typedef uint8_t E32_STATUS;

enum uartBaud {
	Baud_1200 = 0,
	Baud_2400 = 1,
	Baud_4800 = 2,
	Baud_9600 = 3,
	Baud_19200 = 4,
	Baud_38400 = 5,
	Baud_57600 = 6,
	Baud_115200 = 7
};

enum uartParity {
	Parity_8N1 = 0,
	Parity_8O1 = 1,
	Parity_8E1 = 2
};

enum airRate {
	AirRate_0300 = 0,
	AirRate_1200 = 1,//
	AirRate_2400 = 2, // OLUMSUZ!
	AirRate_4800 = 3,//
	AirRate_9600 = 4,//
	AirRate_19200 = 5//
};

enum wakeupTime {
	Wakeup_250 = 0,
	Wakeup_500 = 1,
	Wakeup_750 = 2,
	Wakeup_1000 = 3,
	Wakeup_1250 = 4,
	Wakeup_1500 = 5,
	Wakeup_1750 = 6,
	Wakeup_2000 = 7
};

enum txPower {
	TxPwr_20 = 0,
	TxPwr_17 = 1,
	TxPwr_14 = 2,
	TxPwr_10 = 3
};

enum txMode {
	TxMode_Transparent = 0,
	TxMode_Fixed = 1
};

enum ioMode {
	IOMode_Open = 0,
	IOMode_Pull = 1
};

enum fecSwitch {
	FEC_Off = 0,
	FEC_On = 1
};

E32_STATUS E32_Init(GPIO_TypeDef* portM0, uint16_t pinM0, GPIO_TypeDef* portM1, uint16_t pinM1,
		GPIO_TypeDef* portAux, uint16_t pinAux, UART_HandleTypeDef *h);
E32_STATUS E32_SetMode(uint8_t mode);
uint8_t E32_GetMode(void);
E32_STATUS E32_SetConfig(uint8_t *configBuffer);
E32_STATUS E32_GetConfig(uint8_t *configBuffer);
E32_STATUS E32_GetVersion(uint8_t *configBuffer);
E32_STATUS E32_Reset(void);
E32_STATUS E32_SaveParams(void);
E32_STATUS E32_SetParams(void);
E32_STATUS E32_SetAddress(uint16_t addr);
E32_STATUS E32_SetParity(enum uartParity parity);
E32_STATUS E32_SetUartBaud(enum uartBaud baud);
E32_STATUS E32_SetAirRate(enum airRate rate);
E32_STATUS E32_SetChannel(uint8_t channel);
E32_STATUS E32_SetTransmissionMode(enum txMode mode);
E32_STATUS E32_SetIOMode(enum ioMode mode);
E32_STATUS E32_SetWakeTime(enum wakeupTime wake);
E32_STATUS E32_SetFECSwitch(enum fecSwitch fec);
E32_STATUS E32_SetTXPower(enum txPower power);
E32_STATUS E32_SetTargetChannel(uint8_t channel);
E32_STATUS E32_SetTargetAddress(uint8_t addr);
E32_STATUS E32_Transmit(uint8_t *message, uint16_t length);
uint16_t E32_ReceiveData(uint8_t *buffer, uint16_t bufferLength);
uint8_t E32_DataAvailable(void);

void E32_Poll(void);


#endif /* INC_E32LORA_H_ */
