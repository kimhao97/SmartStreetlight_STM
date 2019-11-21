#ifndef PZEM004T_H
#define PZEM004T_H

#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"

#define PZEM_VOLTAGE (uint8_t)0xB0
#define RESP_VOLTAGE (uint8_t)0xA0

#define PZEM_CURRENT (uint8_t)0xB1
#define RESP_CURRENT (uint8_t)0xA1

#define PZEM_POWER   (uint8_t)0xB2
#define RESP_POWER   (uint8_t)0xA2

#define PZEM_ENERGY  (uint8_t)0xB3
#define RESP_ENERGY  (uint8_t)0xA3

#define PZEM_SET_ADDRESS (uint8_t)0xB4
#define RESP_SET_ADDRESS (uint8_t)0xA4

#define PZEM_POWER_ALARM (uint8_t)0xB5
#define RESP_POWER_ALARM (uint8_t)0xA5

#define PZEM_DEFAULT_READ_TIMEOUT 1000
//const uint8_t* PZEM_ERROR_VALUE = {0x00};

#define SIZE_VOLTAGE_PZEM 3
#define SIZE_CURRENT_PZEM 2
#define SIZE_POWER_PZEM 2
#define SIZE_ENERGY_PZEM 3

extern uint8_t PzemData[7];
extern volatile uint8_t isrPzem; 

uint8_t crcValue(uint8_t* vData);
uint8_t* getArrayPzem(UART_HandleTypeDef *huart, uint8_t pzemReq, uint8_t pzemResp, uint8_t dataSize);
uint8_t* getPzemData(UART_HandleTypeDef *huart);
// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
// 	if(huart->Instance == USART3)
// 	  {
// 	  HAL_UART_Receive_IT(&huart3, (uint8_t*) PzemData , sizeof(PzemData));
// 	  }
// }
/* .CPP
class pzem004t
{
public:
	pzem004t(UART_HandleTypeDef *huart);
	~pzem004t();
	float voltage();
	float current();
	float power();
	float energy();

	uint8_t* getArrayVoltage();
	uint8_t* getArrayCurrent();
	uint8_t* getArrayPower();
	uint8_t* getArrayEnergy();

private:
	uint8_t dataReceive[7];
	uint8_t crc();
	UART_HandleTypeDef *pzemUart;
}; */

	// float voltage();
	// float current();
	// float power();
	// float energy();

	// uint8_t* getArrayVoltage(UART_HandleTypeDef *pzemUart, uint8_t dataReceive[7]);
	// uint8_t* getArrayCurrent(UART_HandleTypeDef *pzemUart, uint8_t dataReceive[7]);
	// uint8_t* getArrayPower(UART_HandleTypeDef *pzemUart, uint8_t dataReceive[7]);
	// uint8_t* getArrayEnergy(UART_HandleTypeDef *pzemUart, uint8_t dataReceive[7]);

	// uint8_t* getPzemData();

#endif /*PZEM004T*/

