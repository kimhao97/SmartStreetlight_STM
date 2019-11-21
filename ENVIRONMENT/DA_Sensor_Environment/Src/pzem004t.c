#include "pzem004t.h"
#include "dwt_stm32_delay.h"
// uint8_t dataRequest[7] = {0xB0, 0xC0, 0xA8, 0x01, 0x01, 0x00, 0x00};
uint8_t crcValue(uint8_t* vData)
{
  uint16_t crc  = 0x0000;
  for(uint8_t i=0; i< 6; i++)
      crc += *(vData + i);
  return (uint8_t)(crc & 0xFF);
}

uint8_t* getArrayPzem(UART_HandleTypeDef *huart, uint8_t pzemReq, uint8_t pzemResp, uint8_t dataSize)
{
  uint8_t dataRequest[7] = {pzemReq, (uint8_t)0xC0, (uint8_t)0xA8, (uint8_t)0x01, (uint8_t)0x01, (uint8_t)0x00}; 
  dataRequest[6] = (uint8_t)crcValue(dataRequest);
  HAL_UART_Transmit(huart, dataRequest, sizeof(dataRequest), 1000);
  // DWT_Delay_us(20);
  // HAL_Delay(1000);
  while(isrPzem);
  isrPzem = 1;
  if ((PzemData[0] == pzemResp) && (PzemData[6] == crcValue(PzemData)))
  {
    return PzemData;
    // for(uint8_t i = 0; i < dataSize; i++)
    // {
    //   *(data + i)= PzemData[i];
    // }
  }
    return (uint8_t*) 0x00;
}

uint8_t dataSend[10];
uint8_t* getPzemData(UART_HandleTypeDef *huart)
{
  uint8_t* dataReceive; 
  dataReceive = getArrayPzem(huart, PZEM_VOLTAGE, RESP_VOLTAGE, sizeof(PzemData));
  dataSend[0] = *(dataReceive + 1);
  dataSend[1] = *(dataReceive + 2);
  dataSend[2] = *(dataReceive + 3);

  dataReceive = getArrayPzem(huart, PZEM_CURRENT, RESP_CURRENT, sizeof(PzemData));
  dataSend[3] = *(dataReceive + 2);
  dataSend[4] = *(dataReceive + 3);

  dataReceive = getArrayPzem(huart, PZEM_POWER, RESP_POWER, sizeof(PzemData));
  dataSend[5] = *(dataReceive + 1);
  dataSend[6] = *(dataReceive + 2); 

  dataReceive = getArrayPzem(huart, PZEM_ENERGY, RESP_ENERGY, sizeof(PzemData));
  dataSend[7] = *(dataReceive + 1);
  dataSend[8] = *(dataReceive + 2);
  dataSend[9] = *(dataReceive + 3);  

  return dataSend;
}

/* .CPP
pzem004t::pzem004t(UART_HandleTypeDef *huart)
{
	*pzemUart = *huart; 
	HAL_UART_Receive_IT(&pzemUart, (uint8_t*) dataReceive , sizeof(dataReceive));
}

pzem004t::~pzem004t()
{

}

float pzem004t::voltage()
{
	uint8_t VOLTAGE[7] = {PZEM_VOLTAGE, 0xC0, 0xA8, 0x01, 0x01, 0x00, 0x1A};
	HAL_UART_Transmit(&pzemUart, VOLTAGE, sizeof(VOLTAGE), PZEM_DEFAULT_READ_TIMEOUT);

} */





