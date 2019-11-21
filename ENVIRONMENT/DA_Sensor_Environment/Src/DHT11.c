/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "DHT11.h"

/* Global variables --------------------------------------------------------- */
GPIO_TypeDef* GPIOx_temperatureRH;
uint16_t GPIO_Pin_temperatureRH;
GPIO_InitTypeDef  GPIO_InitStruct;

/* Private functions ---------------------------------------------------------*/

void set_gpio_output (void)
{
	/*Configure GPIO pin output: PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void set_gpio_input (void)
{
	/*Configure GPIO pin input: PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
void dht11_reset(void)
{
    // ??DHT11????
    set_gpio_output();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
    DWT_Delay_us(19000);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    DWT_Delay_us(30);
    set_gpio_input();
}

uint16_t dht11_scan(void)
{
  if( HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == RESET) return 0; 
	else return 1;
}

uint16_t dht11_read_bit(void)
{
    while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == RESET);
    DWT_Delay_us(40);
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == SET)
    {
        while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == SET);
        return 1;
    }
    else
    {
        return 0;
    }
}

uint16_t dht11_read_byte(void)
{
    uint16_t i;
    uint16_t data = 0;
    for (i = 0; i < 8; i++)
    {
        data <<= 1;
        data |= dht11_read_bit();
    }
    return data;
}

uint16_t dht11_read_data(uint8_t buffer[5])
{
    uint16_t i = 0;
		uint8_t checksum;
    
    dht11_reset();
    if (dht11_scan() == RESET)
    {
        //???DHT11??
        while (dht11_scan() == RESET);
        while (dht11_scan() == SET);
        for (i = 0; i < 5; i++)
        {
            buffer[i] = dht11_read_byte();
        }
        
        while (dht11_scan() == RESET);
        set_gpio_output();
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
        
        checksum = buffer[0] + buffer[1] + buffer[2] + buffer[3];
        if (checksum != buffer[4])
        {
            // checksum error
            return 1;
        }
    }
    
    return 0;
}
