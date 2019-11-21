#ifndef TEMPERATUREANDRH_H_
#define TEMPERATUREANDRH_H_

#include "stm32f1xx.h"



#include "dwt_stm32_delay.h"

/* DEFINES *****************************************************************************/
#define DHT_OK                               0
#define DHT_ERROR_CHECKSUM   -1
#define DHT_ERROR_TIMEOUT    -2

/*
 * Defines the temperature and RH of air
 */
void set_gpio_output (void);
void set_gpio_input (void);
void dht11_reset(void);
uint16_t dht11_scan(void);
uint16_t dht11_read_bit(void);
uint16_t dht11_read_byte(void);
uint16_t dht11_read_data(uint8_t buffer[5]);

#endif /* TEMPERATUREANDRH_H_ */
