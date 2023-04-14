/*
 * DHT11.h
 *
 *  Created on: Apr 9, 2023
 *      Author: nhat.tranminh
 */

#ifndef DHT11_H_
#define DHT11_H_
#include "main.h"
#include "stdio.h"

#define DHT11_PORT 	GPIOB
#define DHT11_PIN 	GPIO_PIN_9

typedef struct{
	uint8_t temperature;
	uint8_t humidity;
}DHT11_Type_t;
void set_pin_output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void set_pin_input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void DHT11_Start(void);
uint8_t DHT11_Check_Response(void);
uint8_t DHT11_Read(void);
void DHT11_GetData(DHT11_Type_t *DHT11_data);

#endif /* DHT11_H_ */
