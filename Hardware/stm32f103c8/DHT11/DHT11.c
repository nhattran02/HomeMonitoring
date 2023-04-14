/*
 * DHT11.c
 *
 *  Created on: Apr 9, 2023
 *      Author: nhat.tranminh
 */

#include "DHT11.h"
#include "main.h"
#include "stdio.h"
void set_pin_output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	if(GPIOx == GPIOA){
		__HAL_RCC_GPIOA_CLK_ENABLE();
	}else if(GPIOx == GPIOB){
		__HAL_RCC_GPIOB_CLK_ENABLE();
	}else if(GPIOx == GPIOC){
		__HAL_RCC_GPIOC_CLK_ENABLE();
	}
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);

}

void set_pin_input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	if(GPIOx == GPIOA){
		__HAL_RCC_GPIOA_CLK_ENABLE();
	}else if(GPIOx == GPIOB){
		__HAL_RCC_GPIOB_CLK_ENABLE();
	}else if(GPIOx == GPIOC){
		__HAL_RCC_GPIOC_CLK_ENABLE();
	}
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}
void DHT11_Start(void)
{
	set_pin_output(DHT11_PORT, DHT11_PIN);
	HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_RESET);
	HAL_Delay(20);
	HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);
	delay_us(30); //wait for 30us
	set_pin_input(DHT11_PORT, DHT11_PIN);
}
uint8_t DHT11_Check_Response(void)
{
	uint8_t response = 0;
	delay_us(40);
	if(!HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)){
		delay_us(80);
		if(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)){
			response = 1;
		}
	}
	while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));
	return response;
}

uint8_t DHT11_Read(void)
{
	uint8_t i, j;
	for(j=0;j<8;j++){
		while(!HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN));
		delay_us(40);
		if(!HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)){
			i &= ~(1 << (7 - j));
		}else{
			i |=  (1 << (7 - j));
		}
		while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN));
	}
	return i;
}

void DHT11_GetData(DHT11_Type_t *DHT11_data)
{
	uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
	uint16_t SUM;
	DHT11_Start();
	if(DHT11_Check_Response()){
		Rh_byte1 = DHT11_Read();
		Rh_byte2 = DHT11_Read();
		Temp_byte1 = DHT11_Read();
		Temp_byte2 = DHT11_Read();
		SUM = DHT11_Read();
		if(Rh_byte1 + Rh_byte2 + Temp_byte1 + Temp_byte2 == SUM){
			DHT11_data->temperature = (uint8_t)Temp_byte1; //(float)Temp_byte1 + (float)(Temp_byte2/10.0)
			DHT11_data->humidity = (uint8_t)Rh_byte1;      //(float)Rh_byte1 + (float)(Rh_byte2/10.0)
		}
	}
}

