/*
 * ov7675.c
 *
 *  Created on: Jan 9, 2024
 *      Author: jorgemiranda
 */

#include <stdio.h>
#include <ov7675.h>


I2C_HandleTypeDef *p_hi2c = NULL;

void Error_Handler(void);

uint8_t ov7675_i2c_write_register(uint8_t reg_addr, uint8_t data);
uint8_t ov7675_i2c_read_register(uint8_t reg_addr, uint8_t *data);

void ov7675_init(I2C_HandleTypeDef *phi2c){
	p_hi2c = phi2c;
	ov7675_i2c_write_register(0x12, 0x80);  // RESET
	printf("ov7675_init: reset\n\r");
	HAL_Delay(30);

	uint8_t buffer[4];
	ov7675_i2c_read_register(0x0b, buffer);
	printf("ov7675_init: dev id = 0x%02X\n\r", buffer[0]);
}

void ov7675_config(){
	printf("ov7675_config: \n\r");
}

uint8_t ov7675_i2c_write_register(uint8_t reg_addr, uint8_t data){
	HAL_StatusTypeDef ret;
	ret = HAL_I2C_Mem_Write(p_hi2c, OV7675_I2C_SLAVE_ADDR, reg_addr, 1, &data, 1, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		Error_Handler();
	}
	return 0;
}

uint8_t ov7675_i2c_read_register(uint8_t reg_addr, uint8_t *data){
	HAL_StatusTypeDef ret;
	//ret = HAL_I2C_Mem_Read(p_hi2c, OV7675_I2C_SLAVE_ADDR, reg_addr, 1, (unsigned char*)data, 1, HAL_MAX_DELAY);
	ret = HAL_I2C_Master_Transmit(p_hi2c, OV7675_I2C_SLAVE_ADDR, &reg_addr, 1, HAL_MAX_DELAY);
	ret |= HAL_I2C_Master_Receive(p_hi2c, OV7675_I2C_SLAVE_ADDR, data, 1, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		Error_Handler();
	}
	return 0;
}
