/*
 * ov7675.h
 *
 *  Created on: Jan 9, 2024
 *      Author: jorgemiranda
 */

#ifndef INC_OV7675_H_
#define INC_OV7675_H_

#include "stm32f4xx_hal.h"

#define OV7675_I2C_SLAVE_ADDR 0x42

void ov7675_init(I2C_HandleTypeDef *phi2c);
void ov7675_config();

#endif /* INC_OV7675_H_ */
