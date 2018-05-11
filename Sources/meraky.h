/*
 * meraky.h
 *
 *  Created on: 8. 5. 2018
 *      Author: ako_2
 */

#ifndef SOURCES_MERAKY_H_
#define SOURCES_MERAKY_H_


void i2c0_event(uint32_t event);
void delay(void);
uint32_t MeasureHumidity(void);		// zmereni vlhkosti
uint32_t MeasureTemperature(void);
void I2C_Init(void);


#endif /* SOURCES_MERAKY_H_ */
