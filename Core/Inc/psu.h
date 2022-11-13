/*
 * psu.h
 *
 *  Created on: Nov 2, 2022
 *      Author: Ivan
 */

#ifndef INC_PSU_H_
#define INC_PSU_H_

#include "main.h"

#define CHAR_BUFF_SIZE  10

void i_DAC10_Set(uint16_t iDACout);
void v_DAC10_Set(uint16_t vDACout);
void get_adcs(volatile uint16_t adc_RAW[], float *vdd, float *temp_MCU,
		float *outU,float *outI, float constU, float constI);
char * float_to_char(float x, char *p);

#endif /* INC_PSU_H_ */
