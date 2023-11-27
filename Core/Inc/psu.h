/*
 * psu.h
 *
 *  Created on: Nov 2, 2022
 *      Author: Ivan
 */

#ifndef INC_PSU_H_
#define INC_PSU_H_

#include "main.h"
#include "st7735.h"

#define CHAR_BUFF_SIZE  10

void i_DAC10_Set(uint16_t iDACout);
void v_DAC10_Set(uint16_t vDACout);
void get_adcs(volatile uint16_t adc_RAW[], float *temp_MCU,
		float *outU,float *outI, float constU, float constI, float vdd);
void get_time(RTC_HandleTypeDef hrtc, char* onTd100, char* onTd10, char* onTd1 ,
		char* onTh10, char* onTh1, char* onTm10, char* onTm1, char* onTs10, char* onTs1,
		bool on_off);
char * float_to_char(float x, char *p);
void draw_main_st(COLOR backgr, COLOR front);
void draw_main_dy(char* ptr, char* float_for_LCD, bool on_off, float outU, float outI,
					char onTd100, char onTd10, char onTd1, char onTh10, char onTh1,
					char onTm10, char onTm1, char onTs10, char onTs1, float temp_MCU);

#endif /* INC_PSU_H_ */
