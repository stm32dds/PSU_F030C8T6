/*
 ******************************************************************************
 * @file           : psu.c
 * @brief          : Basic functions to realize linear power supply
 ******************************************************************************
 *  Created on: Nov 2, 2022
 *      Author: Ivan
 */
#include "psu.h"

void i_DAC10_Set(uint16_t iDACout)
{
	// output bit number 9 set
	if (iDACout & 0x0200)
		 HAL_GPIO_WritePin(I_DAC9_GPIO_Port, I_DAC9_Pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(I_DAC9_GPIO_Port, I_DAC9_Pin, GPIO_PIN_RESET);

	// output bit number 8 set
	if (iDACout & 0x0100)
		 HAL_GPIO_WritePin(I_DAC8_GPIO_Port, I_DAC8_Pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(I_DAC8_GPIO_Port, I_DAC8_Pin, GPIO_PIN_RESET);

	// output bit number 7 set
	if (iDACout & 0x0080)
		 HAL_GPIO_WritePin(I_DAC7_GPIO_Port, I_DAC7_Pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(I_DAC7_GPIO_Port, I_DAC7_Pin, GPIO_PIN_RESET);

	// output bit number 6 set
	if (iDACout & 0x0040)
		 HAL_GPIO_WritePin(I_DAC6_GPIO_Port, I_DAC6_Pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(I_DAC6_GPIO_Port, I_DAC6_Pin, GPIO_PIN_RESET);

	// output bit number 5 set
	if (iDACout & 0x0020)
		 HAL_GPIO_WritePin(I_DAC5_GPIO_Port, I_DAC5_Pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(I_DAC5_GPIO_Port, I_DAC5_Pin, GPIO_PIN_RESET);

	// output bit number 4 set
	if (iDACout & 0x0010)
		 HAL_GPIO_WritePin(I_DAC4_GPIO_Port, I_DAC4_Pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(I_DAC4_GPIO_Port, I_DAC4_Pin, GPIO_PIN_RESET);

	// output bit number 3 set
	if (iDACout & 0x0008)
		 HAL_GPIO_WritePin(I_DAC3_GPIO_Port, I_DAC3_Pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(I_DAC3_GPIO_Port, I_DAC3_Pin, GPIO_PIN_RESET);

	// output bit number 2 set
	if (iDACout & 0x0004)
		 HAL_GPIO_WritePin(I_DAC2_GPIO_Port, I_DAC2_Pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(I_DAC2_GPIO_Port, I_DAC2_Pin, GPIO_PIN_RESET);

	// output bit number 1 set
	if (iDACout & 0x0002)
		 HAL_GPIO_WritePin(I_DAC1_GPIO_Port, I_DAC1_Pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(I_DAC1_GPIO_Port, I_DAC1_Pin, GPIO_PIN_RESET);

	// output bit number 0 set
	if (iDACout & 0x0001)
		 HAL_GPIO_WritePin(I_DAC0_GPIO_Port, I_DAC0_Pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(I_DAC0_GPIO_Port, I_DAC0_Pin, GPIO_PIN_RESET);
}

void v_DAC10_Set(uint16_t vDACout)
{
	// output bit number 9 set
	if (vDACout & 0x0200)
		 HAL_GPIO_WritePin(V_DAC9_GPIO_Port, V_DAC9_Pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(V_DAC9_GPIO_Port, V_DAC9_Pin, GPIO_PIN_RESET);

	// output bit number 8 set
	if (vDACout & 0x0100)
		 HAL_GPIO_WritePin(V_DAC8_GPIO_Port, V_DAC8_Pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(V_DAC8_GPIO_Port, V_DAC8_Pin, GPIO_PIN_RESET);

	// output bit number 7 set
	if (vDACout & 0x0080)
		 HAL_GPIO_WritePin(V_DAC7_GPIO_Port, V_DAC7_Pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(V_DAC7_GPIO_Port, V_DAC7_Pin, GPIO_PIN_RESET);

	// output bit number 6 set
	if (vDACout & 0x0040)
		 HAL_GPIO_WritePin(V_DAC6_GPIO_Port, V_DAC6_Pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(V_DAC6_GPIO_Port, V_DAC6_Pin, GPIO_PIN_RESET);

	// output bit number 5 set
	if (vDACout & 0x0020)
		 HAL_GPIO_WritePin(V_DAC5_GPIO_Port, V_DAC5_Pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(V_DAC5_GPIO_Port, V_DAC5_Pin, GPIO_PIN_RESET);

	// output bit number 4 set
	if (vDACout & 0x0010)
		 HAL_GPIO_WritePin(V_DAC4_GPIO_Port, V_DAC4_Pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(V_DAC4_GPIO_Port, V_DAC4_Pin, GPIO_PIN_RESET);

	// output bit number 3 set
	if (vDACout & 0x0008)
		 HAL_GPIO_WritePin(V_DAC3_GPIO_Port, V_DAC3_Pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(V_DAC3_GPIO_Port, V_DAC3_Pin, GPIO_PIN_RESET);

	// output bit number 2 set
	if (vDACout & 0x0004)
		 HAL_GPIO_WritePin(V_DAC2_GPIO_Port, V_DAC2_Pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(V_DAC2_GPIO_Port, V_DAC2_Pin, GPIO_PIN_RESET);

	// output bit number 1 set
	if (vDACout & 0x0002)
		 HAL_GPIO_WritePin(V_DAC1_GPIO_Port, V_DAC1_Pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(V_DAC1_GPIO_Port, V_DAC1_Pin, GPIO_PIN_RESET);

	// output bit number 0 set
	if (vDACout & 0x0001)
		 HAL_GPIO_WritePin(V_DAC0_GPIO_Port, V_DAC0_Pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(V_DAC0_GPIO_Port, V_DAC0_Pin, GPIO_PIN_RESET);
}

void get_adcs(volatile uint16_t adc_RAW[], float *vdd, float *temp_MCU,
		float *outU,float *outI, float constU, float constI)
{
	  *vdd = 3.3 * (*VREFINT_CAL_ADDR) / adc_RAW[3];
	  *temp_MCU = (((int32_t)adc_RAW[2] * (*vdd)/3.3)- (int32_t) *TEMP30_CAL_ADDR );
	  *temp_MCU = *temp_MCU * (int32_t)(110 - 30);
	  *temp_MCU = *temp_MCU / (int32_t)(*TEMP110_CAL_ADDR - *TEMP30_CAL_ADDR);
	  *temp_MCU = *temp_MCU + 30;
	  *outI = constI*(*vdd)*adc_RAW[1]/4095;
	  *outU = constU*(*vdd)*adc_RAW[0]/4095;
}

char * float_to_char(float x, char *p)
{
    char *s = p + CHAR_BUFF_SIZE; // go to end of buffer
    uint16_t decimals;  // variable to store the decimals
    int units;  // variable to store the units (part to left of decimal place)
    if (x < 0) { // take care of negative numbers
        decimals = (int)(x * -100) % 100; // make 1000 for 3 decimals etc.
        units = (int)(-1 * x);
    } else { // positive numbers
        decimals = (int)(x * 100) % 100;
        units = (int)x;
    }

    *--s = (decimals % 10) + '0';
    decimals /= 10; // repeat for as many decimal places as you need
    *--s = (decimals % 10) + '0';
    *--s = '.';

    while (units > 0) {
        *--s = (units % 10) + '0';
        units /= 10;
    }
    if (x < 0) *--s = '-'; // unary minus sign for negative numbers
    return s;
}

void draw_main_st(COLOR backgr, COLOR front)
{
	LCD_Clear(backgr);
	LCD_DrawRectangle (1, 1, 160, 128, front, DRAW_EMPTY, DOT_PIXEL_1X1);
	LCD_DisplayString(1,1,"00.000V",&Font24,backgr,YELLOW);
	LCD_DisplayString(1,25," 0.000A",&Font24,backgr,YELLOW);
	LCD_DisplayString(1,49,"000.00W",&Font24,backgr,YELLOW);
	LCD_DisplayString(123,4,"OFF",&Font16,RED,WHITE);
	LCD_DisplayString(123,23," CV",&Font16,GBLUE,BLACK);
	LCD_DisplayString(123,42," M0",&Font16,CYAN,BLACK);
	LCD_DisplayString(123,61,"  LGT  ",&Font8,MAGENTA,WHITE);
	LCD_DisplayString(5,73,"XX.XXXV",&Font16,backgr,front);
	LCD_DisplayString(89,73,"Y.YYYA",&Font16,backgr,front);
	LCD_DisplayString(18,89," onT:000d00h00m00s",&Font12,backgr,front);
	LCD_DisplayString(18,101,"runT:000d00h00m00s",&Font12,backgr,front);
	LCD_DisplayString(3,114,"MCU temperature",&Font12,backgr,front);
}
