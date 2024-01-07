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

void get_adcs(volatile uint16_t adc_RAW[], float *temp_MCU,
		float *outU,float *outI, float constU, float constI, float vdd)
{
	  //calculate internal MCU temperature
	  *temp_MCU = (((int32_t)adc_RAW[2] * vdd/3.3)- (int32_t) *TEMP30_CAL_ADDR );
	  *temp_MCU = *temp_MCU * (int32_t)(110 - 30);
	  *temp_MCU = *temp_MCU / (int32_t)(*TEMP110_CAL_ADDR - *TEMP30_CAL_ADDR);
	  *temp_MCU = *temp_MCU + 30;
	  // store RAW ADC data to calculate averaged values for U and I
	  #define FILTER_DEPTH 45  //max.16 else OVERFLOW, if average method used
	  static uint8_t  filt_cnt;
	  static uint16_t adc_RAW_U[FILTER_DEPTH];
	  static uint16_t adc_RAW_I[FILTER_DEPTH];

	  adc_RAW_U[filt_cnt] = adc_RAW[0];
	  adc_RAW_I[filt_cnt] = adc_RAW[1];
	  filt_cnt++;
	  if(filt_cnt > (FILTER_DEPTH-1)) filt_cnt = 0;
	  //calculate averaged values
	  /*
	  uint16_t  avg_U=0, avg_I=0;
	  for(uint8_t  avg_cnt =0; avg_cnt < FILTER_DEPTH; avg_cnt++)
	  {
		  avg_U = avg_U + adc_RAW_U[avg_cnt];
		  avg_I = avg_I + adc_RAW_I[avg_cnt];
	  }
	  avg_U = avg_U/FILTER_DEPTH;
	  avg_I = avg_I/FILTER_DEPTH;
	  //calculate U&I on output (averaged)
	  *outI = constI*avg_I;
	  *outU = constU*avg_U;
	  */

	  // Method to find most common values
	    uint16_t maxCountU = 0;  // Maximum number of encounters -U
	    uint16_t maxCountI = 0;  // Maximum number of encounters -I
	    uint16_t mostFrequentU = adc_RAW_U[0];  // Most common number - U
	    uint16_t mostFrequentI = adc_RAW_I[0];  // Most common number - I

	    for (uint8_t i = 0; i < FILTER_DEPTH; i++) {
	        uint8_t currentCountU = 1;  // We reset the counter for the current number-U
	        uint8_t currentCountI = 1;  // We reset the counter for the current number-I

	        // We loop through the remaining elements of the array and count
	        //the occurrences of the current number
	        for (uint8_t j = i + 1; j < FILTER_DEPTH; j++) {
	            if (adc_RAW_U[i] == adc_RAW_U[j]) currentCountU++;
	            if (adc_RAW_I[i] == adc_RAW_I[j]) currentCountI++;
	        }

	        //We check if the current number has more occurrences than the maximum for U
	        if (currentCountU > maxCountU) {
	            maxCountU = currentCountU;
	            mostFrequentU = adc_RAW_U[i];
	        }
	        //We check if the current number has more occurrences than the maximum for I
	        if (currentCountI > maxCountI) {
	            maxCountI = currentCountI;
	            mostFrequentI = adc_RAW_I[i];
	        }
	    }
		  *outI = constI*mostFrequentI;
		  *outU = constU*mostFrequentU-(0.18*(*outI));
		  //*outU = constU*mostFrequentU;
}

char * float_to_char(float x, char *p)
{
    char *s = p + CHAR_BUFF_SIZE; // go to end of buffer
    uint16_t decimals;  // variable to store the decimals
    int units;  // variable to store the units (part to left of decimal place)
    if (x < 0) { // take care of negative numbers
        decimals = (int)(x * -1000) % 1000; // make 1000 for 3 decimals etc.
        units = (int)(-1 * x);
    } else { // positive numbers
        decimals = (int)(x * 1000) % 1000;
        units = (int)x;
    }

    *--s = (decimals % 10) + '0';
    decimals /= 10; // repeat for as many decimal places as you need
    *--s = (decimals % 10) + '0';
    decimals /= 10; // I want 3 decimals :-)
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
	ST7735_FillScreen(backgr);
	ST7735_DrawRect(0, 0, 160, 128, front);
	ST7735_DrawString(5,3,"00.000V",Font_16x26,YELLOW, backgr);
	ST7735_DrawString(5,29," 0.000A",Font_16x26,YELLOW, backgr);
	ST7735_DrawString(5,55,"000.00W",Font_16x26,YELLOW, backgr);
	ST7735_DrawString(124,2,"OFF",Font_11x18,WHITE, RED);
	ST7735_DrawString(124,21," SL",Font_11x18,BLACK,GBLUE);
	ST7735_DrawString(124,40," M0",Font_11x18,BLACK,GRAY);
	ST7735_DrawString(124,59,"LGT",Font_11x18,WHITE,MAGENTA);
	ST7735_DrawString(5,84,"05.000V",Font_11x18,front,backgr);
	ST7735_DrawString(89,84,"1.000A",Font_11x18,front, backgr);
	ST7735_DrawString(3,106,"PonTime:000d00h00m00s",Font_7x10,front,backgr);
	ST7735_DrawString(3,117,"MCU temperature",Font_7x10,front,backgr);
	ST7735_DrawString(144,117,"*",Font_7x10,WHITE,BLUE);
	ST7735_DrawString(151,117,"C",Font_7x10,WHITE,BLUE);
 }

void get_time(RTC_HandleTypeDef hrtc, char* onTd100, char* onTd10, char* onTd1 ,
		char* onTh10, char* onTh1, char* onTm10, char* onTm1, char* onTs10, char* onTs1,
		bool on_off)
{
	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef sDate = {0};
	static uint8_t oldHours;
	static uint16_t daysON;
	uint32_t toChar; //temp. value to convert days into chars
	if(!on_off)
		HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
	else
	{
		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
		HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BCD);
		//Process RUN time
		*onTs1 = (sTime.Seconds & 0x0F) +'0';
		*onTs10 = (sTime.Seconds >> 4) + '0';
		*onTm1 = (sTime.Minutes & 0x0F) +'0';
		*onTm10 = (sTime.Minutes >> 4) + '0';
		*onTh1 = (sTime.Hours & 0x0F) +'0';
		*onTh10 = (sTime.Hours >> 4) + '0';
		if(oldHours == 0x23)
			if(sTime.Hours == 0) daysON++;
		toChar = daysON;
		*onTd1 = (toChar % 10) + '0';
		toChar /= 10;
		*onTd10 = (toChar % 10) + '0';
		toChar /= 10;
		*onTd100 = (toChar % 10) + '0';
	}
	oldHours = sTime.Hours;
}

void draw_main_dy(char* ptr, char* float_for_LCD, bool on_off, float outU, float outI,
					char onTd100, char onTd10, char onTd1, char onTh10, char onTh1,
					char onTm10, char onTm1, char onTs10, char onTs1, float temp_MCU)
{
	static float old_outU, old_outI, old_outP;
	float outP = outU*outI;
	static char old_onTd100, old_onTd10, old_onTd1,
				old_onTh10, old_onTh1, old_onTm10, old_onTm1, old_onTs10;
	static bool old_on_off;
	static char old_ptr_0, old_ptr_1;
	if(on_off)
	{
		// Output Voltage
		if (old_outU != outU)
		{
			ptr = float_to_char(outU, float_for_LCD);
			if(outU<1)
			{
				ST7735_DrawString(5,3," 0",Font_16x26,WHITE,BLACK);
				ST7735_DrawString(37,3,ptr,Font_16x26,WHITE,BLACK);
			}
			else
			{
				if(outU<10)
				{
					ST7735_DrawString(5,3," ",Font_16x26,WHITE,BLACK);
					ST7735_DrawString(21,3,ptr,Font_16x26,WHITE,BLACK);
				}
				else ST7735_DrawString(5,3,ptr,Font_16x26,WHITE,BLACK);
			}
			ST7735_DrawString(101, 3,"V",Font_16x26,WHITE,BLACK);
		}

		//Output current
		if(old_outI != outI)
		{
			ptr = float_to_char(outI, float_for_LCD);
			if(outI<1)
			{
				ST7735_DrawString(21,29,"0",Font_16x26,WHITE,BLACK);
				ST7735_DrawString(37,29,ptr,Font_16x26,WHITE,BLACK);
			}
			else ST7735_DrawString(21,29,ptr,Font_16x26,WHITE,BLACK);
			ST7735_DrawString(101,29,"A",Font_16x26,WHITE,BLACK);
		}

		//Output Power
		if(old_outP != outP)
		{
			ptr = float_to_char(outP, float_for_LCD);
			if(outP < 1)
			{
				ptr[3] = 0; //wild truncating
				ST7735_DrawString(5,55,"  0",Font_16x26,WHITE,BLACK);
				ST7735_DrawString(53,55,ptr,Font_16x26,WHITE,BLACK);
			}
			else
			{
				if(outP < 10)
				{
					ptr[4] = 0; //wild truncating
					ST7735_DrawString(5,55,"  ",Font_16x26,WHITE,BLACK);
					ST7735_DrawString(37,55,ptr,Font_16x26,WHITE,BLACK);
				}
				else
				{
					if(outP < 100)
					{
						ptr[5] = 0; //wild truncating
						ST7735_DrawString(5,55," ",Font_16x26,WHITE,BLACK);
						ST7735_DrawString(21,55,ptr,Font_16x26,WHITE,BLACK);
					}
					else
					{
						ptr[5] = 0; //wild truncating
						ST7735_DrawString(5,55,ptr,Font_16x26,WHITE,BLACK);
					}
				}
			}
			ST7735_DrawString(101,55,"W",Font_16x26,WHITE,BLACK);
		}
	}
	else //on_off=0/false
	{
		if(old_on_off != on_off)
		{
			ST7735_DrawString(5,3," 0.000V",Font_16x26,YELLOW,BLACK);
			ST7735_DrawString(5,29," 0.000A",Font_16x26,YELLOW,BLACK);
			ST7735_DrawString(5,55,"  0.00W",Font_16x26,YELLOW,BLACK);
		}
	}
	// Time with powered output
	if(old_onTd100 != onTd100)
		ST7735_DrawChar(59,106,onTd100,Font_7x10,WHITE,BLACK);
	if(old_onTd10 != onTd10)
		ST7735_DrawChar(66,106,onTd10,Font_7x10,WHITE,BLACK);
	if(old_onTd1 != onTd1)
		ST7735_DrawChar(73,106,onTd1,Font_7x10,WHITE,BLACK);
	if(old_onTh10 != onTh10)
		ST7735_DrawChar(87,106,onTh10,Font_7x10,WHITE,BLACK);
	if(old_onTh1 != onTh1)
		ST7735_DrawChar(94,106,onTh1,Font_7x10,WHITE,BLACK);
	if(old_onTm10 != onTm10)
		ST7735_DrawChar(108,106,onTm10,Font_7x10,WHITE,BLACK);
	if(old_onTm1 != onTm1)
		ST7735_DrawChar(115,106,onTm1,Font_7x10,WHITE,BLACK);
	if(old_onTs10 != onTs10)
		ST7735_DrawChar(129,106,onTs10,Font_7x10,WHITE,BLACK);
	ST7735_DrawChar(136,106,onTs1,Font_7x10,WHITE,BLACK);
	//display MCU temperature
	ptr = float_to_char(temp_MCU, float_for_LCD);
	if(old_ptr_0 != ptr[0])
		ST7735_DrawChar(130,117,ptr[0],Font_7x10,WHITE,ORANGE);
	if(old_ptr_1 != ptr[1 ])
		ST7735_DrawChar(137,117,ptr[1],Font_7x10,WHITE,ORANGE);
	old_ptr_0 = ptr[0];
	old_ptr_1 = ptr[1];
	//Create logic to update only changed positions on LCD
	old_outU = outU;
	old_outI = outI;
	old_outP = outP;
	old_onTd100 = onTd100;
	old_onTd10 = onTd10;
	old_onTd1 = onTd1;
	old_onTh10 = onTh10;
	old_onTh1 = onTh1;
	old_onTm10 = onTm10;
	old_onTm1 = onTm1;
	old_onTs10 = onTs10;
	old_on_off = on_off;
}

void save_settings(float scaleU, float scaleI, float scaleUsp, float scaleIsp,
											int8_t mem_sel, float uSP, float iSP)
{
	//	__IO uint32_t data32 = 0 , MemoryProgramStatus = 0;
	uint32_t Address, PageError = 0;
	uint8_t i,j;
	float scaleU_FLASH, scaleI_FLASH, scaleUsp_FLASH, scaleIsp_FLASH;
	float mem_sel_img_FLASH[10][2]; // first is set point for U, second for I
	static FLASH_EraseInitTypeDef EraseInitStruct; // Variable used for Erase procedure

	// Create image of current flash memory
	scaleU_FLASH = *( float *)(SCALE_U_ADDR);
	scaleI_FLASH = *( float *)(SCALE_I_ADDR);
	scaleUsp_FLASH = *( float *)(SCALE_U_SP_ADDR);
	scaleIsp_FLASH = *( float *)(SCALE_I_SP_ADDR);
	Address = MEM_SEL_ARR_ADDR;
	for(i=0; i<10; i++)
	{
		for(j=0;j<2; j++)
		{
			mem_sel_img_FLASH[i][j]=*( float *)(Address);
			Address = Address+4;
		}
	}
	HAL_FLASH_Unlock();
	// Fill EraseInit structure
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = FLASH_USER_START_ADDR;
	EraseInitStruct.NbPages = (FLASH_USER_END_ADDR - FLASH_USER_START_ADDR) / FLASH_PAGE_SIZE;
	//Erase flash page 63
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
	{	//PageError will contain the faulty page and then to know the code error on this page,
		//user can call function 'HAL_FLASH_GetError()'
		while (1)   // Infinite loop
		{
			ST7735_FillScreen(RED);
			HAL_Delay(500);
			ST7735_DrawString(0,52,"Error on FLASH erasing!",Font_11x18,WHITE, RED);
			HAL_Delay(1000);
		}
	}
    //Modify flash image
	if(mem_sel > 9)//selected to modify scale coefficients
	{
		scaleU_FLASH = scaleU; scaleI_FLASH =scaleI;
		scaleUsp_FLASH = scaleUsp; scaleIsp_FLASH = scaleIsp;
	}
	else //selected to modify work memories settings
	{
		mem_sel_img_FLASH[mem_sel][0] = uSP;
		mem_sel_img_FLASH[mem_sel][1] = iSP;
	}
	//Write image to flash memory
	if (HAL_FLASH_Program
			  (FLASH_TYPEPROGRAM_WORD, SCALE_U_ADDR, *(uint32_t*)&scaleU_FLASH) != HAL_OK)
		  	  	  err_flash_writing();
	if (HAL_FLASH_Program
			  (FLASH_TYPEPROGRAM_WORD, SCALE_I_ADDR, *(uint32_t*)&scaleI_FLASH) != HAL_OK)
		  	  	  err_flash_writing();
	if (HAL_FLASH_Program
			  (FLASH_TYPEPROGRAM_WORD, SCALE_U_SP_ADDR, *(uint32_t*)&scaleUsp_FLASH) != HAL_OK)
		  	  	  err_flash_writing();
	if (HAL_FLASH_Program
			  (FLASH_TYPEPROGRAM_WORD, SCALE_I_SP_ADDR, *(uint32_t*)&scaleIsp_FLASH) != HAL_OK)
		  	  	  err_flash_writing();
	Address = MEM_SEL_ARR_ADDR;
	for(i=0; i<10; i++)
	{
		for(j=0;j<2; j++)
		{
			if (HAL_FLASH_Program
					(FLASH_TYPEPROGRAM_WORD, Address,
							*(uint32_t*)&mem_sel_img_FLASH[i][j]) != HAL_OK)
					  	  	  	  	  	  	  	  	  	  	  	  err_flash_writing();
			Address = Address+4;
		}
	}
	HAL_FLASH_Lock();
}

//	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, TEMP1_ADDR, *(uint32_t*)&aT);
//	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, TEMP2_ADDR, *(uint32_t*)&bT);
//	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, TEMP3_ADDR, *(uint32_t*)&cT);
//	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, TEMP4_ADDR, *(uint32_t*)&dT);

//Address = FLASH_USER_START_ADDR;

	//while (Address < FLASH_USER_END_ADDR)
	//{
	//  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, DATA_32) == HAL_OK)
	//  {
	  //    Address = Address + 4;
	// }
	//  else
	//  {
    /* Error occurred while writing data in Flash memory.
       User can add here some code to deal with this error */
	//    while (1)
	//    {
      /* Make LED2 blink (100ms on, 2s off) to indicate error in Write operation */
	//        HAL_Delay(1);
	//    }
	//  }
	//}



//	Temp1 = *( float *)(TEMP1_ADDR);
//	Temp2 = *( float *)(TEMP2_ADDR);
//	Temp3 = *( float *)(TEMP3_ADDR);
//	Temp4 = *( float *)(TEMP4_ADDR);

//Address = FLASH_USER_START_ADDR;
//MemoryProgramStatus = 0x0;

//while (Address < FLASH_USER_END_ADDR)
//{
//  data32 = *(__IO uint32_t *)Address;

//  if (data32 != DATA_32)
//  {
//    MemoryProgramStatus++;
//  }
//  Address = Address + 4;
//}

/*Check if there is an issue to program data*/
//if (MemoryProgramStatus == 0)
//{
  /* No error detected. Switch on LED2*/
//    HAL_Delay(1);
//}
//else
//{
  /* Error detected. LED2 will blink with 1s period */
//  while (1)
//  {
//      HAL_Delay(1);
//  }
//}

void err_flash_writing(void)
{
	while (1)   // Infinite loop
	{
		ST7735_FillScreen(BLUE);
		HAL_Delay(500);
		ST7735_DrawString(0,52,"Error on FLASH writing!",Font_11x18,WHITE, BLUE);
		HAL_Delay(1000);
	}
}
