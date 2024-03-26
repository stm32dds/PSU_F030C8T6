1. Firmware for stm32PSU board based on STM32F030C8 MCU with 64kB FLASH
2. Used resources:
	   Memory region         Used Size  Region Size  %age Used
  	            RAM:            2528 B         8 KB     30.86%
   	          FLASH:           55508 B        64 KB     84.70%
3. Not implemented from planned features:
	- CV/CI menu on LCD screen not used (you can implement own feature here)
	- IIC Master/Slave interface not present - no enough memory for this
4. For schematics and PCB details look at "https://stm32dds.tk/stm32-psu"
5. Size of Font_16x26 is reduced only to used here symbols. If you want other symbols on this size
   use full LCD driver from previous commits
6. Long press on encoder button cycles ON-OFF power supply (instead of "M" menu)
7. Long press on encoder when "M" menu is selected:
	- if PSU is ON - store Set Points into chosen memory
	- if PSU is OFF - load Set Points from chosen memory
8. Short press cycles trough menus
9. Pressed encoder when switching PSU from OFF to ON activates setup setting
-----------------------------------------------------------------------------------------------------------
Note: In general, I can implement all planed features here but to do that I should to switch from HAL to REG
programing and this by my opinion will close circle of potential peoples who will decide to
polish and improve this code.