/********************************************************************************************************
 * @file	app_config.h
 *
 * @brief	This is the header file for B80
 *
 * @author	2.4G Group
 * @date	2024
 *
 * @par     Copyright (c) 2024, Telink Semiconductor (Shanghai) Co., Ltd. ("TELINK")
 *
 *          Licensed under the Apache License, Version 2.0 (the "License");
 *          you may not use this file except in compliance with the License.
 *          You may obtain a copy of the License at
 *
 *              http://www.apache.org/licenses/LICENSE-2.0
 *
 *          Unless required by applicable law or agreed to in writing, software
 *          distributed under the License is distributed on an "AS IS" BASIS,
 *          WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *          See the License for the specific language governing permissions and
 *          limitations under the License.
 *
 *******************************************************************************************************/
#pragma once
#include "driver.h"
/* Enable C linkage for C++ Compilers: */
#if defined(__cplusplus)
extern "C" {
#endif

#if (MCU_CORE_B80)
#define LED1     		        GPIO_PA4
#define LED2     		        GPIO_PA5
#define LED3     		        GPIO_PA6
#define LED4     		        GPIO_PA7
#define TIMER_GPIO      		GPIO_PB3
#elif (MCU_CORE_B80B)
#define LED1                    GPIO_PB3
#define LED2                    GPIO_PB4
#define LED3                    GPIO_PB5
#define LED4                    GPIO_PB6
#define TIMER_GPIO              GPIO_PB0
#endif

#define TIMER_SYS_CLOCK_MODE 	1
#define TIMER_GPIO_TRIGGER_MODE 2
#define TIMER_GPIO_WIDTH_MODE 	3
#define TIMER_TICK_MODE 		4
#define TIMER_WATCHDOG_MODE 	5  /* ONLY TIMER2 SUPPORT THIS MODE*/
#define STIMER_MODE				6
#define TIMER_32K_WATCHDOG_MODE 7

#define TIMER_MODE				1



/* Define system clock */
#define CLOCK_SYS_CLOCK_HZ  	24000000

#if(CLOCK_SYS_CLOCK_HZ==12000000)
	#define SYS_CLK  	SYS_CLK_12M_Crystal
#elif (CLOCK_SYS_CLOCK_HZ==16000000)
	#define SYS_CLK  	SYS_CLK_16M_Crystal
#elif (CLOCK_SYS_CLOCK_HZ==24000000)
	#define SYS_CLK  	SYS_CLK_24M_Crystal
#elif (CLOCK_SYS_CLOCK_HZ==32000000)
	#define SYS_CLK  	SYS_CLK_32M_Crystal
#elif (CLOCK_SYS_CLOCK_HZ==48000000)
	#define SYS_CLK  	SYS_CLK_48M_Crystal
#endif


enum{
	CLOCK_SYS_CLOCK_1S = CLOCK_SYS_CLOCK_HZ,
	CLOCK_SYS_CLOCK_1MS = (CLOCK_SYS_CLOCK_1S / 1000),
	CLOCK_SYS_CLOCK_1US = (CLOCK_SYS_CLOCK_1S / 1000000),
};








/* Disable C linkage for C++ Compilers: */
#if defined(__cplusplus)
}
#endif