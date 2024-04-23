/********************************************************************************************************
 * @file	main.c
 *
 * @brief	This is the source file for B80
 *
 * @author	2.4G Group
 * @date	2019
 *
 * @par     Copyright (c) 2019, Telink Semiconductor (Shanghai) Co., Ltd. ("TELINK")
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
#include "driver.h"
#include "../epd/epd.h"
#include "../epd/fonts.h"
#include "../epd/gui.h"
#include "string.h"
unsigned char data_buf[4736];

static void user_init(void)
{

}

int main(void)
{
	blc_pm_select_internal_32k_crystal();

	cpu_wakeup_init(EXTERNAL_XTAL_24M);

    wd_32k_stop();

	user_read_flash_value_calib();

    clock_init(SYS_CLK_24M_Crystal);

    user_init();

    while (1)
    {
    	GUI_Clear(data_buf, 1);
        GUI_DispStr(data_buf, 6, 2, " ESL DEMO", 1);
        GUI_DispPic(data_buf, 220, 0, telink_log, 48, 128);
        GUI_DispStr(data_buf, 6, 4, "IEEE ADDR", 1);
        GUI_DispStr(data_buf, 6, 6, "0X", 1);
        unsigned char prompt_str[20];
        unsigned char ieee_addr[]={0,0,0,0};
        GUI_BytesToHexStr(ieee_addr, sizeof(ieee_addr), prompt_str);
        GUI_DispStr((unsigned char*)data_buf, 6+strlen("0X")*GUI_FONT_WIDTH, 6, (char*)prompt_str, 1);
        GUI_DispPic(data_buf, 0, 8, bar_code, 200, 64);
        EPD_Init();
        EPD_Display(data_buf, 4736);
        EPD_Close();
        WaitMs(3000);
    }
    return 0;
}




