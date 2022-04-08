/********************************************************************************************************
 * @file	gui.c
 *
 * @brief	This is the source file for 8355
 *
 * @author	2.4G Group
 * @date	2019
 *
 * @par     Copyright (c) 2019, Telink Semiconductor (Shanghai) Co., Ltd. ("TELINK")
 *          All rights reserved.
 *
 *          Redistribution and use in source and binary forms, with or without
 *          modification, are permitted provided that the following conditions are met:
 *
 *              1. Redistributions of source code must retain the above copyright
 *              notice, this list of conditions and the following disclaimer.
 *
 *              2. Unless for usage inside a TELINK integrated circuit, redistributions
 *              in binary form must reproduce the above copyright notice, this list of
 *              conditions and the following disclaimer in the documentation and/or other
 *              materials provided with the distribution.
 *
 *              3. Neither the name of TELINK, nor the names of its contributors may be
 *              used to endorse or promote products derived from this software without
 *              specific prior written permission.
 *
 *              4. This software, with or without modification, must only be used with a
 *              TELINK integrated circuit. All other usages are subject to written permission
 *              from TELINK and different commercial license may apply.
 *
 *              5. Licensee shall be solely responsible for any claim to the extent arising out of or
 *              relating to such deletion(s), modification(s) or alteration(s).
 *
 *          THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *          ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *          WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *          DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
 *          DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *          (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *          LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *          ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *          (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *          SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************************************/

#include "driver.h"
#include "common.h"
#include "epd.h"
#include "fonts.h"
#include "gui.h"

#define X_channel 16
#define Y_channel 15
#define X_MAX 296
#define Y_MAX 128

#define E_UNIT 8
#define T_UNIT 2

#define GUI_FONT_WIDTH     8
#define GUI_FONT_HEIGHT    16
#define EPD_DATA_SIZE      4736

enum {
    GUI_ERROR_CODE_NO_ERROR = 0,
    GUI_ERROR_CODE_OVERFLOW,
    GUI_ERROR_CODE_NOT_SUPPORT_CHAR,
};

typedef struct {
    const unsigned char *font;
    unsigned char data;
}Font_TypeDef;

Font_TypeDef font_tbl[] = {
    {NUM_0, '0'},
    {NUM_1, '1'},
    {NUM_2, '2'},
    {NUM_3, '3'},
    {NUM_4, '4'},
    {NUM_5, '5'},
    {NUM_6, '6'},
    {NUM_7, '7'},
    {NUM_8, '8'},
    {NUM_9, '9'},
    {CHAR_A, 'A'},
    {CHAR_B, 'B'},
    {CHAR_C, 'C'},
    {CHAR_D, 'D'},
    {CHAR_E, 'E'},
    {CHAR_F, 'F'},
    {CHAR_G, 'G'},
    {CHAR_H, 'H'},
    {CHAR_I, 'I'},
    {CHAR_J, 'J'},
    {CHAR_K, 'K'},
    {CHAR_L, 'L'},
    {CHAR_M, 'M'},
    {CHAR_N, 'N'},
    {CHAR_O, 'O'},
    {CHAR_P, 'P'},
    {CHAR_Q, 'Q'},
    {CHAR_R, 'R'},
    {CHAR_S, 'S'},
    {CHAR_T, 'T'},
    {CHAR_U, 'U'},
    {CHAR_V, 'V'},
    {CHAR_W, 'W'},
    {CHAR_X, 'X'},
    {CHAR_Y, 'Y'},
    {CHAR_Z, 'Z'},
    {CHAR_Ove, '.'},
    {CHAR_Space, ' '},
};

__attribute__((section(".ram_code"))) static unsigned char *GUI_GetFont(unsigned char data)
{
    int i = 0;
    
    for (i = 0; i < sizeof(font_tbl)/sizeof(Font_TypeDef); i++) {
        if (data == font_tbl[i].data) {
            return (unsigned char *)(font_tbl[i].font);
        }
    }

    return NULL;
}

__attribute__((section(".ram_code"))) void GUI_BytesToHexStr(const unsigned char *bytes, int len, unsigned char *str)
{
    unsigned char tmp;
    int i;
    for (i = 0; i < len; i++) {
        tmp = bytes[len-1-i] / 16;
        if (tmp < 10) {
            str[i*2] = '0' + tmp;
        }
        else {
            str[i*2] = 'A' + (tmp - 10);
        }

        tmp = bytes[len-1-i] % 16;
        if (tmp < 10) {
            str[i*2+1] = '0' + tmp;
        }
        else {
            str[i*2+1] = 'A' + (tmp - 10);
        }
    }
    str[2*len] = '\0';
}

__attribute__((section(".ram_code")))void GUI_IntToDecStr(unsigned int data, unsigned char *str)
{
    unsigned char tmp;
    int i = 0;
    while (data) {
        tmp = data % 10;
        data /= 10;
        str[i++] = tmp + '0';
    }
    str[i] = '\0';

    int j = strlen((const char *)str) - 1;
    for (i = 0; i < j; i++, j--) {
        tmp = str[i];
        str[i] = str[j];
        str[j] = tmp;
    }
}

__attribute__((section(".ram_code"))) unsigned char GUI_DispChar(unsigned char *image, int x, int y, unsigned char data)
{
    assert(image);

    int i, j;

    if ((x+GUI_FONT_WIDTH) > X_MAX || (y+GUI_FONT_HEIGHT) > Y_MAX) {
        return GUI_ERROR_CODE_OVERFLOW;
    }

    unsigned char *font = GUI_GetFont(data);
    if (NULL == font) {
        return GUI_ERROR_CODE_NOT_SUPPORT_CHAR;
    }

    for (i = 0; i < E_UNIT; i++) {
        for (j = 0; j < T_UNIT; j++) {
            image[((x+i)*X_channel+Y_channel-y-j)] = font[T_UNIT*i+!j];
        }  
    }

    return GUI_ERROR_CODE_NO_ERROR;
}

__attribute__((section(".ram_code")))unsigned char GUI_DispStr(unsigned char *image, int x, int y, const unsigned char *str, unsigned char dir)
{
    assert(image);

    int len = strlen((const char *)str);

    int i;
    if (dir) { //horizontal 
        if ((len*GUI_FONT_WIDTH+x) > X_MAX) {
            return GUI_ERROR_CODE_OVERFLOW;
        }
        for (i = 0; i < len; i++) {
            GUI_DispChar(image, x+i*8, y, str[i]);
        }
    }
    else {
        if ((len*GUI_FONT_HEIGHT+y) > Y_MAX) {
            return GUI_ERROR_CODE_OVERFLOW;
        }
        for (i = 0; i < len; i++) {
            GUI_DispChar(image, x, y+i*2, str[i]);
        }
    }
    return GUI_ERROR_CODE_NO_ERROR;
}

__attribute__((section(".ram_code")))unsigned char GUI_DispPic(unsigned char *image, int x, int y, const unsigned char *pic, unsigned char width, unsigned char height)
{
    int i, j, bit, n_bit;

    n_bit = (height) / E_UNIT;

    if (y) {
        bit = (height + y) / E_UNIT;
    }
    else {
        bit = height / E_UNIT;
    }

    if (x > X_MAX || y > Y_MAX) {
        return GUI_ERROR_CODE_OVERFLOW;
    }

   
    for (i = 0; i < width; i++) {
        for (j = 0; j < n_bit; j++) {  
            image[((x+i)*X_channel+Y_channel-y-j)] = pic[n_bit*(1+i)-j-1];
        }
    }

    return GUI_ERROR_CODE_NO_ERROR;
}

__attribute__((section(".ram_code")))void GUI_Clear(unsigned char *image, unsigned char colour)
{
    unsigned char data = colour ? 0xff : 0;
    memset(image, data, EPD_DATA_SIZE);
}
