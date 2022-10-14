/********************************************************************************************************
 * @file	otp_read.c
 *
 * @brief	This is the source file for B80
 *
 * @author	Driver Group
 * @date	2022
 *
 * @par		Copyright (c) 2022, Telink Semiconductor (Shanghai) Co., Ltd.
 *			All rights reserved.
 *
 *          The information contained herein is confidential property of Telink
 *          Semiconductor (Shanghai) Co., Ltd. and is available under the terms
 *          of Commercial License Agreement between Telink Semiconductor (Shanghai)
 *          Co., Ltd. and the licensee or the terms described here-in. This heading
 *          MUST NOT be removed from this file.
 *
 *          Licensee shall not delete, modify or alter (or permit any third party to delete, modify, or
 *          alter) any information contained herein in whole or in part except as expressly authorized
 *          by Telink semiconductor (shanghai) Co., Ltd. Otherwise, licensee shall be solely responsible
 *          for any claim to the extent arising out of or relating to such deletion(s), modification(s)
 *          or alteration(s).
 *
 *          Licensees are granted free, non-transferable use of the information in this
 *          file under Mutual Non-Disclosure Agreement. NO WARRENTY of ANY KIND is provided.
 *
 *******************************************************************************************************/
#include "otp.h"


/**
 * @brief      This function serves to read data from OTP memory,belong to otp normal read.
 *             otp has three kinds of read mode,in general,just use OTP_READ normal read operation, when the execution of burning operation,
 *             need to use margin read(otp_pgm_margin_read,otp_initial_margin_read),check whether the write is successful.
 * @param[in]  addr - the otp address of the data,it has to be a multiple of 4,the otp memory that can access is from 0x0000-0x3ffc,can't access other address.
 * @param[in]  len  - the length of the data,the unit is word(4 bytes).
 * @param[in]  buff - data buff.
 * @return     none
 *
 * Attention: When the vbat voltage is greater than 3.3V, otp supply is 3.3V, if the vabt voltage is lower than 3.3V,
 * then the otp supply voltage will follow the vbat voltage value, write to otp, according to the datasheet,
 * the voltage value is at least 1.62V, if below the voltage value, you can not operate,and prompt adc sampling voltage has certain error,add by shuaixing.zhai, confirmed by baoyi 20211015.
 */
_attribute_ram_code_sec_noinline_ void otp_read(unsigned int addr, unsigned int word_len, unsigned int *buff)
{
	otp_pce_timeout_exceed();
	otp_read_cycle(OTP_PTM_READ,addr,word_len,buff);
	otp_auto_pce_restore();
}