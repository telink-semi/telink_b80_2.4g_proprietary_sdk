
/********************************************************************************************************
 * @file     pm_32k_calib.h
 *
 * @brief    This is the header file for TLSR8258
 *
 * @author	 junwei.lu@telink-semi.com;
 * @date     May 8, 2018
 *
 * @par      Copyright (c) 2018, Telink Semiconductor (Shanghai) Co., Ltd.
 *           All rights reserved.
 *
 *           The information contained herein is confidential property of Telink
 *           Semiconductor (Shanghai) Co., Ltd. and is available under the terms
 *           of Commercial License Agreement between Telink Semiconductor (Shanghai)
 *           Co., Ltd. and the licensee or the terms described here-in. This heading
 *           MUST NOT be removed from this file.
 *
 *           Licensees are granted free, non-transferable use of the information in this
 *           file under Mutual Non-Disclosure Agreement. NO WARRENTY of ANY KIND is provided.
 * @par      History:
 * 			 1.initial release(DEC. 26 2018)
 *
 * @version  A001
 *         
 *******************************************************************************************************/
#ifndef PM_32K_CALIB_H_
#define PM_32K_CALIB_H_

#define DATA_32K_CALIB_NUM    8



typedef struct  pm_clock_drift
{
	unsigned int	ref_tick;
	unsigned int	ref_tick_32k;
	int				offset;
//	int				offset_dc;   //not used now
//	int				offset_cur;  //not used now
	unsigned int	offset_cal_tick;
	int				tc;
	int				rc32;
	int				rc32_wakeup;
	int				rc32_rt;
	int				s0;
	unsigned char	calib;
	unsigned char	ref_no;

} pm_clock_drift_t;
//_attribute_data_retention_ pm_clock_drift_t	pmcd = {0, 0, 0, 0, 0, 0};

unsigned int data_32k_calib[DATA_32K_CALIB_NUM];
extern void pm_32k_freq_track_init(void);
extern void pm_32k_freq_track(void);
extern unsigned int pm_32k_freq_track_get(void);
/**
 * @brief		When 32k rc sleeps, the calibration function is initialized.
 * @return		none.
 */
extern void pm_32k_rc_offset_init(void);
extern void pm_update_32k_rc_sleep_tick (unsigned int tick_32k, unsigned int tick);
extern void pm_cal_32k_rc_offset (int offset_tick);
/**
 * @brief		32k rc calibration clock compensation.
 * @return		32k calibration value after compensation.
 */
extern unsigned int pm_get_32k_rc_calib (void);


#endif



