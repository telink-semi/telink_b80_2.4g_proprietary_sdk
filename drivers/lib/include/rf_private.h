/********************************************************************************************************
 * @file    rf_private.h
 *
 * @brief   This is the header file for B80
 *
 * @author  2.4G Group
 * @date    2019
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
#ifndef RF_PRIVATE_H_
#define RF_PRIVATE_H_
#include "bsp.h"

/**
 *  @brief  Define RF Tx/Rx/Auto mode
 */

typedef enum {
    PRI_MODE_TX = 0,
    PRI_MODE_RX = 1,
    PRI_MODE_AUTO=2
} PRI_StatusTypeDef;

/**
*	@brief	  	This function serves to judge RF Tx/Rx state.
*	@param[in]	rf_status - Tx/Rx status.
*	@param[in]	rf_channel - RF channel.
*	@return	 	failed -1,else success.
*/
int rf_pri_trx_state_set(PRI_StatusTypeDef rf_status, signed short rf_channel);

/**
*	@brief	  	This function serves to get RF status.
*	@param[in]	none.
*	@return	 	RF Rx/Tx status.
*/
PRI_StatusTypeDef rf_pri_trx_state_get(void);
#endif /* RF_PRIVATE_H_ */
