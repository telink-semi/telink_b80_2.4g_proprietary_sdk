/********************************************************************************************************
 * @file	sdk_version.h
 *
 * @brief	This is the header file for B80m
 *
 * @author	Driver Group
 * @date	2022
 *
 * @par     Copyright (c) 2022, Telink Semiconductor (Shanghai) Co., Ltd. ("TELINK")
 *          All rights reserved.
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

/*
 * Release Tool need to change this macro to match the release version,
 * the replace rules is: "$$$B85m_driver_sdk_"#sdk_version_num"$$$", The "#sdk_version_num"
 * will replace with this macro value.
 */
#define B80_SDK_VERSION_NUM				V3.2.3

#define SDK_VERSION_NUM					B80_SDK_VERSION_NUM


#define	SDK_VERSION1(SDK_VERSION_NUM)	"$$$telink_b80_2.4g_proprietary_sdk_"#SDK_VERSION_NUM"$$$"
#define	SDK_VERSION(SDK_VERSION_NUM)	SDK_VERSION1(SDK_VERSION_NUM)


