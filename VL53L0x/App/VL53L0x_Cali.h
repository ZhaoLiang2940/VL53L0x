/*******************************************************************************
 * 	VL53L0x_Cali.h
 *
 *  Created on: 2020年10月26日
 *
 *  Author: ZhaoSir
 *******************************************************************************/
#ifndef __VL53L0X_CAIL_H
#define __VL53L0X_CAIL_H

#include "VL53L0x.h"


// VL53L0x传感器校准信息结构体定义
typedef  struct
{
	uint8_t  adjustok;                    										// 校准成功标志，0XAA，已校准;其他，未校准
	uint8_t  isApertureSpads;             										// ApertureSpads值
	uint8_t  VhvSettings;                 										// VhvSettings值
	uint8_t  PhaseCal;                    										// PhaseCal值
	uint32_t XTalkCalDistance;            										// XTalkCalDistance值
	uint32_t XTalkCompensationRateMegaCps;										// XTalkCompensationRateMegaCps值
	uint32_t CalDistanceMilliMeter;       										// CalDistanceMilliMeter值
	int32_t  OffsetMicroMeter;            										// OffsetMicroMeter值
	uint32_t refSpadCount;                										// refSpadCount值
	
}_VL53L0x_Adjust;

extern 			_VL53L0x_Adjust 		Vl53l0x_data;
extern			VL53L0X_Error 			VL53L0x_Adjust(VL53L0X_Dev_t *dev);


#endif
