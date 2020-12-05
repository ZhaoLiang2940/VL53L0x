/*******************************************************************************
 * 	VL53L0x_Cali.h
 *
 *  Created on: 2020��10��26��
 *
 *  Author: ZhaoSir
 *******************************************************************************/
#ifndef __VL53L0X_CAIL_H
#define __VL53L0X_CAIL_H

#include "VL53L0x.h"


// VL53L0x������У׼��Ϣ�ṹ�嶨��
typedef  struct
{
	uint8_t  adjustok;                    										// У׼�ɹ���־��0XAA����У׼;������δУ׼
	uint8_t  isApertureSpads;             										// ApertureSpadsֵ
	uint8_t  VhvSettings;                 										// VhvSettingsֵ
	uint8_t  PhaseCal;                    										// PhaseCalֵ
	uint32_t XTalkCalDistance;            										// XTalkCalDistanceֵ
	uint32_t XTalkCompensationRateMegaCps;										// XTalkCompensationRateMegaCpsֵ
	uint32_t CalDistanceMilliMeter;       										// CalDistanceMilliMeterֵ
	int32_t  OffsetMicroMeter;            										// OffsetMicroMeterֵ
	uint32_t refSpadCount;                										// refSpadCountֵ
	
}_VL53L0x_Adjust;

extern 			_VL53L0x_Adjust 		Vl53l0x_data;
extern			VL53L0X_Error 			VL53L0x_Adjust(VL53L0X_Dev_t *dev);


#endif
