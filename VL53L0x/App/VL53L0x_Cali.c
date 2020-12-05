/*******************************************************************************
 * 	VL53L0x_Cali.c
 *
 *  Created on: 2020年10月26日
 *
 *  Author: ZhaoSir
 *******************************************************************************/
#include "VL53L0x_Cali.h"
#include "Board_Init.h"

#define 			adjust_num 				5									// 校准错误次数

_VL53L0x_Adjust 	Vl53l0x_adjust; 											// 校准数据24c02写缓存区(用于在校准模式校准数据写入24c02)
_VL53L0x_Adjust 	Vl53l0x_data;   											// 校准数据24c02读缓存区（用于系统初始化时向24C02读取数据）



/********************************************************************************
*               VL53L0x_Cali.c
*函数名称：	VL53L0x_Adjust()
*
*函数作用：	校验VL53L0x传感器
*
*参数说明：	dev：传感器设备信息
*
*函数返回：	无
*
*函数作者：	ZhaoSir
********************************************************************************/
VL53L0X_Error VL53L0x_Adjust(VL53L0X_Dev_t *dev)
{
	
	VL53L0X_DeviceError Status = VL53L0X_ERROR_NONE;
	uint32_t refSpadCount = 7;
	uint8_t  isApertureSpads = 0;
	uint32_t XTalkCalDistance = 100;
	uint32_t XTalkCompensationRateMegaCps;
	uint32_t CalDistanceMilliMeter = 100<<16;
	int32_t  OffsetMicroMeter = 30000;
	uint8_t VhvSettings = 23;
	uint8_t PhaseCal = 1;
	uint8_t i=0;

	VL53L0X_StaticInit(dev);														// 数值恢复默认,传感器处于空闲状态

	spads:
	Systick_DelayMs(10);
	
	Status = VL53L0X_PerformRefSpadManagement(dev,&refSpadCount,&isApertureSpads);	// 执行参考Spad管理
	if(Status == VL53L0X_ERROR_NONE)
	{
	    Vl53l0x_adjust.refSpadCount = refSpadCount;
	    Vl53l0x_adjust.isApertureSpads = isApertureSpads;
        i=0;
	}
	else
	{
	    i++;
	    if(i == adjust_num) return Status;
	    goto spads;
	}
	
	// 设备参考校准---------------------------------------------------
	ref:
	Systick_DelayMs(10);
	Status = VL53L0X_PerformRefCalibration(dev,&VhvSettings,&PhaseCal);				//Ref参考校准
	if(Status == VL53L0X_ERROR_NONE)
	{
		Vl53l0x_adjust.VhvSettings = VhvSettings;
		Vl53l0x_adjust.PhaseCal = PhaseCal;
		i=0;
	}
	else
	{
		i++;
		if(i == adjust_num) return Status;
		goto ref;
	}
	
	// 偏移校准------------------------------------------------
	offset:
	Systick_DelayMs(10);
	Status = VL53L0X_PerformOffsetCalibration(dev,CalDistanceMilliMeter,&OffsetMicroMeter);//鍋忕Щ鏍″噯
	if(Status == VL53L0X_ERROR_NONE)
	{
		Vl53l0x_adjust.CalDistanceMilliMeter = CalDistanceMilliMeter;
		Vl53l0x_adjust.OffsetMicroMeter = OffsetMicroMeter;
		i=0;
	}
	else
	{
		i++;
		if(i==adjust_num) return Status;
		goto offset;
	}
	// 串扰校准-----------------------------------------------------
	xtalk:
	Systick_DelayMs(20);
	Status = VL53L0X_PerformXTalkCalibration(dev,XTalkCalDistance,&XTalkCompensationRateMegaCps);//串扰校准
	if(Status == VL53L0X_ERROR_NONE)
	{
		Vl53l0x_adjust.XTalkCalDistance = XTalkCalDistance;
		Vl53l0x_adjust.XTalkCompensationRateMegaCps = XTalkCompensationRateMegaCps;
		i=0;
	}
	else
	{
		i++;
		if(i==adjust_num) return Status;
		goto xtalk;
	}


	Vl53l0x_adjust.adjustok = 0xAA;																//校准成功
//	AT24CXX_Write(0,(uint8_t*)&Vl53l0x_adjust,sizeof(_vl53l0x_adjust));							//将校准数据保存到24c02
	memcpy(&Vl53l0x_data, &Vl53l0x_adjust, sizeof(_VL53L0x_Adjust));
	return Status;
}


