#ifndef __VL53L0X_H
#define __VL53L0X_H

#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
#include "vl53l0x_gen.h"
#include "vl53l0x_cali.h"


//VL53L0X传感器上电默认IIC地址为0X52(不包含最低位)
#define VL53L0X_Addr 0x52


#define		VL53L0X_Xshut(x) 	GPIO_SET_BIT(GPIOB, 4, x)
#define		DelayMS(x) 			Systick_DelayMs(x)

//使能2.8V IO电平模式
#define USE_I2C_2V8  1

//测量模式
#define Default_Mode   0// 默认
#define HIGH_ACCURACY  1//高精度
#define LONG_RANGE     2//长距离
#define HIGH_SPEED     3//高速

//vl53l0x模式配置参数集
typedef  struct
{
	FixPoint1616_t signalLimit;    //Signal极限数值 
	FixPoint1616_t sigmaLimit;     //Sigmal极限数值
	uint32_t timingBudget;         //采样时间周期
	uint8_t preRangeVcselPeriod ;  //VCSEL脉冲周期
	uint8_t finalRangeVcselPeriod ;//VCSEL脉冲周期范围
	
}mode_data;


extern 		mode_data 		Mode_data[];
extern 		uint8_t  		AjustOK;

extern		void            VL53L0x_RESET(VL53L0X_Dev_t *dev);//vl53l0x复位
extern		uint8_t         VL53L0X_Read(uint8_t mode);//获取一次测量距离数据
extern		VL53L0X_Error 	VL53L0X_Init(VL53L0X_Dev_t *dev);

#endif


