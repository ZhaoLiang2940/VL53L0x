/*******************************************************************************
 * DeviceAction.h
 *
 *  Created on: 2020年10月26日
 *
 *  Author: ZhaoSir
 *******************************************************************************/
 
#ifndef __DEVICE_ACTION_H__
#define	__DEVICE_ACTION_H__
#include "stm32l0xx.h"
#include "stdbool.h"
#include "string.h"
#include "VL53L0x.h"
#include "Board_GPIO.h" 
#include "Board_Init.h" 


#define     OPEN_LED()              GPIO_SET_BIT(LED_PORT, LED_PIN, 0)
#define     CLOSE_LED()             GPIO_SET_BIT(LED_PORT, LED_PIN, 1)

#define     POWERDOWN_VL53L0x()     GPIO_SET_BIT(CC2640_RESET_PORT, CC2640_RESET_PIN, 0)
#define     POWERON_VL53L0x()       GPIO_SET_BIT(CC2640_RESET_PORT, CC2640_RESET_PIN, 1)



typedef struct 
{
	_VL53L0x_Adjust 	AdjustData;												// 传感器的家校准信息	
	uint8_t 			BluetoothStatus;										// 蓝牙的状态信息
	uint8_t 			AlarmDistance;											// 传感器的报警距离参数
	uint8_t 			SamplePeriod;											// 传感器的采样周期
    uint8_t             SystemStatus;
}DeviceInfomation;

extern			int 		UserCommandHandle(const uint8_t* CMD);
extern          bool        readVL53L0X_Data(uint16_t*  readdata);
extern          uint8_t     AlarmDistanceRead(void);

#endif

