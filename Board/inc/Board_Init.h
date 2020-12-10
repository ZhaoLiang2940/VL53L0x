/*******************************************************************************
 * Board_Init.h
 *
 *  Created on: 2020Äê10ÔÂ26ÈÕ
 *
 *  Author: ZhaoSir
 *******************************************************************************/
 
#ifndef __BOARD_INIT_H__
#define	__BOARD_INIT_H__
#include "stm32l0xx.h"


#define     SwitchBLEToSleep()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);   \
                                HAL_Delay(2500);                                        \
                                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); 
                                
                                
#define     SwitchBLEToActive() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);   \
                                HAL_Delay(50);                                           \
                                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);                                 

#define     SleepTime(x)        (uint32_t)(x*230)

extern      RTC_HandleTypeDef   RTCHandle;
extern		void 		        Enter_LowPowerMode(uint8_t sleepTiem);
extern		uint32_t            Exist_LowPowerMode(void);
extern		void 		        Board_Init(void);

extern		void 		        USART2_TX(uint8_t* buff, uint16_t size);
extern		uint32_t 	        SYS_GetTick(void);
extern		void 		        SystemClockConfig_STOP(void);

#endif

