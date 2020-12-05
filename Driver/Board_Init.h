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
#include "stdbool.h"
#include "string.h"
#include "Board_GPIO.h" 



extern		void 		Enter_LowPowerMode(void);
extern		void 		Quit_LowPowerMode(void);
extern		void 		Board_Init(void);
extern		void 		USART1_TX(const char* buff, uint16_t size);
extern		void 		USART2_TX(const char* buff, uint16_t size);
extern		uint32_t 	SYS_GetTick(void);
extern		void 		Systick_DelayUs(uint16_t us);
extern		void 		Systick_DelayMs(uint16_t us);
extern		bool 		USART2_RX_IsStart(void);
extern		void 		CompleteUSART2_RX(void);
extern		bool 		USART2_RX_IsComplete(void);




#endif

