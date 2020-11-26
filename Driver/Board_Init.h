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

extern		void 		Board_Init(void);
extern		void 		USART1_TX(const char* buff, uint16_t size);
extern		void 		USART2_TX(const char* buff, uint16_t size);
extern		uint32_t 	SYS_GetTick(void);
extern		void 		Systick_DelayUs(uint16_t us);
extern		void 		Systick_DelayMs(uint16_t us);
#endif

