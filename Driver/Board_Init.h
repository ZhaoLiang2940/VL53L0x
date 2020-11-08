/*******************************************************************************
 * Board_Init.h
 *
 *  Created on: 2020��10��26��
 *
 *  Author: ZhaoSir
 *******************************************************************************/
 
#ifndef __BOARD_INIT_H__
#define	__BOARD_INIT_H__
#include "stm32l0xx.h"

extern		void 		Board_Init(void);
extern		void 		USART1_TX(const char* buff, uint16_t size);
extern		uint32_t 	SYS_GetTick(void);

#endif
