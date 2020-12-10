/*******************************************************************************
 * USART.h
 *
 *  Created on: 2020?10?26?
 *
 *  Author: ZhaoSir
 *******************************************************************************/
 #ifndef    __USAHT_H__
 #define    __USAHT_H__  
 
 #include "main.h"
 
 extern         void                USART2_Init(void);
 extern         bool                USART2_RX_IsStart(void);
 extern         void                CompleteUSART2_RX(void);
 extern         bool                USART2_RX_IsComplete(void);
 extern         void                SendDistanceToBLE(uint16_t distance);
 extern         void                SendCommandToBLE(uint8_t cmd, uint16_t cmdData);

  
 #endif
 
 
