/*******************************************************************************
 * Board_GPIO.c
 *
 *  Created on: 2020年11月1日
 *
 *  Author: ZhaoSir
 *******************************************************************************/
#include "Board_GPIO.h"



/********************************************************************************
*               Board_GPIO.c
*函数名称：	GPIO_Init()
*
*函数作用：	板载相关硬件初始化
*
*参数说明：	无
*
*函数返回：	无
*
*函数作者：	ZhaoSir
********************************************************************************/
void GPIO_Init(void)
{
    
	GPIO_InitTypeDef GPIO_InitStruct;
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    /*
     * 初始化完成后主动复位一次CC2640，点亮LED 1S
     */    
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Alternate = 1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); 
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1 , GPIO_PIN_RESET); 
    HAL_Delay(2000);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET); 
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1 , GPIO_PIN_SET); 
}


/********************************************************************************
*               Board_GPIO.c
*函数名称：	GPIO_SET_BIT()
*
*函数作用：	设置引脚输出状态
*
*参数说明：	gpiox：引脚端口号
*			Bitx：位号
*			Value：设置的值
*
*函数返回：	2：错误；
*
*函数作者：	ZhaoSir
********************************************************************************/
uint8_t GPIO_SET_BIT(GPIO_TypeDef *gpiox, uint8_t Bitx, bool Value)
{
	if((gpiox != GPIOA) && (gpiox != GPIOB)) return 2;
	if(Value)
		gpiox->BSRR |= (0X1 << Bitx);
	else
		gpiox->BSRR |= (0X1 << (Bitx + 16));
	return Value;
}

/********************************************************************************
*               Board_GPIO.c
*函数名称：	GPIO_GET_BIT()
*
*函数作用：	读取引脚状态
*
*参数说明：	gpiox：引脚端口号
*			Bitx：位号
*
*函数返回：	2：错误
*
*函数作者：	ZhaoSir
********************************************************************************/
uint8_t	GPIO_GET_BIT(GPIO_TypeDef *gpiox, uint8_t Bitx)
{
	uint32_t regVal = 0;
	if((gpiox != GPIOA) && (gpiox != GPIOB)) return 2;
	regVal = gpiox->IDR;
	
	if(regVal & (1 << Bitx)) return 1;
	else	return 0;
}



