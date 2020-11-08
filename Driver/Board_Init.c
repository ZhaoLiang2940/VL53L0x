/*******************************************************************************
 * Board_Init.c
 *
 *  Created on: 2020年10月26日
 *
 *  Author: ZhaoSir
 *******************************************************************************/

#include "Board_Init.h"
#include "Board_GPIO.h"

#define  		RTC_INT_PRIORITY           	((uint32_t)4U)    					/*!< RTC interrupt priority */
#define  		TICK_INT_PRIORITY           ((uint32_t)3U)    					/*!< tick interrupt priority */
#define  		USART1_INT_PRIORITY         ((uint32_t)2U)    					/*!< USART1 interrupt priority */
#define  		SYSCLOCK_FRE				4194304
static void 	USART1_Init(void);
static void 	SYSTICK_Init(void);
static void 	SYS_IncTick(void);
static void 	RTC_AWU_Init(void);

static __IO uint32_t uwTick;
bool	awuFlag = true;
/********************************************************************************
*               Board_Init.c
*函数名称：	Board_Init()
*
*函数作用：	板载相关硬件初始化
*
*参数说明：	无
*
*函数返回：	无
*
*函数作者：	ZhaoSir
********************************************************************************/
void Board_Init(void)
{
	GPIO_Init();
	USART1_Init();
	SYSTICK_Init();
	RTC_AWU_Init();
}


/********************************************************************************
*               Board_Init.c
*函数名称：	RTC_IRQHandler()
*
*函数作用：	RTC中断服务函数
*
*参数说明：	无
*
*函数返回：	无
*
*函数作者：	ZhaoSir
********************************************************************************/
void RTC_IRQHandler(void)
{
	if(RTC->ISR & (0X1 << 10)) 
	{
		RTC->ISR  &= ~(0X1 << 10);
		awuFlag = 1- awuFlag;
	}
}


/********************************************************************************
*               Board_Init.c
*函数名称：	SysTick_Handler()
*
*函数作用：	Systick中断服务函数
*
*参数说明：	无
*
*函数返回：	无
*
*函数作者：	ZhaoSir
********************************************************************************/
void SysTick_Handler(void)
{
	SYS_IncTick();
}

/********************************************************************************
*               Board_Init.c
*函数名称：	USART1_Init()
*
*函数作用：	串口1初始化，波特率115200
*
*参数说明：	无
*
*函数返回：	无
*
*函数作者：	ZhaoSir
********************************************************************************/
static void USART1_Init(void)
{
	uint32_t regTmp = 0;
	uint32_t Baud = 0;
	RCC->APB2ENR |= (0X1 << 14);
	RCC->IOPENR  |= 0X01;
	
	GPIOA->MODER &= ~(0X0F << 18);
	GPIOA->MODER |=  (0X0A << 18);
	
	GPIOA->AFR[1] &= ~(0XFF << 4);
	GPIOA->AFR[1] |=  (0X44 << 4);
	
	regTmp |= (0X1 << 5);														/* RX interrput enable */
	regTmp |= (0X1 << 3);														/* Transmitter is enable */
	regTmp |= (0X1 << 2);														/* Receiver Enable */
	USART1->CR1 = regTmp;
	
	Baud = SYSCLOCK_FRE / 9600;
	USART1->BRR = Baud;
	USART1->CR1 |= (0X1 << 0);													/* USRAT1 Enable */
	NVIC_SetPriority(USART1_IRQn,	USART1_INT_PRIORITY);
}



/********************************************************************************
*               Board_Init.c
*函数名称：	SYSTICK_Init()
*
*函数作用：	Systick初始化，用于延时
*
*参数说明：	无
*
*函数返回：	无
*
*函数作者：	ZhaoSir
********************************************************************************/
static void SYSTICK_Init(void)
{
	SysTick->LOAD = ((SYSCLOCK_FRE / 1000) - 1);
	NVIC_SetPriority (SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL); 			/* set Priority for Systick Interrupt */
	SysTick->VAL = 0UL;
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
					SysTick_CTRL_TICKINT_Msk   |
					SysTick_CTRL_ENABLE_Msk;
	
	NVIC_SetPriority(SysTick_IRQn,	TICK_INT_PRIORITY);
}

/********************************************************************************
*               Board_Init.c
*函数名称：	USART1_TX()
*
*函数作用：	串口发送函数
*
*参数说明：	buff：发送内容的指针；
*			size：发送内容的大小；
*
*函数返回：	无
*
*函数作者：	ZhaoSir
********************************************************************************/
static void RTC_AWU_Init(void)
{
	uint16_t x = 0X7FF;
	
	PWR->CR |=  (0X1 << 8);														// Disable backup write protection
	RCC->CSR |=  (0X1 << 0);													// Enable LSI clocks
	while(x-- & (!(RCC->CSR & 0X2)));
	RCC->CSR |=  (0X1 << 19);													// Reset RTC
	x = 0X7FF;
	while(x--);
	RCC->CSR &= ~(0X1 << 19);													// Finish Reset RTC
	
	RCC->CSR |=  (0X1 << 18);													// Enable RTC Clock
	
	RTC->WPR = 0XCA;
	RTC->WPR = 0X53;
	RCC->CSR &= ~(0X3 << 16);													// CLEAR_BIT
	RCC->CSR |=  (0X2 << 16);													// Config LSI oscillator clock used as RTC clock
	
	EXTI->IMR	|= (0X1 << 20);
	EXTI->RTSR  |= (0X1 << 20);
	EXTI->PR 	|= (0X1 << 20);
	
	RTC->CR &= ~(0X1 << 10);													// Disable Auto  Wakeup Timer
	while(!(RTC->ISR & 0X04)){};												// Waitting for access to config  Wakeup timer register
	RTC->CR &= ~(0X7 << 0);														// RTCCLK(LSI 37Khz) / 16 as Auto Wakeup time clock
	RTC->CR |=  (0X1 << 14);													// Eable Auto  Wakeup Timer interrupt
	RTC->WUTR = 2312-1;															// 37000/16/2312 = 1S
	RTC->CR |=  (0X1 << 10);
	NVIC_SetPriority(RTC_IRQn,	RTC_INT_PRIORITY);
	NVIC_EnableIRQ(RTC_IRQn);
}



/********************************************************************************
*               Board_Init.c
*函数名称：	USART1_TX()
*
*函数作用：	串口发送函数
*
*参数说明：	buff：发送内容的指针；
*			size：发送内容的大小；
*
*函数返回：	无
*
*函数作者：	ZhaoSir
********************************************************************************/
void USART1_TX(const char* buff, uint16_t size)
{
	while(size--)
	{
		while(!(USART1->ISR & (0X1 << 7))){}
			USART1->TDR = *buff++;
	}
	while(!(USART1->ISR & (0X1 << 7))){};
}
 

/********************************************************************************
*               Board_Init.c
*函数名称：	HAL_IncTick()
*
*函数作用：	设置系统节拍函数
*
*参数说明：	无
*
*函数返回：	无
*
*函数作者：	ZhaoSir
********************************************************************************/
static void SYS_IncTick(void)
{
	uwTick++;
}

/********************************************************************************
*               Board_Init.c
*函数名称：	HAL_GetTick()
*
*函数作用：	获取系统时钟节拍函数
*
*参数说明：	无
*
*函数返回：	无
*
*函数作者：	ZhaoSir
********************************************************************************/
uint32_t SYS_GetTick(void)
{
	return uwTick;
}






