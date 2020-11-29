/*******************************************************************************
 * Board_Init.c
 *
 *  Created on: 2020年10月26日
 *
 *  Author: ZhaoSir
 *******************************************************************************/

#include "Board_Init.h"
#include "Board_GPIO.h"

#define  		RTC_INT_PRIORITY           	((uint32_t)3U)    					/*!< RTC interrupt priority */
#define  		TICK_INT_PRIORITY           ((uint32_t)3U)    					/*!< tick interrupt priority */
#define  		USART1_INT_PRIORITY         ((uint32_t)2U)    					/*!< USART1 interrupt priority */
#define  		USART2_INT_PRIORITY         ((uint32_t)2U)    					/*!< USART1 interrupt priority */
#define  		SYSCLOCK_FRE				32000000
static void 	USART1_Init(void);
static void 	USART2_Init(void);
static void 	SYSTICK_Init(void);
static void 	SYS_IncTick(void);
static void 	RTC_AWU_Init(void);


volatile float 	systick_usFac = 0;
#ifndef LowPowerMode
static __IO uint32_t uwTick;
#else
static __IO uint32_t uwSecond;
#endif
bool	awuFlag = true;

/********************************************************************************
*               Board_Init.c
*函数名称：	SystemClock_Init()
*
*函数作用：	设置系统时钟；使用MSI作为HCLK和PCLK，使用4.194Mhz
*			LSE驱动RTC唤醒系统；
*
*参数说明：	无
*
*函数返回：	无
*
*函数作者：	ZhaoSir
********************************************************************************/
uint16_t SystemClock_Init(void)
{
	uint32_t regTmp = 	0;
	
	/* PREREAD_ENABLE */
	FLASH->ACR |= FLASH_ACR_PRE_READ;
	
	/* PREFETCH_ENABLE */
	FLASH->ACR |= FLASH_ACR_PRFTEN;
	FLASH->ACR |= FLASH_ACR_LATENCY;
	/* Enable Power Control clock */
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	
	RCC->CR &=  ~(0X1 << 24);
	/* Set voltage scaling1 */
	regTmp = PWR->CR;
	regTmp &= ~(0X03 << 11);
	regTmp |=  (0X01 << 11);
	PWR->CR = regTmp;
	
	regTmp = RCC->CFGR;
	regTmp &= ~(0X3 << 22);
	regTmp &= ~(0XF << 18);
	regTmp &= ~(0X7 << 11);
	regTmp &= ~(0X7 <<  8);
	regTmp &= ~(0XF <<  4);
	
	regTmp |=  (0X1 << 22);											// PLLout = PLLVCO / 2;  PLLVCO = 16/ 4 * 16 = 64Mhz;  PLL = 32Mhz
	regTmp |=  (0X5 << 18);											// PLLVCO = (HSI_16 / 4) * 16 = 64Mhz
	RCC->CFGR = regTmp;
	
	/* Enable HSI Clock, and wait ready */
	RCC->CR |= 0X08;
	RCC->CR |= 0X01;		
	uint16_t wx = 0XFFFF;
	/* 等待HSI稳定 */
	while((!(RCC->CR & (0X04))) && (wx--));							// Waitting HSI Ready
	if(wx == 0) return 1;
	
	RCC->CR |=  (0X1 << 24);												// Enable PLL
	wx = 0XFFFF;
	/* 等待PLL稳定*/
	while((!(RCC->CR & (0X1 << 25))) && (wx--));							// Waitting HSI Ready
	if(wx == 0) return 1;
	
	/* switch PLL to sysclock*/
	RCC->CFGR |= 0X3;
	wx = 0XFFFF;
	/* 等待PLL成为系统时钟 */
	while((!((RCC->CFGR & 0X0C) == 0X0C)) && (wx--));							// Waitting HSI Ready
	if(wx == 0) return 1;
	
	return 0;
}
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
	SystemClock_Init();
	GPIO_Init();
	USART2_Init();
	SYSTICK_Init();
	RTC_AWU_Init();
}

void ExistLP_Mode(void)
{
	SystemClock_Init();
	GPIO_Init();
	USART2_Init();
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
	uint32_t t = 0;
	if(RTC->ISR & (0X1 << 10)) 
	{
		EXTI->PR |= (0X1 << 20);												// 不清楚的话会一直触发中断
		RTC->ISR  &= ~(0X1 << 10);
		t = RTC->TR;
		t = RTC->DR;
		SYS_IncTick();
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
void USART2_IRQHandler(void)
{
	uint8_t tmp = 0;
	if(USART2->ISR & (0X1 << 5))
	{
		tmp = USART2->RDR;
	}
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
	RCC->CCIPR &= ~0X3;
	RCC->CCIPR |=  0X1;
	
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
static void USART2_Init(void)
{
	uint32_t regTmp = 0;
	uint32_t Baud = 0;
	RCC->APB1ENR |= (0X1 << 17);
	RCC->IOPENR  |= 0X01;
	
	GPIOA->MODER &= ~(0X0F <<  4);
	GPIOA->MODER |=  (0X0A <<  4);
	
	GPIOA->AFR[0] &= ~(0XFF << 8);
	GPIOA->AFR[0] |=  (0X44 << 8);
	
	regTmp |= (0X1 << 5);														/* RX interrput enable */
	regTmp |= (0X1 << 3);														/* Transmitter is enable */
	regTmp |= (0X1 << 2);														/* Receiver Enable */
	USART2->CR1 = regTmp;
	
	Baud = SYSCLOCK_FRE / 9600;
	USART2->BRR = Baud;
	USART2->CR1 |= (0X1 << 0);													/* USRAT1 Enable */
	NVIC_SetPriority(USART2_IRQn,	USART2_INT_PRIORITY);
	//NVIC_EnableIRQ(USART2_IRQn);
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
	systick_usFac  = SYSCLOCK_FRE / 1000000.0;
	SysTick->VAL = 0UL;
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk ;
}

/********************************************************************************
*               Board_Init.c
*函数名称：	Systick_DelayUs()
*
*函数作用：	利用Systick来进行us延时
*
*参数说明：	us：延时时长
*
*函数返回：	无
*
*函数作者：	ZhaoSir
********************************************************************************/
void Systick_DelayUs(uint16_t us)
{
	uint32_t systickVal = 0;
	uint32_t	tmp = 0;
	NVIC_DisableIRQ(SysTick_IRQn);
	systickVal = systick_usFac * us;
	SysTick->VAL = 0;
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk ;
	SysTick->LOAD = systickVal;
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
	
	do{
		tmp = SysTick->CTRL;
	}while((tmp&0x01) && (!(tmp&(1<<16))));
	SysTick->LOAD = 0;
	SysTick->CTRL &= ~(0X1) ;
}


/********************************************************************************
*               Board_Init.c
*函数名称：	Systick_DelayUs()
*
*函数作用：	利用Systick来进行us延时
*
*参数说明：	us：延时时长
*
*函数返回：	无
*
*函数作者：	ZhaoSir
********************************************************************************/
void Systick_DelayMs(uint16_t us)
{
	while(us)
	{
		us --;
		Systick_DelayUs(1000);
	}
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
	PWR->CR |=  (0X1 << 0);
	RCC->CSR |=  (0X1 << 19);													// Reset RTC
	
	x = 0X7FF;
	while(x--);
	RCC->CSR &= ~(0X1 << 19);													// Finish Reset RTC
	
	x = 0X7FF;
	RCC->CSR |=  (0X1 << 8);													// Enable LSI clocks
	while(x-- & (!(RCC->CSR & 0X200)));
	
	RCC->CSR |=  (0X1 << 18);													// Enable RTC Clock
	
	RTC->WPR = 0XCA;
	RTC->WPR = 0X53;
	RCC->CSR &= ~(0X3 << 16);													// CLEAR_BIT
	
	RCC->CSR |=  (0X1 << 16);													// Config LSI oscillator clock used as RTC clock
	
	EXTI->IMR	|= (0X1 << 20);
	EXTI->RTSR  |= (0X1 << 20);
	EXTI->PR 	|= (0X1 << 20);
	
	RTC->CR &= ~(0X1 << 10);													// Disable Auto  Wakeup Timer
	while(!(RTC->ISR & 0X04)){};												// Waitting for access to config  Wakeup timer register
	RTC->CR &= ~(0X7 << 0);														// RTCCLK(LSI 37Khz) / 16 as Auto Wakeup time clock
	RTC->CR |=  (0X1 << 14);													// Eable Auto  Wakeup Timer interrupt
	RTC->WUTR = 0X1000 - 1;														// 37000/16/2312 = 1S
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
void USART2_TX(const char* buff, uint16_t size)
{
	while(size--)
	{
		while(!(USART2->ISR & (0X1 << 7))){}
			USART2->TDR = *buff++;
	}
	while((!(USART2->ISR & (0X1 << 7))) || (!(USART2->ISR & (0X1 << 6)))){};
	USART2->ICR |= (0X3 << 6);
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
#ifndef LowPowerMode
	uwTick++;
#else
	uwSecond++;
#endif
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
#ifndef LowPowerMode
	return uwTick;
#else
	return  uwSecond;
#endif
}






