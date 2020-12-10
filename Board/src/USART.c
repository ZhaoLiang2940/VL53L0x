/*******************************************************************************
 * USART.c
 *
 *  Created on: 2020?10?26?
 *
 *  Author: ZhaoSir
 *******************************************************************************/
#include "USART.h"
#include "string.h"


#define         USART2_RX_TIMEOUT           20


volatile 		bool   						USART2_RX_ERROR = false;
volatile 		bool   						USART2_RX_FLAG = false;
volatile		uint32_t					Timer2Count	= 0;					// 用于存储TIMER2的系统计数
static			uint32_t					USART_StartRX_Count	= 0;			// 用于存储开始接收数据时TIMER2计数
static			uint8_t						USART2_RX_Count = 0;				// 临时变量存储USART2的接收计数
volatile		uint8_t						USART2_RX_SIZE = 0;				    // 全局使用变量存储USART2的接收计数
static			uint8_t						USART2_RX_BUFF[100] = {0,0};		// 临时变量存储USART2的接收数据
volatile		uint8_t						USART2_BUFF_DATA[100] = {0,0};		// 全局使用变量存储USART2接收数据


static          uint32_t                    GET_USART_StartTIME(void);
static          void                        SET_USART_StartTIME(uint32_t time);
static          void                        USART2_TX( uint8_t* buff, uint16_t size);



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
void USART2_Init(void)
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
	NVIC_EnableIRQ(USART2_IRQn);
}


/********************************************************************************
*               Board_Init.c
*函数名称：	SendDistanceToBLE()
*
*函数作用：	像蓝牙设备发送距离信息
*
*参数说明：	distance：距离参数
*
*函数返回：	无
*
*函数作者：	ZhaoSir
********************************************************************************/
void SendDistanceToBLE(uint16_t distance)
{
    uint8_t TXBuff[7] = {0X5A, 0X00, 0X00, 0X00, 0XFE, 0XFE, 0XA5};
    TXBuff[1] = 0X00;
    TXBuff[2] = 0X00;
    TXBuff[3] = (distance >> 8) & 0XFF;
    TXBuff[4] = (distance >> 0) & 0XFF;
    TXBuff[5] = TXBuff[4] + TXBuff[3] + TXBuff[2] + TXBuff[1];
    USART2_TX(TXBuff, sizeof(TXBuff));
}

/********************************************************************************
*               Board_Init.c
*函数名称：	SendDistanceToBLE()
*
*函数作用：	像蓝牙设备发送距离信息
*
*参数说明：	distance：距离参数
*
*函数返回：	无
*
*函数作者：	ZhaoSir
********************************************************************************/
void SendCommandToBLE(uint8_t cmd, uint16_t cmdData)
{
    uint8_t TXBuff[7] = {0X5A, 0X00, 0X00, 0X00, 0XFE, 0XFE, 0XA5};
    TXBuff[1] = 0X01;
    TXBuff[2] = cmd;
    TXBuff[3] = (cmdData >> 8) & 0XFF;
    TXBuff[4] = (cmdData >> 0) & 0XFF;
    TXBuff[5] = TXBuff[4] + TXBuff[3] + TXBuff[2] + TXBuff[1];
    USART2_TX(TXBuff, sizeof(TXBuff));
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
static void USART2_TX( uint8_t* buff, uint16_t size)
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
    if(USART2->ISR & (0X1 << 5))
	{
        if(USART2_RX_Count < 100)
        {
			USART2_RX_BUFF[(USART2_RX_Count++) % 100] = USART2->RDR;
			SET_USART_StartTIME(HAL_GetTick());
        }
	}
    else
    {
        USART2->ICR |= 0XFFFFFFFF;
    }
}



/********************************************************************************
*               Board_Init.c
*函数名称：	GET_USART_StartTIME()
*
*函数作用：	获取USART2最近一次接收数据的事件
*
*参数说明： 无
*
*函数返回：	无
*
*函数作者：	ZhaoSir
********************************************************************************/
static uint32_t GET_USART_StartTIME(void)
{
	return USART_StartRX_Count;
}

/********************************************************************************
*               Board_Init.c
*函数名称：	SET_USART_StartTIME()
*
*函数作用：	设置USART2最近一次接收数据的事件
*
*参数说明： 无
*
*函数返回：	无
*
*函数作者：	ZhaoSir
********************************************************************************/
static void SET_USART_StartTIME(uint32_t time)
{
	USART_StartRX_Count = time;
	USART2_RX_FLAG = true;
}

/********************************************************************************
*               Board_Init.c
*函数名称：	CompleteUSART2_RX()
*
*函数作用：	结束一次USART2的接收数据处理，主要恢复一些用于接收数据的变量
*
*参数说明： 无
*
*函数返回：	无
*
*函数作者：	ZhaoSir
********************************************************************************/
bool USART2_RX_IsStart(void)
{
	 return USART2_RX_FLAG;
}

/********************************************************************************
*               Board_Init.c
*函数名称：	CompleteUSART2_RX()
*
*函数作用：	结束一次USART2的接收数据处理，主要恢复一些用于接收数据的变量
*
*参数说明： 无
*
*函数返回：	无
*
*函数作者：	ZhaoSir
********************************************************************************/
void CompleteUSART2_RX(void)
{
	USART2_RX_FLAG = false;	
	memcpy((void*)USART2_BUFF_DATA,USART2_RX_BUFF, USART2_RX_Count);
	memset(USART2_RX_BUFF,0, sizeof(USART2_RX_BUFF));
	USART2_RX_SIZE = USART2_RX_Count;
	USART2_RX_Count = 0;
}


/********************************************************************************
*               Board_Init.c
*函数名称：	USART2_RX_IsComplete()
*
*函数作用：	检测USART2接收数据是否完成
*
*参数说明： 无
*
*函数返回：	无
*
*函数作者：	ZhaoSir
********************************************************************************/
bool USART2_RX_IsComplete(void)
{
	return ((HAL_GetTick() - GET_USART_StartTIME()) > USART2_RX_TIMEOUT);
}



