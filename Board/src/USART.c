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
volatile		uint32_t					Timer2Count	= 0;					// ���ڴ洢TIMER2��ϵͳ����
static			uint32_t					USART_StartRX_Count	= 0;			// ���ڴ洢��ʼ��������ʱTIMER2����
static			uint8_t						USART2_RX_Count = 0;				// ��ʱ�����洢USART2�Ľ��ռ���
volatile		uint8_t						USART2_RX_SIZE = 0;				    // ȫ��ʹ�ñ����洢USART2�Ľ��ռ���
static			uint8_t						USART2_RX_BUFF[100] = {0,0};		// ��ʱ�����洢USART2�Ľ�������
volatile		uint8_t						USART2_BUFF_DATA[100] = {0,0};		// ȫ��ʹ�ñ����洢USART2��������


static          uint32_t                    GET_USART_StartTIME(void);
static          void                        SET_USART_StartTIME(uint32_t time);
static          void                        USART2_TX( uint8_t* buff, uint16_t size);



/********************************************************************************
*               Board_Init.c
*�������ƣ�	USART1_Init()
*
*�������ã�	����1��ʼ����������115200
*
*����˵����	��
*
*�������أ�	��
*
*�������ߣ�	ZhaoSir
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
*�������ƣ�	SendDistanceToBLE()
*
*�������ã�	�������豸���;�����Ϣ
*
*����˵����	distance���������
*
*�������أ�	��
*
*�������ߣ�	ZhaoSir
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
*�������ƣ�	SendDistanceToBLE()
*
*�������ã�	�������豸���;�����Ϣ
*
*����˵����	distance���������
*
*�������أ�	��
*
*�������ߣ�	ZhaoSir
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
*�������ƣ�	USART1_TX()
*
*�������ã�	���ڷ��ͺ���
*
*����˵����	buff���������ݵ�ָ�룻
*			size���������ݵĴ�С��
*
*�������أ�	��
*
*�������ߣ�	ZhaoSir
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
*�������ƣ�	SysTick_Handler()
*
*�������ã�	Systick�жϷ�����
*
*����˵����	��
*
*�������أ�	��
*
*�������ߣ�	ZhaoSir
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
*�������ƣ�	GET_USART_StartTIME()
*
*�������ã�	��ȡUSART2���һ�ν������ݵ��¼�
*
*����˵���� ��
*
*�������أ�	��
*
*�������ߣ�	ZhaoSir
********************************************************************************/
static uint32_t GET_USART_StartTIME(void)
{
	return USART_StartRX_Count;
}

/********************************************************************************
*               Board_Init.c
*�������ƣ�	SET_USART_StartTIME()
*
*�������ã�	����USART2���һ�ν������ݵ��¼�
*
*����˵���� ��
*
*�������أ�	��
*
*�������ߣ�	ZhaoSir
********************************************************************************/
static void SET_USART_StartTIME(uint32_t time)
{
	USART_StartRX_Count = time;
	USART2_RX_FLAG = true;
}

/********************************************************************************
*               Board_Init.c
*�������ƣ�	CompleteUSART2_RX()
*
*�������ã�	����һ��USART2�Ľ������ݴ�����Ҫ�ָ�һЩ���ڽ������ݵı���
*
*����˵���� ��
*
*�������أ�	��
*
*�������ߣ�	ZhaoSir
********************************************************************************/
bool USART2_RX_IsStart(void)
{
	 return USART2_RX_FLAG;
}

/********************************************************************************
*               Board_Init.c
*�������ƣ�	CompleteUSART2_RX()
*
*�������ã�	����һ��USART2�Ľ������ݴ�����Ҫ�ָ�һЩ���ڽ������ݵı���
*
*����˵���� ��
*
*�������أ�	��
*
*�������ߣ�	ZhaoSir
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
*�������ƣ�	USART2_RX_IsComplete()
*
*�������ã�	���USART2���������Ƿ����
*
*����˵���� ��
*
*�������أ�	��
*
*�������ߣ�	ZhaoSir
********************************************************************************/
bool USART2_RX_IsComplete(void)
{
	return ((HAL_GetTick() - GET_USART_StartTIME()) > USART2_RX_TIMEOUT);
}



